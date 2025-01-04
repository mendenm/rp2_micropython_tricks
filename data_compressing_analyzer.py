# data-compressing logic analyzer
# returns state changes on a selected set of pins
# using DMA and PIO, so it takes no CPU load when collecting data.
# because of the compression, it can monitor pins for a long time
# waiting for a transition, without filling up much memory.
# requires one DMA channel and one state machine with 14 instructions.
# requires at least v1.22 to get rp2.DMA

# author: Marcus Mendenhall mendenmh@gmail.com
#
# MIT License

# Copyright (c) 2025 Marcus Mendenhall

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Version 1.0 MHM 20231229

# import micropython as mp
from micropython import const
import machine
import rp2
import uctypes
import sys

from rp2 import DMA

_PIO0_base = const(0x50200000)
_PIO1_base = const(0x50300000)

DMA_SIZE_32 = const(2)


class compressing_analyzer:
    # this requires at least 11 clock cycles per pass,
    # so the time resolution is 11/frequency
    # the factor of 11 plays nicely with processor and clock frequencies
    # which are multiples of 11,
    # so 22 MHz, etc. gives nice round conversion factors for time,
    # and 110 MHz system clock is nice.
    # note that it works fine with any frequency, but the slightly
    # non-deterministic clock will introduce apparent jitter,
    # if the ratio of this clock to the system clock isn't integral.
    def __init__(self, sm_idx):
        # dummy PIO setup, will get overwritten
        # when the analyzer is configured
        def dumb_pio_pins():
            pass

        self.pio_prog = rp2.asm_pio()(dumb_pio_pins)

        self.sm_idx = sm_idx % 4
        self.pio_idx = sm_idx // 4
        # set up a passive state machine watching all pins
        self.sm = rp2.StateMachine(sm_idx, prog=self.pio_prog, in_base=Pin(0))
        self.pio = rp2.PIO(self.pio_idx)
        self.pio_base = _PIO0_base if not self.pio_idx else _PIO1_base
        dma = self.dma = DMA()
        fifo_addr = self.pio_base + 0x20 + 4 * self.sm_idx
        dreq = 8 * self.pio_idx + 4 + self.sm_idx  # 2.5.3.1. System DREQ Table
        ctrl = dma.pack_ctrl(size=DMA_SIZE_32, inc_write=1, inc_read=0, treq_sel=dreq)
        dma.config(read=fifo_addr, ctrl=ctrl)

    @micropython.native
    def done(self):
        return self.dma.count == 0

    def start_analyzer(
        self,
        freq_hz,
        max_transition_count,
        in_base=None,
        pin_count=32,
        initial_state=0,
        trigger_pin=None,
        trigger_polarity=1,
        extra_delay_cycles=0,
    ):
        buffer = self.buffer = bytearray(8 * max_transition_count)

        # compile the instruction
        # which will be stuffed to intialize the state machine
        self.initial_state = rp2.asm_pio_encode(f"set(x,{initial_state:d})", 0, 0)

        # build a convenience struct  to allow acces to data
        self.data_words = uctypes.struct(
            uctypes.addressof(self.buffer),
            (
                0 | uctypes.ARRAY,
                max_transition_count,
                {"data": (0 | uctypes.UINT32), "cycles": (4 | uctypes.UINT32)},
            ),
        )

        self.frequency = freq_hz
        # number of instructions after 'changed' minus 1
        _delay_cycles = const(3)
        self.loop_cycles = (
            8 + _delay_cycles + extra_delay_cycles
        )  # 8 instructions inside loop
        self.abort()  # make sure we aren't running
        sm = self.sm

        if self.pio_prog is not None:
            self.pio.remove_program(self.pio_prog)
            self.pio_prog = None

        def pio_pins():
            """build PIO program with compiled-in variables from our namespace"""
            # this uses osr as an extra register, since no data comes into here
            if trigger_pin is not None:
                # wait for toggle in the right direction on the trigger channel
                wait(1 - trigger_polarity, gpio, trigger_pin)
                wait(trigger_polarity, gpio, trigger_pin)
            wrap_target()
            label("loop")
            mov(y, osr)  # retrieve count from cache
            jmp(y_dec, "count")  # decrement time counter (can't increment!)
            label("count")
            mov(osr, y).delay(extra_delay_cycles)  # cache count
            mov(isr, null)  # clear shift register
            in_(pins, pin_count)
            mov(y, isr)  # get new word for comparison
            jmp(x_not_y, "changed")
            # keep cycle count the same by setting delay
            jmp("loop").delay(_delay_cycles)
            label("changed")
            mov(x, y)  # update current value
            push()  # first word is data, already in isr
            mov(isr, invert(osr))  # get current count
            push()  # push it
            wrap()

        self.pio_prog = pio_prog = rp2.asm_pio(
            autopush=False, autopull=True, in_shiftdir=rp2.PIO.SHIFT_LEFT
        )(pio_pins)
        sm.init(prog=pio_prog, freq=freq_hz, in_base=in_base)
        self.go()

    @property
    def time_step(self):
        return self.loop_cycles / self.frequency  # each loop is 11 cycles

    @property
    def loop_frequency(self):
        return self.frequency / self.loop_cycles

    def go(self):
        self.abort()  # make sure we are stopped before retstarting
        bufcnt = len(self.buffer) // 4
        while self.sm.rx_fifo():  # drain old data
            self.sm.get()
        self.bufcnt = bufcnt
        self.dma.config(write=self.buffer, count=bufcnt, trigger=True)
        self.sm.restart()  # back at  the beginning
        # we can queue up one put and one instruction to get executed
        # when the SM activates
        self.sm.put(0)  # initial counter, will get autopulled into OSR
        self.sm.exec(self.initial_state)  # set initial low 5 bits of state
        self.sm.active(1)

    @micropython.native
    def get_word_count(self):
        """number of transitions captured"""
        return (self.bufcnt - self.dma.count) // 2

    @micropython.native
    def pause(self, state=True):
        self.sm.active(0 if state else 1)

    def abort(self):
        self.sm.active(0)
        self.dma.active(0)

    def close(self):
        self.abort()
        if self.pio_prog is not None:
            self.pio.remove_program(self.pio_prog)
            self.pio_prog = None
        self.dma.close()
        del self.dma  # make sure we're broken after we close

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, exc_tb):
        self.close()


if __name__ == "__main__":
    # sample code which watches a PWM run
    # because of 11-cycle time for analyzer, multiples of 11 are nice
    freqtarget = 110_000_000
    machine.freq(freqtarget)
    print("setting frequency to ", freqtarget)

    from machine import Pin, PWM
    import time

    pwm = PWM(Pin(25))
    pwm.freq(20)
    pwm.duty_u16(65536 // 5)

    with compressing_analyzer(1) as a:  # let it clean up at end!
        a.start_analyzer(
            110_000_000,
            max_transition_count=200,
            pin_count=1,
            in_base=Pin(25),
            trigger_pin=None,
            trigger_polarity=1,
            extra_delay_cycles=0,
        )

        for i in range(20):
            if a.done():
                break
            print(a.get_word_count(), "  ", end="")
            time.sleep(0.05)

        pwm.deinit()
        print(a.get_word_count())
        print("{:10s}{:10s}{:10s}".format("    cycles", "    ms", "   data"))
        for i in range(a.get_word_count() // 2):
            print(
                f"{a.data_words[i].cycles:10d} {a.data_words[i].cycles*a.time_step*1000:10.4f}    {a.data_words[i].data:01x}"
            )
        print()
