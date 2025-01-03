from micropython import const
import uctypes
import asyncio
_PIO0_base = const(0x50200000)
_PIO1_base = const(0x50300000)
import machine

import rp2

#neopixel program using side-sets based on rp2040 manual section 3.6.2
#much more compact than previous version
#timings (6,7,7) are for 1 MHz shift frequency with 20 MHz PIO clock.  This is a cheat, since it should be 800  KHz.
#to get official 800 KHz, use (7,10,8) at 20 MHz
def neo_prog_side(T1=6, T2=7, T3=7):
    """define a pio program with the appropriate delays for the intended sm clock speed"""
    
    def func():   
        label("loop")
        out(x, 1).side(0).delay(T3-1)            # x <= left-most 1 bit of osr
        jmp(not_x, "zero").side(1).delay(T1-1)
        jmp("loop").side(1).delay(T2-1)
        label("zero")
        nop().side(0).delay(T2-1)       
        wrap()
    return func

class neopixel:
    pio32 = rp2.asm_pio(
        sideset_init=[rp2.PIO.OUT_LOW],
        out_shiftdir=rp2.PIO.SHIFT_LEFT,
        autopull = True,
        pull_thresh = 32
     ) ( neo_prog_side() ) #override these to change clock speeds, e.g.
    
    pio24 = rp2.asm_pio(
        sideset_init=[rp2.PIO.OUT_LOW],
        out_shiftdir=rp2.PIO.SHIFT_LEFT,
        autopull = True,
        pull_thresh = 24
     ) ( neo_prog_side() )

    pio8 = rp2.asm_pio(
        sideset_init=[rp2.PIO.OUT_LOW],
        out_shiftdir=rp2.PIO.SHIFT_LEFT,
        autopull = True,
        pull_thresh = 8
     ) ( neo_prog_side() )

    pio_freq = 20_000_000
    
    def __init__(self, pin: machine.Pin, string_length: int, colors: int, sm_idx: int = 0, buffer_count:int = 1, auto_switch: bool = False):
        import array
        from rp2 import DMA
        import rp2
        self.sm_idx = sm_idx % 4
        self.pio_idx = sm_idx // 4
        self.pio = rp2.PIO(self.pio_idx)
        self.pin = pin
        self.count = string_length
        self.colors = colors
        self.zeros = bytearray((4 if colors > 1 else 1)*string_length) #easy way to fill things with zeros
        self.buffers = [self.create_buffer() for _ in range(buffer_count)]
        self.auto_switch = auto_switch #switch to next buffer after write starts if True
        self.select_buffer(0)
        self.rgbwbuf = bytearray(4) 
        self.pio_base = _PIO0_base if not self.pio_idx else _PIO1_base
        
        #note: if we are running single-color (all-white) strings, buffer is bytes
        #otherwise, always 32-bit integers with either 3 or 4 colors per word
        self.pio_prog = self.pio32 if colors == 4 else self.pio24 if colors == 3 else self.pio8
        
        self.sm = sm = rp2.StateMachine(
                sm_idx,
                prog = self.pio_prog,
                freq=self.pio_freq,
                sideset_base = pin
            )
            
        dma = self.dma = DMA()
        fifo_addr = self.pio_base + 0x10 + 4 * self.sm_idx
        dreq = 8*self.pio_idx + 0 + self.sm_idx #2.5.3.1. System DREQ Table
        DMA_SIZE_32 = 2
        DMA_SIZE_8 = 0
        size = DMA_SIZE_32 if colors > 1 else DMA_SIZE_8
        ctrl = dma.pack_ctrl(size=size, inc_write=0, inc_read=1, treq_sel=dreq)
        dma.config(write = fifo_addr, ctrl = ctrl)
 
    def select_buffer(self, buf_idx : int):
        """make the indexed buffer the current one"""
        self.buf = self.buffers[buf_idx]
        self.current_buffer = buf_idx
        
    def next_buffer(self):
        """advance to the next buffer"""
        next_buffer = (self.current_buffer + 1) % len(self.buffers)
        self.select_buffer(next_buffer)
        
    def create_buffer(self):
        """ return an array.array of the same size as our internal buffers"""
        import array
        return array.array('L'if self.colors > 1 else 'B', self.zeros)
    
    def fill(self, index : (int | slice), data : (array.array | list | tuple | int), buf : array.array):
        """ store converted data into a buffer, use built-in if buf is None"""
        if buf is None:
            buf = self.buf #pre-bind for multiple references
        if type(index) is slice:
            if  isinstance(data, type(buf)): # already packed into ints, fast copy
                buf[index] = data
            else: #unpack likely list of RGB or RGBW lists
                convert = self.convert_rgbw #prebind for fast re-use
                start = index.start if index.start is not None else 0
                for i, d in enumerate(data):
                    buf[i+start] = convert(d)
        else:
            buf[index] = self.convert_rgbw(data)

    def __len__(self):
        return len(self.buf)
    
    def __setitem__(self, index : (int | slice), data : (array.array | list | tuple | int)):
        self.fill(index, data, self.buf)

    def start_write(self):
        """trigger a DMA transfer and, if appropriate, switch to the next buufer for next update"""
        sm = self.sm
        sm.active(1)
        sm.restart()
        self.dma.config(read = uctypes.addressof(self.buf), count = len(self.buf), trigger = True )
        if self.auto_switch:
            self.next_buffer()

    @micropython.native
    def get_word_count(self) -> int:
        """get number of words remaining"""
        return self.dma.count
    
    @micropython.native
    def check_write_done(self) -> True:
        """return True if last transfer complete"""
        return self.dma.count == 0
    
    async def write_async(self):
        self.start_write()
        while self.dma.count != 0:
            await asyncio.sleep_ms(1)

    @micropython.native
    def convert_rgbw(self, rgbw) -> int:
        #convert a tuple or list  to a packed 32 bit integer
        if type(rgbw) is int: #already converted!
            return rgbw
        ba = self.rgbwbuf #keep a pre-allocated 4-byte buffer to reduce object allocation
        ba[0] = rgbw[1]
        ba[1] = rgbw[0]
        ba[2] = rgbw[2]
        ba[3] = rgbw[3] if len(rgbw) == 4 else 0
        return int.from_bytes(ba,'big')

    def close(self):
        self.sm.active(0)
        self.dma.active(0)
        self.dma.close()
        del self.dma #make sure it can't be tried any more

if __name__=="__main__":
    from machine import Pin
    pin = Pin(0)
    neo = neopixel(pin=pin, string_length=1, colors=3, sm_idx=0)
    import random
    import time
    
    @micropython.native
    def rgb_hsv(h: int, s : int, v : int, buff = None):
        h = int(h) % 360
        u = h // 120
        b = h % 120
        a = 120 - b
        w = 120 * (255 - s) #unsaturated white component
        z = 120 * 256
        a = (a * s + w) * v // z
        b = (b * s + w) * v // z
        c = w * v // z
        if u == 0:
            r = a
            g = b
            b = c
        elif u ==  1:
            r = c
            g = a
            b = b
        else:
            r = b
            g = c
            b = a
            
        if buff is not None:
            buff[0] = g
            buff[1] = r
            buff[2] = b
            buff[3] = 0
            return int.from_bytes(buff,'big')
        else:
            return r, g, b
            
    loops = 0
    buff = bytearray(4)

    while not rp2.bootsel_button():
        neo[0] = rgb_hsv(int(loops), 255, 255, buff)
        neo.start_write()
        loops = (loops + 3) % 360
        while not neo.check_write_done():
            pass
        time.sleep(0.01)

        