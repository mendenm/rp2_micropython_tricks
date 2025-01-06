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

import uctypes
import rp2
import machine
import micropython
from micropython import const
import array
import time
from rp2 import PIO
import asyncio

#the ieee488 bus starts at Pin(4), and occupies the next 12 pins.
#the low 8 bits are the dio lines.
#the next 4 bits are the multiline-message control.

#all lines will be operated in 'pseudo-open-collector' mode, that is
#control will be maintained by setting all lines to zero, and using pindir
#in or out to control whether they are at high impedance or pulling down.
#there is significant online evidence that that  lines will tolerate (sort of)
#5V high-impedance sources when in input mode.

_PADCTRL = const(0x4001c000) #pad control registers

def set_drive(pad_idx, level):
    assert pad_idx >=1 and pad_idx <=32 and level >=0 and level <=3
    base = _PADCTRL + 4*(pad_idx + 1)
    machine.mem32[base] &= 0xffffffcf #clear current drive
    machine.mem32[base] |= (level & 3) << 4 #set  drive

def get_pad_reg(pad_idx):
    base = _PADCTRL + 4*(pad_idx + 1)
    return   machine.mem32[base]

class direct_drive_pins: # a mixin for case where data pins are directly driven, and handshakes are pseudo-OC

    _DATA_BASE = 4 #first data pin, 8 data pins must be consecutive, override in your class
    _CTRL_BASE = 18 #first control-line pin, 4 control loine pins must be consecutive, override in your class
    _DAV_IDX = const(0) #offset  from _CTRL_BASE to DAV
    _NRFD = const(1) #offset from CTRL_BASE to NRFD*
    _NDAC = const(2) #offset from CTRL_BASE to NDAC*
    _IFC = const(3) #IFC*
    _MIN_DELAY = const(5) #after pins set, before DAV  asserted, at 10 MHz, 500 ns settling

    _EOI_BIT = const(1<<9)
    _ATN_BIT = const(1<<11)

    #very quickly copy bytes, oring in ATN, etc.
    #note that data are output inverted, since we are in pseudo-OC mode,
    #and the bits well getting sent to the pindir register, and enabling output pulls down.
    #r0 = string address
    #r1 = word-buffer address
    #r2 = string length (not zero!)
    #r3 = extra bits to or in (typically ATN)
    #returns final output buffer address in r0
    #trims all zero bytes, since atn words shouldn't be null
    @micropython.asm_thumb()
    @staticmethod
    def expand_data(r0, r1, r2, r3) -> int:
        label(loop)
        ldrb(r4,[r0,0]) #grab byte
        orr(r4,r4) #test for zero
        beq (nextchar) #nulls aren't interesting in atn strings
        orr(r4, r3) #append any extra bits
        strh(r4, [r1,0]) #store half word
        add(r1,2) #bump words
        label(nextchar)
        add(r0,1) #bump string
        sub(r2,1)
        bne(loop)
        mov(r0,r1) #return current buffer pointer
        
    def setup_pins(self):
        
        #ieee-488 bus protocol:
        #initialize for talking with DAV active, NRFD and NDAC listening
        #send data as long as high bit is set in 16-bit blocks.
        #last  word will have high bit clear, then go to set up for listen.
        #Handshaking pins run in pseudo-OC mode, using pindirs to control pull-down
        #note that this must get created in setup_pins() to correctly absorb the right pin numbers from the class
        @rp2.asm_pio(pull_thresh = 16, autopull = True, autopush = False,
                     out_shiftdir = rp2.PIO.SHIFT_RIGHT,
                     in_shiftdir = rp2.PIO.SHIFT_LEFT, 
                     out_init = 12*(rp2.PIO.OUT_HIGH,),
                     set_init = 3*(rp2.PIO.OUT_HIGH,)) #3 handshaking/control lines
        def protocol():
            #start with bus in completely idle state, all pins disabled
            #but ready for tri-state drive on data lines, pseudo-OC drive on control lines
            wrap_target()
            set(pindirs, 0) #disable control lines at start
            set(pins, 0) #but leave control levels in low state so pseudo-OC mode pulls down on enable
            out(pindirs, 16) #get either 0x0000 or 0xffff from init_bus() to disable or enable data lines
            out(y, 16) #get this mask to allow ATN to flash on in read after one transer, as needed for spoll
            
            label('send_loop') #init done, go into talk mode
            set(pindirs, 0) #de-assert DAV if we are looping
            out(x,15)
            mov(pins, invert(x))[_MIN_DELAY] #can put data out while NRFD is still asserted
            out(x,1) #grab high bit which is end flag
            wait(1, gpio, _NRFD + _CTRL_BASE)
            set(pindirs, 1<<_DAV_IDX) #assert DAV
            wait(1, gpio, _NDAC +_CTRL_BASE) #wait for NDAC to be released
            jmp(not_x, 'send_loop') #and if the high bit was unset, go to keep talking
            
            set(pindirs, 1<<_NRFD) #de-assert DAV and assert NRFD at  the same time
            mov(osr, null) #make all data pins as inputs,since 'x' is zero here
            out(pindirs, 12) #don't out 16! that would autopull more data. we are only using 12 pins anyways
            label('listen loop')
            set(pindirs, 1<<_NDAC) #we're ready to listen, de-assert  NRFD* and assert  NDAC*
            wait(0, gpio, _CTRL_BASE + _DAV_IDX) #use absolute pin number
            in_(pins, 12) #grab the 12 bits of real bus data to test  EOI
            in_(null, 31-9) #shift EOI to high bit, all the rest will be zero
            mov(x, isr) #'x' will be zero if EOI is asserted at this pont (inverse logic...)
            in_(pins, 12) #re-grab the 12 bits of real bus data, don't push yet
            mov(osr, y) #if we are doing spoll, 'y' value will return ATN to active quickly, to prevent spoll data barf
            out(pindirs, 12)
            set(pindirs, 1<<_NRFD) #we've got the data...
            push() #now potentially stall
            wait(1, gpio, _CTRL_BASE + _DAV_IDX) #use absolute pin number, wait for DAV to be unasserted
            label('eoi')
            jmp(not_x, 'eoi') #just freeze up if EOI was asserted
            jmp(not_y, 'listen loop') #if we set ATN again, we are resetting the bus for 1-byte transfers
            wrap()

        self.basepin = machine.Pin(_DATA_BASE)
        self.ctrl_pins = machine.Pin(_CTRL_BASE)
        self.ifc = machine.Pin(_CTRL_BASE + _IFC, machine.Pin.OUT)

        self.ifc.on() #unasserted at start
        
        for i in range(_DATA_BASE, _CTRL_BASE + 4):
            machine.Pin(i, pull=machine.Pin.PULL_UP)
            set_drive(i,2 ) #medium drive

        self.sm = rp2.StateMachine(self.sm_idx, 
                protocol,
                freq = self.sm_frequency,
                in_base = self.basepin,  out_base = self.basepin,
                set_base = self.ctrl_pins, 
               ) #initialize PIOs

        self.sm.active(1)
        self.do_ifc()

    def do_ifc(self):
        self.ifc.off() #assert IFC
        time.sleep_us(200) #100 min required by 488.2
        self.ifc.on()

    @micropython.native()
    def init_bus(self, enable, sticky_atn):
        # put the bus in an idle state, waiting for 0xffff pushed to turn on data lines
        self.sm_restart()
        self.sm_put(0xffff if enable else 0) #first word in disables pins
        self.sm_put(_ATN_BIT if sticky_atn else 0) #if doing SPOLL, will raise ATN after one byte read
        
    @micropython.native()
    def emit_block(self, atn_bytes, data, do_eoi, end_atn, go_to_read):

        bufbase = uctypes.addressof(self.buffer)
        sm_put = self.sm_put
        expand_data = self.expand_data #bind to whatever mixin handles data processing
        l_atn = len(atn_bytes)
        l_data= len(data)
        l_end_atn = len(end_atn)
        buffer_memview = self.buffer_memview
        mem16 = machine.mem16
        
        #put atn string
        if l_atn:
            newbuf = expand_data(uctypes.addressof(atn_bytes), bufbase, l_atn, _ATN_BIT)
            if not (l_data + l_end_atn) and go_to_read: #no more, check for sticking on go-to-read
                mem16[newbuf-2] |= 0x8000
            sm_put(buffer_memview[:(newbuf-bufbase)//2]) #the memory view slices efficiently
        
        if l_data:
            #the data stream is always 8 bits, with upper bits zero, and will be sent as such by sm_put
            sliceable = memoryview(data)
            sm_put(sliceable[:-1])
            last = sliceable[-1] #this is converted to a real integer, and will have 16 bits
            if do_eoi:
                last |= _EOI_BIT
            if (not l_end_atn) and go_to_read:
                last |= 0x8000  
            sm_put(last)

        #put atn string
        if l_end_atn:
            newbuf = expand_data(uctypes.addressof(end_atn), bufbase, l_end_atn, _ATN_BIT)
            if go_to_read:
                mem16[newbuf-2] |= 0x8000
            sm_put(buffer_memview[:(newbuf-bufbase)//2]) #the memory view slices efficiently
                            
        while self.sm_tx_fifo():
            time.sleep_us(10) #drain the FIFO before returning
        return l_data

class base_ieee488: #requires a mixin for how the pins will be driven 

    sm_frequency = 10_000_000
    async_mode = False
    
    def __init__(self, sm_idx = 0, inbufsize = 256, address = 1, frequency = None):
        #the output buffer will be allocated with some reasonable initial size,
        #and expanded if necessary
        #build buffer array from generator object to avoid extra  allocations
        if frequency is not None:
            self.sm_frequency = frequency
            
        if self.async_mode:
            self.bus_lock = asyncio.Lock()
        else:
            import _thread
            self.bus_lock=_thread.allocate_lock()
        
        #buffer is used for (typically short) outputs
        self.buffer = array.array('H', 0 for _ in range(8)) #small, fixed buffer for ATN-type headers 
        self.buffer_memview = memoryview(self.buffer) #easily and quickly sliced
        #in_buffer is used for reads by default, but user can provide another
        self.in_buffer = array.array('B', 0 for _ in range(inbufsize))
        self.in_buffer_bytes = memoryview(
            uctypes.bytearray_at(uctypes.addressof(self.in_buffer), inbufsize))
        self.header_buffer = bytearray(5) #for MTA, MLA & secondary addresses
        self.header_bp = uctypes.addressof(self.header_buffer)
        
        self.read_loop_results = array.array("I", (0,0,0)) #preallocated to avoid tuples
        
        self.my_address = address
        self.sm_idx = sm_idx
        
        self.setup_pins()
                
        self.sm_restart = self.sm.restart #partly pre-bind
        self.sm_put = self.sm.put #ditto
        self.sm_get = self.sm.get #ditto
        self.sm_rx_fifo = self.sm.rx_fifo
        self.sm_tx_fifo = self.sm.tx_fifo

        self.init_bus(False, False)

    def __enter__(self): #from: https://github.com/peterhinch/micropython-async/blob/master/v3/docs/TUTORIAL.md
        self.bus_lock.acquire() 
        return self

    def __exit__(self, *args):
        if args[0] is not None: #any exception should reset  the state...
            self.do_ifc() 
            self.init_bus(False, False)
        self.bus_lock.release() 
    
    def pack_address(self, address: int | (int, int)) -> int:
        if type(address) is int:
            return address
        else:
            return address[1]  << 16 | address[0]
    
    @micropython.viper
    #note:  secondary addressing is in the MSB of the address
    def fill_header(self, mode: int, talk_addr: int, listen_addr: int): 
        bp = ptr8(int(self.header_bp))
        bp[0] = mode #may be something like SPE
        a =int(listen_addr)
        bp[1] = (a & 0x1f) + 0x20
        secondary = (a >> 8) & 0x1f
        bp[2] = secondary  + (0x60 if secondary else 0)
        a = int(talk_addr)
        bp[3] = (a & 0x1f) + 0x40
        secondary = (a >> 8) & 0x1f
        bp[4] = secondary  + (0x60 if secondary else 0)
    
    def chunk_write(self, address, data, starting, ending, do_eoi, timeout_ms):
        if starting : #starting a new transfer with addressing
            self.init_bus(True,False)
            self.fill_header(0, self.my_address, address)
            atn_bytes = self.header_buffer
        else: #probably continuing a previously-started transfer
            atn_bytes = ""
 
        if ending:
            end_atn = "\x5f\x3f" #UNT untalk then UNL unlisten
        else:
            end_atn = ""
        
        wordcount = self.emit_block(atn_bytes, data, do_eoi, end_atn, 0)
        
        if ending:
            self.init_bus(False, False) #disable bus
            
        return len(data)

    @micropython.viper
    def readloop(self, fifo, getter, buffer, maxlen : int, endchar: int):
        if maxlen < 4:
            raise RuntimeError("cannot read into buffer size less than 4 bytes")
        i = 0
        ec = int(endchar)
        eoi = False
        term = False
        limit1 =  maxlen - 4 #shrink buffer to leave room for FIFO data
        x = int(abs(ec)) + 1 #this should never match ec, nor have EOI set
        while i < maxlen: #leave room in buffer for fifo overflow
            stopped = (i == limit1)
            if stopped:
                self.init_bus(False, False) #no more data transfers, if the buffer is nearly full
            while 1:
                nfifo = int(fifo())
                if nfifo != 0: break
                #only test  for end conditions if FIFO is (at least temprarily) drained
                eoi = (x & _EOI_BIT) != 0
                term = (x & 0xff) == ec #will never match if endchar < 0
                if eoi or term: break #always end if EOI asserted
                time.sleep_ms(1) #we should only hit this on very slow devices, and it allows KeyboardInterrupts, etc.
            if (stopped and nfifo == 0) or eoi or term: break 
            x = int(getter()) ^ 0xffff #bus logic is inverted
            buffer[i] = x
            i = i + 1

        rb = self.read_loop_results
        rb[0] = i
        rb[1] = eoi
        rb[2] = term
                                  
    @micropython.viper
    def drain_fifo(self):
        fifo = self.sm_rx_fifo #bind for speed
        g = self.sm_get #bind for speed
        while int(fifo()): g()
    
    @micropython.native
    def chunk_read(self, address, buffer, starting, ending, endchar, timeout_ms) :
        #give this a sliced-memoryview buffer to set the maximum read length
        
        if starting : #starting a new transfer with addressing
            self.init_bus(True, False)
            self.fill_header(0, address, self.my_address)                                    
            self.emit_block(self.header_buffer, "", False, "", 1)
        
        if type(endchar) is str: endchar = ord(endchar)

        self.readloop(self.sm_rx_fifo, self.sm_get, buffer, len(buffer), endchar)
                
        if ending:
            self.init_bus(True, False)
            self.emit_block(b"\x5f\x3f", "", False, "", 0) #untalk and unlisten
            self.init_bus(False, False) #disable bus
            
    def read(self, addr, buffer = None, max_count = None, timeout = None, endchar = -1):
        #read into a buffer.  The user is reponsible for slicing and converting the buffer
        if buffer is None:
            if max_count is not None:
            #use internal buffer, but sliced to length
                buffer = self.in_buffer_bytes[:max_count]
            else:
                buffer = self.in_buffer
                
        with self:
            self.chunk_read(addr, buffer, True, True, endchar, timeout)
        
        rb = self.read_loop_results #has data stored from read loop
        return (rb[0], buffer, rb[1], rb[2])
    
    def write(self, addr, data, timeout = None):
        with self:
            count = self.chunk_write(addr, data, True, True, True, 0)
        return count

    def transaction(self, addr, data, buffer = None, max_count = None, endchar = -1, timeout = None):
        if buffer is None:
            if max_count is not None:
            #use internal buffer, but sliced to length
                buffer = self.in_buffer_bytes[:max_count]
            else:
                buffer = self.in_buffer

        with self:
            self.chunk_write(addr, data, True, True, True, 0)
            self.chunk_read(addr, buffer, True, True, endchar, timeout)
            
        rb = self.read_loop_results #has data stored from read loop
        return (rb[0], buffer, rb[1], rb[2])
    
    #@micropython.native
    def spoll(self, addresses):
        #spoll requires a setup and then a bunch of bus switches
        with self:
            polls = bytearray(len(addresses))
            idx = 0
            #pre-bind names for faster access 
            hb = self.header_buffer
            my_address = self.my_address
            sm_get = self.sm_get
            fill_header = self.fill_header
            init_bus = self.init_bus
            emit_block = self.emit_block
            for addr in addresses:
                fill_header(0x18, addr, my_address)
                init_bus(True, True) #special mode to hold ATN during read, and only read 1 byte, then halt
                emit_block(hb, "", False, "", True) #send SPE header to enable poll
                polls[idx]= (sm_get() & 0xff) ^ 0xff #grab one byte off the fifo only.
                idx = idx + 1
                my_address = 0 #no point sending it after first round
            self.init_bus(True, False) #allow write here
            self.emit_block(b"\x19\x5f\x3f", "", False, "", False) #send spd header to disable poll
            self.init_bus(False, False)
            
        return polls

class ieee488(base_ieee488, direct_drive_pins):
    pass

class ieee488_async(ieee488):
    
    async_mode = True #will allocate bus locks with asyncio, etc.
    
    async def __aenter__(self): #from: https://github.com/peterhinch/micropython-async/blob/master/v3/docs/TUTORIAL.md
        await self.bus_lock.acquire()  # a coro defined with async def
        return self

    def __aexit__(self, *args):
        if args[0] is not None: #any exception should reset  the state...
            self.do_ifc() 
            self.init_bus(False, False)
        self.bus_lock.release() 

    def emit_block(self, atn_bytes, data, do_eoi, end_atn, go_to_read):
        wordcount = buildit(self.buffer,
                atn_bytes, data, do_eoi, end_atn, go_to_read)
        self.sm.put(self.buffer_memview[:wordcount]) #the memory view slices efficiently
        while self.sm.tx_fifo():
            await asyncio.sleep_ms(0) #drain the FIFO before returning
        return wordcount
    
    def chunk_write(self, address, data, starting, ending, do_eoi, timeout_ms):
        if starting : #starting a new transfer with addressing
            self.init_bus(True,False)
            self.fill_header(0, self.my_address, address)
            atn_bytes = self.header_buffer
        else: #probably continuing a previously-started transfer
            atn_bytes = ""
 
        if ending:
            end_atn = "\x5f\x3f" #UNT untalk then UNL unlisten
        else:
            end_atn = ""
        
        wordcount = await self.emit_block(atn_bytes, data, do_eoi, end_atn, 0)
        
        if ending:
            self.init_bus(False, False) #disable bus
            
        return len(data)

    @micropython.native
    def readloop(self, fifo, getter, buffer, maxlen : int, endchar: int):
        i = 0
        ec = int(endchar)
        while i < maxlen:
            while  int(fifo()) == 0:
                await asyncio.sleep_ms(0) #we should only hit this on very slow devices, and it allows KeyboardInterrupts, etc.
            x = int(getter()) ^ 0xffff #bus logic is inverted
            buffer[i] = x
            i = i + 1
            eoi = (x & _EOI_BIT) != 0
            term = (x & 0xff) == ec #will never match if endchar < 0
            if eoi or term: break #always end if EOI asserted

        rb = self.read_loop_results
        rb[0] = i
        rb[1] = eoi
        rb[2] = term

    @micropython.native
    def chunk_read(self, address, buffer, starting, ending, endchar, timeout_ms) :
        #give this a sliced-memoryview buffer to set the maximum read length
        
        if starting : #starting a new transfer with addressing
            self.init_bus(True, False)
            self.fill_header(0, address, self.my_address)                                    
            await self.emit_block(self.header_buffer, "", False, "", 1)
        
        if type(endchar) is str: endchar = ord(endchar)

        await self.readloop(self.sm_rx_fifo, self.sm_get, buffer, len(buffer), endchar)
                
        if ending:
            self.init_bus(True, False)
            await self.emit_block(b"\x5f\x3f", "", False, "", 0) #untalk and unlisten
            self.init_bus(False, False) #disable bus
            
        return count, buffer, eoi, term

    async def read(self, addr, buffer = None, max_count = None, timeout = None, endchar = -1):
        #read into a buffer.  The user is reponsible for slicing and converting the buffer
        if buffer is None:
            if max_count is not None:
            #use internal buffer, but sliced to length
                buffer = self.in_buffer_bytes[:max_count]
            else:
                buffer = self.in_buffer
                
        async with self:
            self.chunk_read(addr, buffer, True, True, endchar, timeout)        
            rb = self.read_loop_results #has data stored from read loop
            return (rb[0], buffer, rb[1], rb[2])

    async def write(self, addr, data, timeout = None):
        async with self:
            count = self.chunk_write(addr, data, True, True, True, 0)
        return count

    async def transaction(self, addr, data, buffer = None, max_count = None, endchar = -1, timeout = None):
        async with self:
            if buffer is None:
                if max_count is not None:
                #use internal buffer, but sliced to length
                    buffer = self.in_buffer_bytes[:max_count]
                else:
                    buffer = self.in_buffer

            self.chunk_write(addr, data, True, True, True, 0)
            self.chunk_read(addr, buffer, True, True, endchar, timeout)
            
            rb = self.read_loop_results #has data stored from read loop
            return (rb[0], buffer, rb[1], rb[2])
    
class gpib_device():
    """completely synchronous adapter that might use underlying async bus"""

    def __init__(self, bus: ieee488, address: int | (int, int),
                 default_timeout = None):
        self.bus = bus
        self.address = bus.pack_address(address)
        
        self.default_timout = default_timeout
        self.asyn = bus.async_mode

    def read(self, **kwargs):
        #simple, synchronous read
        if self.asyn:
            return asyncio.run(self.bus.read(self.address, **kwargs))
        else:
            return self.bus.read(self.address, **kwargs)
        
    def write(self, data, **kwargs):
        #simple, synchronous write
        if self.asyn:
            return asyncio.run(self.bus.write(self.address, data, **kwargs))
        else:
            return self.bus.write(self.address, data, **kwargs)
        
    def transaction(self, data, **kwargs):
        #simple, synchronous write
        if self.asyn:
            return asyncio.run(self.bus.transaction(self.address, data, **kwargs))
        else:
            return self.bus.transaction(self.address, data, **kwargs)

    def ascii_unpack_read(self, read_result):
        return str(read_result[1][:read_result[0]],'ascii')

    def get_scpi_error(self):
        err = self.transaction(":syst:error?");
        res = str(err[1][:err[0]],'ascii')
        errval , errstr = res.split(",")
        return int(errval), errstr.strip()[1:-1]
        
if __name__ == "__main__":

    f = machine.freq()
    f0 = 100_000_000
    if f != f0:
        machine.freq(f0)
        print("resetting freq")
        machine.soft_reset()
          
if __name__ == "__main__":

    import asyncio
    
    global analyzer
    def  dump_analyzer(t):
        a = analyzer
        n0 = 0
        dt = a.time_step()*1000 #working in ms
        for i in range(a.get_word_count()):
            n = a.data_words[i].cycles
            v = a.data_words[i].data
            data = (v & 0xfff) ^ 0xfff #un-invert  bus data
            ctrl = (v >> 14) & 0x7 #control lines
            print(f"{n*dt:10.2f} {(n-n0)*dt*1000:10.2f} {ctrl:03b} {data:03x}")
            n0 = n
            
    def setup_analyzer(bus, wait_time_ms = 2000):
        from data_compressing_analyzer import compressing_analyzer
        analyzer = a = compressing_analyzer(4) #need to be on second PIO; first one is getting full
        a.start_analyzer(freq_hz =machine.freq(), max_transition_count = 100,
             in_base = bus.basepin, pin_count = 22,
             ) # trigger_pin = _DAV_IDX + _CTRL_BASE, trigger_polarity = 0)
        a.pause(True) #not really running yet
    
        from machine import Timer
        timer = Timer(mode=Timer.ONE_SHOT, period = wait_time_ms,
                      callback = lambda t: micropython.schedule(dump_analyzer, t))
        return a

if __name__ == "__main__":
    bus = ieee488(0, frequency = 20_000_000)
    dev = gpib_device(bus, 5)
    analyzer = setup_analyzer(bus, 2000)
    analyzer.pause(False)
    dev.write("*rst")
    dev.write("*cls")
    dev.write ("*ese 255") #enable all errors in status byte
    dev.write("*idn?")
    count1, buffer1, eoi1, term1 = dev.read(endchar='\n')
    print(count1, eoi1, term1, str(buffer1[:count1],'ascii').strip())
    count, buffer, eoi, term = dev.transaction("*idn?")
    print(count, eoi, term, str(buffer[:count],'ascii').strip())
    print("before error", bus.spoll((5,))[0])
    dev.write("ugh...") #create an error
    time.sleep(0.1)
    print("after error", bus.spoll((5,))[0])
    dev.write("*cls")
    time.sleep(0.5) #this device takes a while to update spoll results from status changes
    print("cleared error", bus.spoll((5,))[0])
    
    if 1:
        dt0 = time.ticks_us()
        spoll = bus.spoll((5,)*100)
        dt1 = time.ticks_us()
        print((dt1-dt0)/len(spoll), "us per spoll", spoll[0])

    
