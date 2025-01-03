#neopixel ring operations

            
import machine
from machine import Pin
import random
import time
import array
import rp2
import gc
import asyncio
from time import ticks_ms, ticks_us, ticks_add, ticks_diff

machine.freq(120_000_000)

_PADCTRL = const(0x4001c000) #pad control registers

def set_drive(pad_idx, level):
    assert pad_idx >=1 and pad_idx <=32 and level >=0 and level <=3
    base = _PADCTRL + 4*(pad_idx + 1)
    machine.mem32[base] &= 0xffffffcf #clear current drive
    machine.mem32[base] |= (level & 3) << 4 #set  drive

if 1:
    #init all pins
    for i in range(1, 30):
        Pin(i, Pin.OUT)
        set_drive(i, 0) #min drive

from rp2_neopixel import neopixel

@micropython.native
def rgb_hsv(h: int, s : int, v : int) -> int:
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

    return g | (r << 8) | (b << 16) #return packed int for 3-color neopixels


#keep a copy of the current color configuration, for overlaying shooting stars

run_flag = True
lights_state = "off"

@micropython.viper
def weighted_sum(in1, in2, out, w1: int, w2: int):
    nn = int(len(out))*4
    p1 = ptr8(in1)
    p2 = ptr8(in2)
    p3 = ptr8(out)
    scale = 65536 // (w1 + w2)
    w1 = w1 * scale
    w2 = w2 * scale
    idx = 0
    while idx < nn:        
        p3[idx] = (w1 * p1[idx] + w2 * p2[idx]) >> 16
        idx += 1

thread_state = None

def main_colors(strings, sleeptime_ms):
    global thread_state
    thread_state = "running"
    
    import array
    buffers = [array.array('L', s.buf2) for s in strings]
    newcolors = [None if not getattr(s,'smooth',False) else array.array('L', s.buf2) for s in strings]            
    color_arrays = [ [s.neo.convert_rgbw(x) for x,y in s.color_table for _ in range(y)] for s in strings]
    programs = tuple(zip(buffers, newcolors, strings, color_arrays))
    loops = 0
    
    while run_flag:
        for mainbuf, newcolor, string, table in programs:
            replacements = string.replacements
            neo_count = len(mainbuf)
            color_count = len(table)
            
            if newcolor is not None:
                for i in range(replacements):
                    idx = random.randrange(neo_count)
                    newcolor[idx] = table[random.randrange(color_count)]
                weighted_sum(mainbuf, newcolor, mainbuf, string.oldweight,  1)
            else:
                for i in range(replacements):
                    light = random.randrange(neo_count)
                    color = table[random.randrange(color_count)]
                    mainbuf[light] = color
                
            string[:] = mainbuf #locking handled by string
        
        time.sleep_ms(sleeptime_ms)
        loops = loops + 1
        thread_state = loops
        
    thread_state = "done"
    
class star_string:

    star = ( (0,0,32), (0,0,64), (0,0,128), (0,0,255), (0,128,0), (128, 0, 0), (255,255,255), (0,0,0), )
    update_ms = 50
    star_fraction = 100
    
    def __init__(self, sm_idx, neo_pin_idx, neo_count, max_stars=5):
        self.pin = Pin(neo_pin_idx)
        #set_drive(neo_pin_idx+1, 3) #max drive
        self.neo = neo = neopixel(self.pin,neo_count,3, sm_idx = sm_idx, buffer_count = 2, auto_switch = True)
        self.buf2 = neo.create_buffer()
        import _thread
        self.data_lock = _thread.allocate_lock()
        self.star_queue = []
        self.max_stars = max_stars
        self.star_tasks = [self.star_task(i) for i in range(max_stars)]
    
    def __len__(self):
        return len(self.neo)
    
    def __setitem__(self, index, data):
        #copy a buffer to our buf2, under thread lock
        if type(index) is slice:
            #mv = memoryview(self.buf2)[index]
            self.data_lock.acquire()
            self.buf2[index] = data
            self.data_lock.release()
        else:
            self.data_lock.acquire()
            self.buf2[index] = self.neo.convert_rgbw(data)        
            self.data_lock.release()
    
    def star_task(self, idx):        
        pix = self.neo
        data_lock = self.data_lock
        buf2 = self.buf2
        q = self.star_queue
                
        while 1:
            while not len(q): #just keep yielding until a queue entry is available
                yield
            starbytes, backstarbytes = q.pop()
            if starbytes is None: break #flag for shutdown
            
            ll = len(starbytes)
                            
            maxstep = len(buf2) - ll

            #start propagating a star
            flip = random.randrange(2) == 0
            if 0:
                speed = int((random.randrange(4, 10)**2) / 10) #bias towards fast stars
                if speed < 5:
                    step = 4
                elif speed < 10:
                    step = 2
                else:
                    step = 1
            else:
                step = int(random.randrange(1,10))
                speed = 1
                
            print(f"{idx:d} go {speed:d} {step:d} {flip:d}!")
            for i in range(0, maxstep, step):                
                if not run_flag: break
                base = i if not flip else maxstep - i
                for _ in range(speed):
                    pix[base: base + ll] = starbytes if not flip else backstarbytes
                    yield
                    
            print(f"{idx:d} done")
            
            if not run_flag: break
            
    async def run_stars(self):

        pix = self.neo
        buf2 = self.buf2
        data_lock = self.data_lock

        if self.star:
            starbytes = array.array('L', pix.convert_rgbw(x) for x in self.star )
            backstarbytes = array.array('L', reversed(starbytes))
            star_pair = (starbytes, backstarbytes)
            print(starbytes, backstarbytes, self.star_fraction)
        else:
            starbytes = None
        
        
        last_ms = ticks_ms()
        
        while run_flag:
            if  lights_state != "on":
                for i in range(len(pix.buf)):
                    pix[i] = 0 #may as well do this slowly... no point with a fast-copy buffer
                await pix.write_async()
                await asyncio.sleep(30) #waiting for night
                continue
            
            elif starbytes is not None and len(self.star_queue) < self.max_stars and random.randrange(self.star_fraction) == 0:
                #queue up a star
                #print("queued")
                self.star_queue.append(star_pair)
            
            pix.start_write() #start the write in current buffer, and auto-switch to next
            
            #refresh the neopixels from the primary color buffer 'buf2'
            t0  = ticks_us()
            data_lock.acquire()
            pix[:] = buf2
            data_lock.release()
            dt_copy = ticks_diff(ticks_us(), t0)
            
            #now, tell all the star tasks to propagate a star into the buffer, if they have one
            for task in self.star_tasks:
                next(task)

            gc.collect() #nothing better to do in spare time...

            while not pix.check_write_done(): #should probably be done long before the next refresh time
                await asyncio.sleep_ms(2)
            
            # compute delays in phase-stable way
            # note that this will eventually fail if the phase stability cannot be maintained
            # because too much time is being spent preparing data
            next_ms = ticks_add(last_ms, self.update_ms)
            wait_time = min(max(ticks_diff(next_ms, ticks_ms()), 1), self.update_ms)
            #print(dt_copy, last_ms, next_ms, wait_time)
            last_ms = next_ms
            await asyncio.sleep_ms(wait_time)
           
        self.star_queue[:]=[(None, None)]*self.max_stars #None is a sentinel to terminate

light_thresh = const(20000) # evening detection threshold
adc_pin = const(26)

if 1:
    on_time = 5 * 3600 # seconds after trigger
    night_time = 12 * 3600 # always off at least this long after running
    debounce_time = 60
else:
    on_time = 20 # seconds after trigger
    night_time = 60 # always off at least this long after running
    debounce_time = 5
    
async def watch_light_level():
    global lights_state
    
    from machine import ADC, Pin
    import time
    
    adc = ADC(Pin(adc_pin)) #ADC0
    last_change = time.time()
    # if it's already dark when we power up, assume night
    # this prevents the system from starting up on ion the middle of the night
    # after  a power outage
    lights_state = "off" if adc.read_u16() > light_thresh else "night"
    
    while run_flag:
        val = adc.read_u16()
        print("light level", val)
        low = val < light_thresh
        now = time.time()
        dt = now - last_change
        print("lights:", val, lights_state, dt)
        
        if lights_state == "off" and low:
            #starting countdown check for low light
            lights_state = "debounce_low"
            last_change = now
        elif lights_state == "debounce_low":
            if not low:
                lights_state = "off" #not low long enough, try again
            elif dt > debounce_time:
                last_change = now
                lights_state = "on"
        elif lights_state == "on":
            if dt > on_time: # run for specified time
               lights_state = "night"
               last_change = now
        elif lights_state == "night" and dt > night_time:
            # wait until well past morning before resetting
            lights_state = "off"
        await asyncio.sleep(1)

    Pin(26, Pin.IN) # reset ADC pin
    
def xmas(timer=True):
    
    xmas_color_table = [
        [[255,0,0], 10],
        [[255,255,0], 1],
        [[0,255,0], 10],
        [[0,255,255], 1],
        [[0,0,255], 2],
        [[255,0,255], 1],
#        [[255,255,255],1], 
        [[0,0,0], 50],
    ]

    strings = []
    for sm, p, n in ( (0, 17, 900),  ):
        s = star_string(sm, p, n)
        s.smooth = True
        s.replacements = 20
        s.color_table = xmas_color_table
        s.oldweight = 15
        s.star_fraction = len(s) // 8
        strings.append(s)
        
    import _thread
    thr = _thread.start_new_thread(main_colors, (strings, 100) )
    #main_colors(strings, 100)
    global run_flag, lights_state
    if timer:
        lights_state = "off" # using lighting control, start "off"
        lights = asyncio.create_task(watch_light_level())
    else:
        lights_state = "on"
    stars = [asyncio.create_task(s.run_stars()) for s in strings]

    led = Pin("LED", Pin.OUT)

    string1 = strings[0]
    white = string1.neo.convert_rgbw((255,255,255,0))
    
    while 1: # not rp2.bootsel_button():
        await asyncio.sleep_ms(100)
        led.toggle()
        if 1:
            for i in range(10):
                idx = random.randrange(len(string1.neo))
                string1[idx] = white #white snowflake
                
    run_flag = False
    await asyncio.gather(*stars)


def thanksgiving(**kwargs):
    
    thanksgiving_color_table = [
        [[16,6,1], 1],
        [[255,96,0], 1],
        [[255,32,0], 1],
     ]
    
    string1 = star_string(0, 17, 900)
    string1.smooth = True
    string1.replacements = 10
    string1.color_table = thanksgiving_color_table
    string1.oldweight = 15
    string1.star = None
    
    import _thread
    thr = _thread.start_new_thread(main_colors, ([string1], 100 ) )
    global run_flag, lights_state
    lights_state = "on" # no lighting control here, just  be  "on"
    #lights = asyncio.create_task(watch_light_level())
    star1 = asyncio.create_task(string1.run_stars())
    
    led = Pin("LED", Pin.OUT)
    
    white = string1.neo.convert_rgbw((255,255,255,0))
    
    while 1: # not rp2.bootsel_button():
        await asyncio.sleep_ms(100)
        led.toggle()
        for i in range(10):
            idx = random.randrange(len(string1.neo))
            string1[idx] = white #white snowflake
            
    run_flag = False
    time.sleep(1)
    await star1
    print(thread_state)
    
if __name__ == "__main__":
    def doit():
        global run_flag
        try:
            asyncio.run(xmas(timer=False))
        except KeyboardInterrupt:
            run_flag = False
            print("stopped")
    doit()


