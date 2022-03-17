# Challenge 1 - Dancing LED lights

The following code was used for challenge 1

```python
import pyb
from pyb import Pin, Timer, ADC, DAC, LED
from array import array			# need this for memory allocation to buffers
from oled_938 import OLED_938	# Use OLED display driver
from audio import MICROPHONE
from neopixel import NeoPixel

# initialise neopixel Object
np = NeoPixel(Pin("Y12", Pin.OUT), 8)

# set all LEDs to dark
for i in range(8):
    np[i] = (0, 0, 0)
    np.write()
    pyb.delay(1) 

#  The following two lines are needed by micropython
#   ... must include if you use interrupt in your program
import micropython
micropython.alloc_emergency_exception_buf(100)

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()				# find the I2C device number
oled = OLED_938(
    pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
    height=64,
    external_vcc=False,
    i2c_devid=i2c.scan()[0],
)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Beat Detection')
oled.display()

# define ports for microphone, LEDs and trigger out (X5)

b_LED = LED(4)		# flash for beats on blue LED

def flash():		# routine to flash blue LED when beat detected
    ####
    ## control board LED
    ####

    b_LED.on()
    pyb.delay(10)
    b_LED.off()

    ####
    ## control NeoPixel
    ####

    # Band 1
    if BEAT_THRESHOLD < c <= BEAT_THRESHOLD + 0.1:
        # set colour
        np[0] = (56, 15, 91)
        np.write()
        pyb.delay(10)

        # reset to blank
        for i in range(8):
            np[i] = (0, 0, 0)
            np.write()

    # Band 2
    if BEAT_THRESHOLD + 0.1 < c <= BEAT_THRESHOLD + 0.2:
        # set colour
        np[0] = (56, 15, 91) 
        np[1] = (48, 49, 111)
        np.write()
        pyb.delay(10)

        # reset to blank
        for i in range(8):
            np[i] = (0, 0, 0)
            np.write()

    # Band 3
    if BEAT_THRESHOLD + 0.2 < c <= BEAT_THRESHOLD + 0.3:
        # set colour
        np[0] = (56, 15, 91) 
        np[1] = (48, 49, 111)
        np[3] = (40, 84, 132)
        np.write()
        pyb.delay(10)

        # reset to blank
        for i in range(8):
            np[i] = (0, 0, 0)
            np.write()

    # Band 4
    if BEAT_THRESHOLD + 0.3 < c <= BEAT_THRESHOLD + 0.4:
        # set colour
        np[0] = (56, 15, 91) 
        np[1] = (48, 49, 111)
        np[3] = (40, 84, 132)
        np[4] = (32, 118, 152)
        np.write()
        pyb.delay(10)

        # reset to blank
        for i in range(8):
            np[i] = (0, 0, 0)
            np.write()

    # Band 5
    if BEAT_THRESHOLD + 0.4 < c <= BEAT_THRESHOLD + 0.5:
        # set colour
        np[0] = (56, 15, 91) 
        np[1] = (48, 49, 111)
        np[3] = (40, 84, 132)
        np[4] = (32, 118, 152)
        np[5] = (24, 152, 173)
        np.write()
        pyb.delay(10)

        # reset to blank
        for i in range(8):
            np[i] = (0, 0, 0)
            np.write()

    # Band 6
    if BEAT_THRESHOLD + 0.5 < c <= BEAT_THRESHOLD + 0.6:
        # set colour
        np[0] = (56, 15, 91) 
        np[1] = (48, 49, 111)
        np[3] = (40, 84, 132)
        np[4] = (32, 118, 152)
        np[5] = (24, 152, 173)
        np[6] = (16, 186, 193)
        np.write()
        pyb.delay(10)

        # reset to blank
        for i in range(8):
            np[i] = (0, 0, 0)
            np.write()

    # Band 7
    if BEAT_THRESHOLD + 0.6 < c <= BEAT_THRESHOLD + 0.7:
        # set colour
        np[0] = (56, 15, 91) 
        np[1] = (48, 49, 111)
        np[3] = (40, 84, 132)
        np[4] = (32, 118, 152)
        np[5] = (24, 152, 173)
        np[6] = (16, 186, 193)
        np[7] = (8, 221, 214)
        np.write()
        pyb.delay(10)

        # reset to blank
        for i in range(8):
            np[i] = (0, 0, 0)
            np.write()

    # Band 8
    if BEAT_THRESHOLD + 0.7 < c :
        # set colour
        np[0] = (56, 15, 91) 
        np[1] = (48, 49, 111)
        np[2] = (40, 84, 132)
        np[3] = (32, 118, 152)
        np[4] = (24, 152, 173)
        np[5] = (16, 186, 193)
        np[6] = (8, 221, 214)
        np[7] = (0, 255, 234)
        np.write()
        pyb.delay(10)

        # reset to blank
        for i in range(8):
            np[i] = (0, 0, 0)
            np.write()

# Create timer interrupt - one every 1/8000 sec or 125 usec
pyb.disable_irq()
sample_timer = pyb.Timer(7, freq=8000)	# set timer 7 for 8kHz

N = 160				# number of sample to calculate instant energy
mic = ADC(Pin('Y11'))
audio = MICROPHONE(sample_timer, mic, N)
pyb.enable_irq(True)
oled.draw_text(0,20, 'Ready to GO')	# Useful to show what's happening?
oled.display()
pyb.delay(100)

# Calculate energy over 50 epochs, each 20ms (i.e. 1 sec)
M = 50						# number of instantaneous energy epochs to sum
BEAT_THRESHOLD = 2.4		# threshold for c to indicate a beat
MIN_BEAT_PERIOD = 200	# no beat less than this

# initialise variables for main program loop
e_ptr = 0					# pointer to energy buffer
e_buf = array('L', 0 for i in range(M))	# reserve storage for energy buffer
sum_energy = 0				# total energy in last 50 epochs

tic = pyb.millis()			# mark time now in msec

while True:				# Main program loop
    if audio.buffer_is_filled():		# semaphore signal from ISR - set if buffer is full
		
        # Fetch instantaneous energy
        E = audio.inst_energy()			# fetch instantenous energy
        audio.reset_buffer()			# get ready for next epoch

		# compute moving sum of last 50 energy epochs with circular buffer
        sum_energy = sum_energy - e_buf[e_ptr] + E
        e_buf[e_ptr] = E			# over-write earliest energy with most recent
        e_ptr = (e_ptr + 1) % M		# increment e_ptr with wraparound - 0 to M-1
        average_energy = sum_energy/M

		# Compute ratio of instantaneous energy/average energy
        c = E/average_energy
		
        if (pyb.millis()-tic > MIN_BEAT_PERIOD):	# if longer than minimum period
            if (c>BEAT_THRESHOLD):		# look for a beat
                flash()					# beat found, flash blue LED
                tic = pyb.millis()		# reset tic
        audio.reset_buffer()				# reset status flag

        print(E)
```

In summary this code functioned very well, the Neopixel strip reacted to the beat of the music. 

We implemented a system where the number of neopixel LEDs that would illuminate is in respect to the Energy `E` (volume) of the music/beat. In order to make this more visual we also made it so it would light up in a gradient from dark blue to light blue, although this ended up being very subtle on the real strip.

Our greatest challenge was with the sensitivity, alongside having the neopixel react we kept the reactive board LED from Lab 6 in order to compare the two. Initially the board LED would light up despite the neopixel not lighting up, meaning that `BEAT_THRESHOLD` had been met but the energy required to light u the neopixel LEDs was too great.

Subsequently we lowered the energy threashold required to light up each LED. Initially there were gaps of 0.2 (beat threshold) required to light up each subsequent LED, this was lowered to 0.1. Upon dong this it functioned as expected.

A demo video is accessible at `media/neopixel_demo.mp4`