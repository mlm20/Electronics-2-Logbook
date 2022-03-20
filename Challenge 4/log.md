# Challenge 4 - Segway dancing to music with stabilisers

The following code was used for the challenge

```python
# Imports
import pyb
from pyb import Pin, Timer, ADC, DAC, LED, ExtInt
from array import array			# need this for memory allocation to buffers
from oled_938 import OLED_938	# Use OLED display driver
from audio import MICROPHONE
from neopixel import NeoPixel

# Importing dance routine from text file
with open('dancemoves.txt','r') as file:
    dance_routine_string = file.read().strip('\n')
dance_routine = list(dance_routine_string)

# Dancemove class
class DANCE:
    def __init__(self, PWM, regular_delay, spin_delay):
        self.PWM = PWM
        self.regular_delay = regular_delay
        self.spin_delay = spin_delay

    def forward(self):
        A_forward(self.PWM)
        B_forward(self.PWM)
    
        pyb.delay(self.regular_delay)
    
        A_stop()
        B_stop()

    def backward(self):
        A_back(self.PWM)
        B_back(self.PWM)
    
        pyb.delay(self.regular_delay)
    
        A_stop()
        B_stop()

    def spin(self):
        A_forward(self.PWM)
        B_back(self.PWM)
    
        pyb.delay(self.spin_delay)
    
        A_stop()
        B_stop()

####
## Motor control setup
####

# define potentiometer pin
pot = pyb.ADC(Pin("X11"))

# define pins to control motor
A1 = Pin("X3", Pin.OUT_PP)
A2 = Pin("X4", Pin.OUT_PP)
PWMA = Pin("X1")

B1 = Pin("X7", Pin.OUT_PP)
B2 = Pin("X8", Pin.OUT_PP)
PWMB = Pin("X2")

# configure timer 2 to produce 1kHz clock for PWM control
tim = Timer(2, freq = 1000)
motorA = tim.channel(1, Timer.PWM, pin = PWMA)
motorB = tim.channel(2, Timer.PWM, pin = PWMB)

# motor control functions
def A_forward(value):
    A1.low()
    A2.high()
    motorA.pulse_width_percent(value)   

def B_forward(value):
    B1.high()
    B2.low()
    motorB.pulse_width_percent(value)  

def A_back(value):
    A1.high()
    A2.low()
    motorA.pulse_width_percent(value)

def B_back(value):
    B1.low()
    B2.high()
    motorB.pulse_width_percent(value)

def A_stop():
    A1.low()
    A2.low()

def B_stop():
    B1.low()
    B2.low()

def stop():
    A_stop()
    B_stop()

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

## Initialise dance Object
dance = DANCE(80, 200, 400)

# Initilaise move counter
move_number = 0

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

        if (pyb.millis() - tic > MIN_BEAT_PERIOD):	# if longer than minimum period
            if (c > BEAT_THRESHOLD):		# look for a beat
                # Reset move counter if greater than length of dance_routine
                #flash LED
                b_LED.on()
                pyb.delay(10)
                b_LED.off()

                if move_number > (len(dance_routine) - 1):
                    move_number = 0

                # Perform dance move at counter index
                if dance_routine[move_number] == "F":
                    dance.forward()
                    print("F")

                if dance_routine[move_number] == "B":
                    dance.backward()
                    print("B")

                if dance_routine[move_number] == "S":
                    dance.spin()
                    print("S")
                
                # increase counter by 1
                move_number = move_number + 1

                tic = pyb.millis()		# reset tic
        audio.reset_buffer()				# reset status flag
```

The following dance move order was implemented:

1. Forwards
2. Back
3. Forwards
4. Back
5. Spin

A class was created for the dance moves. Methods in the class were created for each dance move. The duration of each dance move as well as the PWM intensity of each can be specified in the initialisation parameters of the dance move Object.

The code worked as expected! A video of it working can be found in the `media` folder.