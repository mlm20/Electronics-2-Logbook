import pyb
import time
from pyb import Pin, Timer
from oled_938 import OLED_938 

####
## Setting up OLED screen control
####

i2c = pyb.I2C(2, pyb.I2C.MASTER)
oled = OLED_938(
    pinout = {
        "sda": "Y10",
        "scl": "Y9",
        "res": "Y8"
    },
    height = 64,
    external_vcc = False,
    i2c_devid = i2c.scan()[0]
)
oled.poweron()
oled.init_display()

####
## Setting up control functions
####

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

def A_forward(value):
    A1.low()
    A2.high()
    motorA.pulse_width_percent(value)   

def B_forward(value):
    B1.low()
    B2.high()
    motorB.pulse_width_percent(value)  

def A_back(value):
    A1.high()
    A2.low()
    motorA.pulse_width_percent(value)

def B_back(value):
    B1.high()
    B2.low()
    motorB.pulse_width_percent(value)

def A_stop():
    A1.low()
    A2.low()

def B_stop():
    B1.low()
    B2.low()

####
## Potentiometer control
####

# initialise variables
SPEED = 0
pot = pyb.ADC(Pin("X11"))

while True:
    
    # Read potentiometer and scale value
    SPEED = int((pot.read()-2048)*200/4096)

    # Write to display
    oled.draw_text(0, 40, f"Motor Drive:{SPEED}%")
    oled.display()

    # Write to motor
    if SPEED >= 0:
        A_forward(SPEED)
        B_forward(SPEED)
    else:
        A_back(abs(SPEED))
        B_back(abs(SPEED))
