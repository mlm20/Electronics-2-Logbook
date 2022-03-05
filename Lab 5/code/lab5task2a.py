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
## Motor Control
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
## Motor speed testing
####

# Define pins for motor speed sensors
A_sense = Pin("Y4", Pin.PULL_NONE)
B_sense = Pin("Y6", Pin.PULL_NONE)

# Initialise variables
A_state = 0             # previous state of A sensor
A_speed = 0             # latest speed of motor A  
A_count = 0             # postiive transition count
tic = pyb.millis()      # keep time in millisecond

# Dectection loop
while True:

    # Detect rising edge on sensor A
    if (A_state == 0) and (A_sense.value()==1):
        A_count += 1
    A_state = A_sense.value()

    # Check to see if 100 ms has elapsed
    toc = pyb.millis()
    if (toc - tic) >= 100:
        A_speed = A_count

        ## drive motor - controlled by potentiometer
        pot = pyb.ADC(Pin("X11"))

        # Read potentiometer and scale value
        A_speed = int((pot.read()-2048)*200/4096)

        # Write to motor
        if A_speed >= 0:
            A_forward(A_speed)
            B_forward(A_speed)
        else:
            A_back(abs(A_speed))
            B_back(abs(A_speed))

        # Display new speed
        oled.draw_text(0, 20, f"Motor A: {A_speed} rps")
        oled.display()
        tic = pyb.millis()
