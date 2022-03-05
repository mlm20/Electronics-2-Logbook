import pyb
import time
from pyb import Pin, Timer, ExtInt
from oled_938 import OLED_938
import micropython

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

# Initialise variables
speed = 0
A_state = 0             # previous state of A sensor
A_speed = 0             # latest speed of motor A  
A_count = 0             # postiive transition count
tic = pyb.millis()      # keep time in millisecond

####
## Interrupt section
####

def isr_motorA(dummy):
    global A_count
    A_count += 1

def isr_speed_timer(dummy):
    global A_count
    global A_speed
    A_speed = A_count
    A_count = 0

# Create external interruptuons for motorA Hall Effect Sensor
micropython.alloc_emergency_exception_buf(100)
motorA_int = ExtInt("Y4", ExtInt.IRQ_RISING, Pin.PULL_NONE, isr_motorA)

# Create timer interrupts at 100 ms intervals
speed_timer = pyb.Timer(4, freq = 10)
speed_timer.callback(isr_speed_timer)

####
## Detection Loop
####

while True:

    # Read potentiometer and scale value
    speed = int((pot.read() - 2048) * 200 / 4096)

    # Write to motor
    if speed >= 0:
        A_forward(speed)
        B_forward(speed)

    else:
        A_back(abs(speed))
        B_back(abs(speed))

    # Display new speed
    oled.draw_text(0, 20, f"Motor Drive: {speed} rps")
    oled.draw_text(0, 35, f"Motor A: {A_speed/39} rps")
    oled.display()

    # Delay
    pyb.delay(100)
