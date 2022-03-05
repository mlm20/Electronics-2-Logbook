# Lab 5 -  Motor & Interrupt

## Task 1

Motor A was controlled successfully using a PWM signal using the following code

```python
import pyb
from pyb import Pin, Timer

# define pins to control motor
A1 = Pin("X3", Pin.OUT_PP)
A2 = Pin("X4", Pin.OUT_PP)
PWMA = Pin("X1")

# Contigure timer 2 produce 1KHz clock for PWN control
tim = Timer(2, freq = 1000)
motorA = tim.channel(1, Timer.PWM, pin = PWMA)

def A_forward(value):
    A1.low()
    A2.high()
    motorA.pulse_width_percent(value)
```

Subsequently, `A_back(value)` and `A_stop()` were added to the code.

A clock loop was added to test the functionality of these.

Initially this code didn’t work for us. This was for two separate issues

1. The directory in `user.py` wasn’t correct
2. Our motor was faulty. We tested this code using another group’s motor. This worked. Subsequent tasks were completed on a  replacement motor.

### Part C

Functionality for motor B as well as potentiometer control and OLED screen display were added. The following code was written

```python
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

```

This worked as expected.

## Task 2

The template code in the Lab guide was adapted using my code rom the previous task to measure the speed (rpm) of the motor.

```python
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
```

This worked as expected though it was fairly noisy as suggested in the guide. The value kept changing around a certain point.

When `pyb.delay(1)` was added to the `while True:` loop I was unable to observe any difference. The speed reading remained at the same value and still moved around said value.

The code was adapted to turn motor B in addition to motor A. This was done by replicating all the statements referencing motor A for motor B.

## Task 3

The interruption code snippet was implemented into the code I wrote previously, it functioned as expected.

```python
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
```

Using interrupts instead of polling is good as it is

1. More reliable. Since the processor is forced to stop and update every time there is a change in the rps all changes are recorded.
2. More efficient. You are saving processing power since it no longer needs to poll the Hall Sensors are regular intervals.
