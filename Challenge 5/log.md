# Challenge 5 - Self-balancing the segway

The following code was used for challenge 5

```python
####
## Setup
####

# Imports
import pyb
from pyb import Pin, Timer, ExtInt
import micropython
from mpu6050 import MPU6050

####
## IMU Setup
####

# Initialise IMU (connected to X9 and X10)
imu = MPU6050(1, False)

def read_imu(dt):

    # Setup global variables
    global g_pitch
    global g_pitch_dot

    # Change time constant (larger = longer)
    alpha = 0.7

    # Read pitch & pitch dot
    pitch = int(imu.pitch())
    pitch_dot = int(imu.get_gy())

    # Set filtered pitch to global variables
    g_pitch = alpha * (pitch + imu.get_gy() * dt * 0.001) + (1 - alpha) * pitch
    g_pitch_dot = pitch_dot

####
## Motor control setup
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

####
## PID Control
####

class PIDC:
    """
    Control PID Controller
    """
    def __init__(self, Kp, Kd, Ki):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.error_last = 0       # These are global variables to remember various states of controller
        self.tic = pyb.millis()
        self.error_sum = 0

    def getPWM(self, target, pitch, pitch_dot):

        # Error Input
        error = target - pitch          # e[n]
    
        # Derivative Input
        derivative = - pitch_dot         # negative feedback

        toc = pyb.millis()
        dt = (toc - self.tic) * 0.001       # find dt as close to when used as possible
        # Integration Input 
        self.error_sum += error * dt            
 
        # Output 
        PID_output = (self.Kp * error) + (self.Ki * self.error_sum) + (self.Ki * derivative)

        # Store Previous Values 
        self.error_last = error
        self.tic = toc

        pwm_out = min(abs(PID_output), 100)             # Make sure pwm is less than 100 
 
        if PID_output > 0:                              # Output direction (need to check)
            direction = "forward"
        elif PID_output < 0:
            direction = "back"
        else: 
            direction = "stop"

        return (pwm_out, direction)

####
## Main program loop
####

# Start timer
tic = pyb.millis()

# Create PIDC Object
pid = PIDC(4.0, 0.60, 60)

# Infinite loop
while True:

    # Read IMU
    toc = pyb.millis()
    read_imu(toc - tic)

    ## PIDC Reading 
    pid_output = pid.getPWM(0.0, g_pitch, g_pitch_dot)
    
    # Write to motor
    if pid_output[1] == "forward":
        A_forward(pid_output[0])
        B_forward(pid_output[0])
    elif pid_output[1] == "back":
        A_back(abs(pid_output[0]))
        B_back(abs(pid_output[0]))
    else:
        A_stop()
        B_stop()

    # Delay
    pyb.delay(1)

    # Restart timer
    tic = pyb.millis()
```

I faced many difficulties when tryign to do this challenge.

Initially the Segway was extremely laggy, in that there would be a noticeable delay between it pithing in one direction and the motors turning. This resulted in extreme oscillations regardless of the values used for `Kp`, `Kd` and `Ki`. 

Upon further investigation I realised that this was due to me: 

1. Measuring the motor speeds (via interrupts) from the Hall Effect sensors.
2. Displaying the pitch angle and motor speeds on the OLED.

These are both very slow processes and when these were removed from the code the Segway significantly more reponsive.

Upon tuning the  `Kp`, `Kd` and `Ki` values I was able to achieve partial stabilisation. The Segway is able to stabilise for a few seconds but I was unable to get it to remain stead after that time period.

A video of it in action is available in the `media `folder.