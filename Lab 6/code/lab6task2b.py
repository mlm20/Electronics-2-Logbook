from audio import MICROPHONE
import pyb
from array import array
from oled_938 import OLED_938	# Use OLED display driver

mic = pyb.ADC(pyb.Pin('Y11'))
sample_timer = pyb.Timer(7, freq = 8000)

audio = MICROPHONE(sample_timer, mic, 160)

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

#  Function to plot data on OLED display
def	plot_audio(signal):
    oled.clear()
    oled.draw_text(0,0,'Audio Signal')
    sig_range = max(signal)-min(signal)
    for i in range(0, 127):
        x = i
        y = 32 - int(signal[i]*20/sig_range)
        oled.set_pixel(x,y,True)
    oled.display()

local_buffer = array('h', 0 for i in range(160))    # reserve space for local samples

while True:
    if audio.buffer_is_filled():		# got a full buffer of samples
        for i in range(len(audio.data())):  # make a local copy
            local_buffer[i] = audio.data()[i]
        plot_audio(local_buffer)
        buffer_full = False
        audio.reset_buffer()			# start next set of 160 data points