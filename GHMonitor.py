#!/usr/bin/python
# Importing modules
import blynklib
import time
import spidev # To communicate with SPI devices
from numpy import interp	# To scale values
from time import sleep	# To add delay
import RPi.GPIO as GPIO	# To use GPIO pins
# Start SPI connection
spi = spidev.SpiDev() # Created an object
spi.open(0,0)	
# Initializing LED pin as OUTPUT pin

BLYNK_AUTH = 'TJkD0-GGNUnVzcHW7OxqzkuW1hC9P8mp' #insert your Auth Token here
# base lib init
blynk = blynklib.Blynk(BLYNK_AUTH)

output_LDR = 0.0
output_POT = 0.0
output_TEMP = 0.0

localtime=0
systime=0
date =0
t0=time.time()
sampletime=1
int check = -1
led_pin = 20
i = 0
GPIO.setmode(GPIO.BCM)
GPIO.setup(led_pin, GPIO.OUT)
# Creating a PWM channel at 100Hz frequency
pwm = GPIO.PWM(led_pin, 100)
pwm.start(0) 
# Read MCP3008 data
def analogInput(channel):
    spi.max_speed_hz = 1350000
    adc = spi.xfer2([1,(8+channel)<<4,0])
    data = ((adc[1]&3) << 8) + adc[2]
    return data

def Volts(data):
    volts = (data * 3.3) / float(1023)
    volts = round(volts, 2) # Round off to 2 decimal places
    return volts
 
# Below function will convert data to temperature.
def Temp(data):
    temp = (data-1.1)/0.01
    temp = round(temp,1)
    return temp

@blynk.handle_event('read V0')
def read_virtual_pin_handler(pin):

  blynk.virtual_write(0, str(round(output_LDR,2)))
  blynk.virtual_write(1, str(round(output_POT,2)))
  blynk.virtual_write(2, str(round(output_TEMP,1)))
  blynk.virtual_write(3, "Hello World\n")

while True:
    blynk.run()


    localtime = time.strftime("%H:%M:%S", time.gmtime(time.time()))
    systime = time.strftime("%H:%M:%S", time.gmtime(time.time()-t0))
    second = int(localtime[-2:])

    output_LDR = analogInput(0) # Reading from CH0
    output_LDR = interp(output_LDR, [0, 1023], [0, 33])/10
    
    output_POT = analogInput(1) # Reading from CH1
    output_POT = interp(output_POT, [0, 1023], [0, 33])/10
    
    output_TS = analogInput(2) # Reading from CH2
    output_TS = interp(output_TS, [0, 1023], [0, 33])/10
    output_TEMP = Temp(output_TS)
    print(systime[-2:])
    print(localtime)
    print(systime)
    print("Light: " + str(round(output_LDR,2)))
    print("Humidity: " + str(round(output_POT,2)))
    print("Temp Voltage: " + str(output_TS))
    print("Temperature: " + str(round(output_TEMP,1))+"\n")

	#pwm.ChangeDutyCycle(output)

    sleep(1)