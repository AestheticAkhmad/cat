import time
# Import the PCA9685 module.
from board import SCL, SDA
import busio
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

import RPi.GPIO as GPIO
# Initialise the PCA9685 using the default address (0x40).
pwm =  PCA9685(i2c_bus)

move_speed = 0x7FFF  # half of Max pulse length out of 0xFFFF

# Set frequency to 60hz, good for servos.
pwm.frequency = 60
GPIO.setmode(GPIO.BCM) # GPIO number  in BCM mode
GPIO.setwarnings(False)
#define L298N(Model-Pi motor drive board) GPIO pins
LEFT_BACK = 23  #Left motor direction pin
LEFT_FRONT = 24  #Left motor direction pin
RIGHT_BACK = 27  #Right motor direction pin
RIGHT_FRONT = 22  #Right motor direction pin
ENA = 0  #Left motor speed PCA9685 port 0
ENB = 1  #Right motor speed PCA9685 port 1

# Define motor control  pins as output
GPIO.setup(LEFT_BACK, GPIO.OUT)   
GPIO.setup(LEFT_FRONT, GPIO.OUT) 
GPIO.setup(RIGHT_BACK, GPIO.OUT)   
GPIO.setup(RIGHT_FRONT, GPIO.OUT) 

def changespeed(speed):
	pwm.channels[ENA].duty_cycle = speed
	pwm.channels[ENB].duty_cycle = speed


def stopcar():
	GPIO.output(LEFT_BACK, GPIO.LOW)
	GPIO.output(LEFT_FRONT, GPIO.LOW)
	GPIO.output(RIGHT_BACK, GPIO.LOW)
	GPIO.output(RIGHT_FRONT, GPIO.LOW)
	changespeed(0)


def backward():
	GPIO.output(IN1, GPIO.HIGH)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.HIGH)
	GPIO.output(IN4, GPIO.LOW)
	changespeed(move_speed)
 
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(0.25)  
	#stopcar()
	
def forward():
	GPIO.output(LEFT_BACK, GPIO.LOW)
	GPIO.output(LEFT_FRONT, GPIO.HIGH)
	GPIO.output(RIGHT_BACK, GPIO.LOW)
	GPIO.output(RIGHT_FRONT, GPIO.HIGH)
	changespeed(move_speed)
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(0.25)  
	#stopcar()
	
def turnRight():
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN2, GPIO.HIGH)
	GPIO.output(IN3, GPIO.HIGH)
	GPIO.output(IN4, GPIO.LOW)
	changespeed(move_speed)
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(0.25)  
	#stopcar()
	
def turnLeft():
	GPIO.output(IN1, GPIO.HIGH)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.LOW)
	GPIO.output(IN4, GPIO.HIGH)
	changespeed(move_speed)	
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(0.25)  
	#stopcar()
	
forward()
time.sleep(6)  
stopcar()
time.sleep(0.25) 


print('press Ctrl-C to quit...')
#while True:
# Move servo on channel O between extremes.

time.sleep(2)
