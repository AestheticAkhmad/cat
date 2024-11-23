import time
# Import the PCA9685 module.
from board import SCL, SDA
import busio
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
# Create the I2C bus interface.
import RPi.GPIO as GPIO
class Robot:# Initialise the PCA9685 using the default address (0x40).
	i2c_bus = busio.I2C(SCL, SDA)
	pwm =  PCA9685(i2c_bus)

	move_speed = 0x5555  # half of Max pulse length out of 0xFFFF

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

	def changespeed(self, speed):
		pwm.channels[ENA].duty_cycle = speed
		pwm.channels[ENB].duty_cycle = speed


	def stopcar(self):
		GPIO.output(LEFT_BACK, GPIO.LOW)
		GPIO.output(LEFT_FRONT, GPIO.LOW)
		GPIO.output(RIGHT_BACK, GPIO.LOW)
		GPIO.output(RIGHT_FRONT, GPIO.LOW)
		changespeed(0)


	def backward(self):
		GPIO.output(IN1, GPIO.HIGH)
		GPIO.output(IN2, GPIO.LOW)
		GPIO.output(IN3, GPIO.HIGH)
		GPIO.output(IN4, GPIO.LOW)
		changespeed(move_speed)
		
	def forward(self):
		GPIO.output(LEFT_BACK, GPIO.LOW)
		GPIO.output(LEFT_FRONT, GPIO.HIGH)
		GPIO.output(RIGHT_BACK, GPIO.LOW)
		GPIO.output(RIGHT_FRONT, GPIO.HIGH)
		changespeed(move_speed)

	def turnRight(self):
		GPIO.output(IN1, GPIO.LOW)
		GPIO.output(IN2, GPIO.HIGH)
		GPIO.output(IN3, GPIO.HIGH)
		GPIO.output(IN4, GPIO.LOW)

		
	def turnLeft(self):
		GPIO.output(IN1, GPIO.HIGH)
		GPIO.output(IN2, GPIO.LOW)
		GPIO.output(IN3, GPIO.LOW)
		GPIO.output(IN4, GPIO.HIGH)
		changespeed(move_speed)

