import time
# Import the PCA9685 module.
from board import SCL, SDA
import busio
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
# Create the I2C bus interface.
import RPi.GPIO as GPIO
class Robot:# Initialise the PCA9685 using the default address (0x40).
	def __init__(self):
		self.move_speed = 0x5555  # half of Max pulse length out of 0xFFFF
		self.LEFT_BACK = 23  #Left motor direction pin
		self.LEFT_FRONT = 24  #Left motor direction pin
		self.RIGHT_BACK = 27  #Right motor direction pin
		self.RIGHT_FRONT = 22  #Right motor direction pin
		self.i2c_bus = busio.I2C(SCL, SDA)
		self.pwm =  PCA9685(self.i2c_bus)
		self.pwm.frequency = 60

		GPIO.setup(self.LEFT_BACK, GPIO.OUT)   
		GPIO.setup(self.LEFT_FRONT, GPIO.OUT) 
		GPIO.setup(self.RIGHT_BACK, GPIO.OUT)   
		GPIO.setup(self.RIGHT_FRONT, GPIO.OUT) 

		self.ENA = 0  #Left motor speed PCA9685 port 0
		self.ENB = 1  #Right motor speed PCA9685 port 1

	# Set frequency to 60hz, good for servos.
	
	GPIO.setmode(GPIO.BCM) # GPIO number  in BCM mode
	GPIO.setwarnings(False)
	#define L298N(Model-Pi motor drive board) GPIO pins

	def changespeed(self, speed):
		self.pwm.channels[self.ENA].duty_cycle = speed
		self.pwm.channels[self.ENB].duty_cycle = speed

	def stopcar(self):
		GPIO.output(self.LEFT_BACK, GPIO.LOW)
		GPIO.output(self.LEFT_FRONT, GPIO.LOW)
		GPIO.output(self.RIGHT_BACK, GPIO.LOW)
		GPIO.output(self.RIGHT_FRONT, GPIO.LOW)
		self.changespeed(0)

	def backward(self):
		GPIO.output(IN1, GPIO.HIGH)
		GPIO.output(IN2, GPIO.LOW)
		GPIO.output(IN3, GPIO.HIGH)
		GPIO.output(IN4, GPIO.LOW)
		changespeed(move_speed)
		
	def forward(self):
		GPIO.output(self.LEFT_BACK, GPIO.LOW)
		GPIO.output(self.LEFT_FRONT, GPIO.HIGH)
		GPIO.output(self.RIGHT_BACK, GPIO.LOW)
		GPIO.output(self.RIGHT_FRONT, GPIO.HIGH)
		self.changespeed(self.move_speed)
		time.sleep(0.35)

	def turnRight(self):
		GPIO.output(self.LEFT_BACK, GPIO.LOW)
		GPIO.output(self.LEFT_FRONT, GPIO.HIGH)
		GPIO.output(self.RIGHT_BACK, GPIO.HIGH)
		GPIO.output(self.RIGHT_FRONT, GPIO.LOW)
		self.changespeed(self.move_speed)
		
	def turnLeft(self):
		GPIO.output(self.LEFT_BACK, GPIO.HIGH)
		GPIO.output(self.LEFT_FRONT, GPIO.LOW)
		GPIO.output(self.RIGHT_BACK, GPIO.LOW)
		GPIO.output(self.RIGHT_FRONT, GPIO.HIGH)
		self.changespeed(self.move_speed)

