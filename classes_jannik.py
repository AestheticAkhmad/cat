import time
import cv2
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import RPi.GPIO as GPIO

class Robot:
    def __init__(self):
        # Initialize motor speed and GPIO pins for motor direction
        self.move_speed = 0x7FFF
        self.LEFT_BACK = 23
        self.LEFT_FRONT = 24
        self.RIGHT_BACK = 27
        self.RIGHT_FRONT = 22

        # Initialize I2C communication and PCA9685 for motor control
        self.i2c_bus = busio.I2C(SCL, SDA)
        self.pwm = PCA9685(self.i2c_bus)
        self.pwm.frequency = 60

        # Set up GPIO pins for motor direction
        GPIO.setup(self.LEFT_BACK, GPIO.OUT)
        GPIO.setup(self.LEFT_FRONT, GPIO.OUT)
        GPIO.setup(self.RIGHT_BACK, GPIO.OUT)
        GPIO.setup(self.RIGHT_FRONT, GPIO.OUT)

        # Define PWM channels for motor speed control
        self.ENA = 0
        self.ENB = 1

    # Configure GPIO settings
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    def changespeed(self, leftSpeed, rightSpeed):
        # Set PWM duty cycle for left and right motors
        self.pwm.channels[self.ENA].duty_cycle = leftSpeed
        self.pwm.channels[self.ENB].duty_cycle = rightSpeed

    def stopcar(self):
        # Stop the robot by setting motor direction pins to LOW and speed to 0
        GPIO.output(self.LEFT_BACK, GPIO.LOW)
        GPIO.output(self.LEFT_FRONT, GPIO.LOW)
        GPIO.output(self.RIGHT_BACK, GPIO.LOW)
        GPIO.output(self.RIGHT_FRONT, GPIO.LOW)
        self.changespeed(0, 0)

    def backward(self):
        # Move the robot backward
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)

    def forward(self):
        # Move the robot forward
        GPIO.output(self.LEFT_BACK, GPIO.LOW)
        GPIO.output(self.LEFT_FRONT, GPIO.HIGH)
        GPIO.output(self.RIGHT_BACK, GPIO.LOW)
        GPIO.output(self.RIGHT_FRONT, GPIO.HIGH)

    def turnRight(self):
        # Turn the robot to the right
        GPIO.output(self.LEFT_BACK, GPIO.LOW)
        GPIO.output(self.LEFT_FRONT, GPIO.HIGH)
        GPIO.output(self.RIGHT_BACK, GPIO.HIGH)
        GPIO.output(self.RIGHT_FRONT, GPIO.LOW)

    def turnLeft(self):
        # Turn the robot to the left
        GPIO.output(self.LEFT_BACK, GPIO.HIGH)
        GPIO.output(self.LEFT_FRONT, GPIO.LOW)
        GPIO.output(self.RIGHT_BACK, GPIO.LOW)
        GPIO.output(self.RIGHT_FRONT, GPIO.HIGH)

    def preprocess_image(self, frame):
        # Focus on the lower third of the frame to detect the line
        height, width, _ = frame.shape
        roi = frame[int(height * 2 / 3):, :]

        # Convert to grayscale and apply Gaussian blur
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        gray_blurred = cv2.GaussianBlur(gray, (7, 7), 0)

        # Threshold the image to detect the black line
        _, binary = cv2.threshold(gray_blurred, 60, 255, cv2.THRESH_BINARY_INV)
        contours = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]

        # Find the largest contour and calculate its centroid
        largest = max(contours, key=cv2.contourArea) if contours else None
        if largest is not None:
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                return cx
        return None

    def detect_duck(self, frame):
        # Convert frame to HSV to detect yellow objects
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        lower_yellow = (20, 100, 100)
        upper_yellow = (30, 255, 255)

        # Create a mask for yellow objects and find contours
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Check if a large yellow object (duck) is detected
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                print("Careful, there is a Duck!")
                return True
        return False

    def process_qr_code(self, frame):
        # Detect and decode QR codes in the frame
        qr_detector = cv2.QRCodeDetector()
        data, points, _ = qr_detector.detectAndDecode(frame)

        # Perform actions based on QR code instructions
        # 2.5 sec for 360°
        if data == "car_stop_10s":
            print("Instruction:", data)
            Robot.stopcar()
            time.sleep(10)
        if data == "car_turn_around":
            print("Instruction:", data)
            Robot.stopcar()
            qr_turn_speed = 0x6FFF
            Robot.changespeed(qr_turn_speed, qr_turn_speed)
            time.sleep(1.25)
        if data == "car_rotate_720":
            print("Instruction:", data)
            Robot.stopcar()
            qr_turn_speed = 0x6FFF
            Robot.changespeed(qr_turn_speed, qr_turn_speed)
            time.sleep(5)
            return True
        return False
