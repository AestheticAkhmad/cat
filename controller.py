import cv2
from picamera2 import Picamera2
import time
import math
import RPi.GPIO as GPIO
from classes import Robot, PIDController
import numpy as np

# Initialize the robot
robot = Robot()

# Initialize the camera
picam2 = Picamera2()
picam2.start()

# Video settings
frame_rate = 30
video_duration = 25
frame_width, frame_height = 640, 480

# Robot parameters
distance_to_line = 12  # Ground distance in cm, calibrated for 45-degree camera angle
turn_speed = 0x6FFF     # Adjust speed for turning
straight_speed = 0x7FFF

# PID parameters
Kp_dir, Ki_dir, Kd_dir = 0.6, 0.3, 0.1
Kp_spd, Ki_spd, Kd_spd = 0.5, 0.2, 0.1

# Helper function: preprocess the image
def preprocess_image(frame):
    height, width, _ = frame.shape
    roi = frame[int(height * 2 / 3):, :]  # Focus on the lower third of the frame
    
    gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
    gray_blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    
    # Thresholding to detect the black line
    _, binary = cv2.threshold(gray_blurred, 60, 255, cv2.THRESH_BINARY_INV)
    _, contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(contours)
    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        return cx

# Instantiate PID controllers
PID_direction = PIDController(Kp_dir, Ki_dir, Kd_dir)
PID_speed = PIDController(Kp_spd, Ki_spd, Kd_spd)

# Initialize variables
frame_center = frame_width // 2
base_speed = straight_speed
prev_error = 0
angle_sum = 0
counter = 0

# Main loop
start_time = time.time()
try:
    while time.time() - start_time < video_duration:
        # Capture frame from the camera
        frame = picam2.capture_array()
        
        # Preprocess the frame to find the line's centroid
        cx = preprocess_image(frame)
        
        if cx is not None:
            # Calculate the error
            error = frame_center - cx
            
            # Update PID controllers
            direction_speed = PID_direction.update(error)
            speed_correction = PID_speed.update(error - prev_error)
            
            # Adjust motor speeds
            left_speed = base_speed + speed_correction
            right_speed = base_speed - speed_correction
            
            # Send commands to the robot
            robot.changespeed(left_speed, right_speed)
            
            # Update angle sum and counter for analysis
            angle_sum += math.atan2(error, distance_to_line)
            counter += 1
            
            # Store the previous error
            prev_error = error
        else:
            print("No line detected, stopping.")
            robot.stopcar()
        
        # Break loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Cleanup resources
    picam2.stop()
    cv2.destroyAllWindows()
    robot.stopcar()
    print("Driving done!")
    if counter > 0:
        print(f"Average steering angle: {angle_sum / counter:.2f} radians")