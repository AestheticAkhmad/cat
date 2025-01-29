from simple_pid import PID
import cv2
from picamera2 import Picamera2
import time
import RPi.GPIO as GPIO
from classes import Robot
import numpy as np

# Initialize the robot
robot = Robot()

# Initialize the camera
picam2 = Picamera2()
picam2.start()

# Video settings
frame_rate = 30
video_duration = 120
frame_width, frame_height = 640, 480

# Robot parameters
turn_speed = 0x6FFF
straight_speed = 0x6FFF
max_speed = 0x7FFF

# pid_direction = PID(Kp=13, Ki=66, Kd=33, setpoint=frame_width // 2)
pid_direction = PID(Kp=100, Ki=340, Kd=9, setpoint=frame_width // 2)

pid_direction.output_limits = (-max_speed // 2, max_speed // 2) 

# Helper function: preprocess the image
def preprocess_image(frame):
    height, width, _ = frame.shape
    roi = frame[int(height * 2 / 3):, :]  # Focus on the lower third of the frame

    gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
    gray_blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    # Thresholding to detect the black line
    _, binary = cv2.threshold(gray_blurred, 60, 255, cv2.THRESH_BINARY_INV)
    contours = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    largest = max(contours, key=cv2.contourArea) if contours else None
    if largest is not None:
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            return cx
    return None

def detect_duck(frame):
    img = frame.convert("RGB")
    img_array = np.array(img)

    hsv_img = cv2.cvtColor(img_array, cv2.COLOR_RGB2HSV)

    lower_orange = np.array([10, 80, 80])
    upper_orange = np.array([30, 255, 255]) 

    mask = cv2.inRange(hsv_img, lower_orange, upper_orange)

    num_labels, _, stats, _ = cv2.connectedComponentsWithStats(mask)

    min_area = 400
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        if area > min_area:
            return True

    return False

# Main loop
prev_speed_left = 0
prev_speed_right = 0

start_time = time.time()
try:
    while time.time() - start_time < video_duration:
        # Capture frame from the camera
        frame = picam2.capture_array()
        
        if detect_duck(frame) == True:
            robot.stopcar()
            print("Waiting for the duck to be removed...")
            continue

        # Preprocess the frame to find the line's centroid
        cx = preprocess_image(frame)

        if cx is not None:
            # Calculate the PID output for direction control
            direction_speed = pid_direction(cx)

            turn_scale = 1.0
            if cx < 80 or cx > 220:
                turn_scale = 1.2
            
            left_speed = int(max(0, min(max_speed, straight_speed - direction_speed*turn_scale)))
            right_speed = int(max(0, min(max_speed, straight_speed + direction_speed*turn_scale)))
            prev_speed_left = left_speed
            prev_speed_right = right_speed

            # Send commands to the robot
            robot.changespeed(left_speed, right_speed)
            robot.forward()
        else:
            #print("No line detected, stopping.")
            robot.stopcar()
            robot.changespeed(turn_speed, turn_speed)
            if prev_speed_left - prev_speed_right > -15000:
                robot.turnRight()

            elif prev_speed_left - prev_speed_right < 15000:
                robot.turnLeft()

            else:
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
