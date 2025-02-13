from simple_pid import PID
import cv2
from picamera2 import Picamera2
import time
import RPi.GPIO as GPIO
from classes_jannik import Robot
import numpy as np

# Initialize the robot and camera
robot = Robot()
picam2 = Picamera2()
picam2.start()

# Video and robot parameters
frame_rate = 30
video_duration = 120
frame_width, frame_height = 640, 480
turn_speed = 0x5FFF
straight_speed = 0x6FFF
max_speed = 0x7FFF

# Initialize PID controller for line following
pid_direction = PID(Kp=100, Ki=340, Kd=9, setpoint=frame_width // 2)
pid_direction.output_limits = (-max_speed // 2, max_speed // 2)

# Track previous speeds to handle turns
prev_speed_left = 0
prev_speed_right = 0

def preprocess_image(frame):
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

def detect_duck(frame):
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

def process_qr_code(frame):
    # Convert to grayscale and apply Gaussian blur
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    gray_blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    
    # Detect and decode QR codes in the frame
    qr_detector = cv2.QRCodeDetector()
    data, points, _ = qr_detector.detectAndDecode(frame)

    # Perform actions based on QR code instructions
    # 2.5 sec for 360°
    if data == "car_stop_10s":
        print("Instruction:", data)
        robot.stopcar()
        time.sleep(10)
    if data == "car_turn_around":
        print("Instruction:", data)
        robot.stopcar()
        qr_turn_speed = 0x6FFF
        robot.changespeed(qr_turn_speed, qr_turn_speed)
        time.sleep(1.25)
    if data == "car_rotate_720":
        print("Instruction:", data)
        robot.stopcar()
        qr_turn_speed = 0x6FFF
        robot.changespeed(qr_turn_speed, qr_turn_speed)
        time.sleep(5)
        return True
    return False

# Start the main loop
start_time = time.time()
try:
    while time.time() - start_time < video_duration:
        # Capture a frame from the camera
        frame = picam2.capture_array()
        
        # Stop if a duck is detected
        if detect_duck(frame) == True:
            robot.stopcar()
            continue
        
        # Process QR codes if detected
        if process_qr_code(frame) == True:
            continue

        # Preprocess the frame to find the centroid of the line
        cx = preprocess_image(frame)

        if cx is not None:
            # Adjust direction using the PID controller
            direction_speed = pid_direction(cx)
            
            # Scale turn speeds for sharp turns
            turn_scale = 1.0
            if cx < 80 or cx > 220:
                turn_scale = 1.2
            
            # Calculate motor speeds
            left_speed = int(max(0, min(max_speed, straight_speed - direction_speed * turn_scale)))
            right_speed = int(max(0, min(max_speed, straight_speed + direction_speed * turn_scale)))
            prev_speed_left = left_speed
            prev_speed_right = right_speed

            # Update the robot's movement
            robot.changespeed(left_speed, right_speed)
            robot.forward()
        else:
            # Stop and decide turning direction if no line is detected
            robot.stopcar()
            robot.changespeed(turn_speed, turn_speed)
            if prev_speed_left - prev_speed_right > -15000:
                robot.turnRight()
            elif prev_speed_left - prev_speed_right < 15000:
                robot.turnLeft()
            else:
                robot.stopcar()

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Cleanup resources after loop ends
    picam2.stop()
    cv2.destroyAllWindows()
    robot.stopcar()
    print("Driving done!")