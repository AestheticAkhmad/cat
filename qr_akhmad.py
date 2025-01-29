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

## Working duck
# Robot parameters
turn_speed = 0x6FFF
straight_speed = 0x5FFF
max_speed = 0x6FFF

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

# def check_qr(frame):
#     detected_qr = frame
#     operation = ""

#     for code in decode(detected_qr):
#         operation += code.data.decode("utf-8")

#     return operation

# def check_qr(frame):
#     qr_detector = cv2.QRCodeDetector()

#     # Detect and decode QR code
#     data, bbox, _ = qr_detector.detectAndDecode(frame)

#     if bbox:
#         print("QR Code detected!")

# def check_qr(img):

#     # Initialize QR code detector
#     detector = cv2.QRCodeDetector()

#     # Detect and decode QR code
#     data, _, _ = detector.detectAndDecode(img)

#     #print("QR Code detected!")

#     if data:
#         print("Decoded QR Code Data:", data)
#         return data
#     else:
#         print("No data found.")
#         return ""

def check_qr(img):
    # Initialize QR code detector
    detector = cv2.QRCodeDetector()

    # Detect and decode QR code
    data, bbox, _ = detector.detectAndDecode(img)

    if bbox is not None:
        print("QR Code detected!")

        # Draw bounding box around QR code
        for i in range(len(bbox)):
            cv2.line(img, tuple(bbox[i][0]), tuple(bbox[(i + 1) % len(bbox)][0]), (0, 255, 0), 2)

        # Get the four corner points
        pts = bbox.reshape(4, 2)
        
        # Define new points for a top-down (bird’s-eye) view
        width = 300
        height = 300
        dst_pts = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]], dtype="float32")

        # Compute the perspective transformation matrix
        matrix = cv2.getPerspectiveTransform(pts, dst_pts)

        # Warp the image
        warped = cv2.warpPerspective(img, matrix, (width, height))

        # Try decoding again with the corrected image
        corrected_data, _, _ = detector.detectAndDecode(warped)

        if corrected_data:
            print("Decoded QR Code Data (After Correction):", corrected_data)
            return corrected_data
        else:
            print("QR Code detected, but no data found after correction.")
            return ""
    else:
        print("No QR code found.")
        return None

def rotate_robot(rotation):
    full_rotation_time = 2.5
    rotation_time = full_rotation_time * rotation

    prev_time = time.time()
    while prev_time + rotation_time >= time.time():
        robot.changespeed(turn_speed, turn_speed)
        robot.turnRight()

def do_qr_operation(operation):
    if operation == "car_rotate_720":
        print("Rotating the car 720 degrees.")
        rotate_robot(2)
    elif operation == "car_stop_10s":
        print("Stopping the car for 10 seconds.")
        wait_time = 10
        time.sleep(wait_time)
    elif operation == "car_turn_around":
        print("Rotating the car 180 degrees.")
        rotate_robot(0.5)

# Main loop
prev_speed_left = 0
prev_speed_right = 0

start_time = time.time()
try:
    while time.time() - start_time < video_duration:
        # Capture frame from the camera
        frame = picam2.capture_array()
        
        qr_operation = check_qr(frame)
        if qr_operation != "":
            robot.stopcar()
            print("QR code detected. Doing mentioned operation.", qr_operation)
            do_qr_operation(qr_operation)
            continue

        # Preprocess the frame to find the line's centroid
        cx = preprocess_image(frame)

        if cx is not None:
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
