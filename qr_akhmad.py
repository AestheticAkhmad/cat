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
straight_speed = 0x5FFF
max_speed = 0x6FFF

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

# def correct_perspective(img, bbox):
#     """ Warps the QR code region to a frontal view. """
#     if bbox is None or len(bbox) != 1:
#         print("No valid QR")
#         return img
    
#     bbox = bbox[0]

#     # Define the destination points for a square warp
#     width = 150
#     height = 150
#     dst_pts = np.array([
#         [0, 0],
#         [width - 1, 0],
#         [width - 1, height - 1],
#         [0, height - 1]
#     ], dtype="float32")

#     # Compute the perspective transform matrix
#     M = cv2.getPerspectiveTransform(bbox.astype("float32"), dst_pts)

#     # Apply the perspective warp
#     corrected = cv2.warpPerspective(img, M, (width, height))
    
#     return corrected

# def check_qr(img):
#     # Initialize OpenCV's QR Code detector
#     detector = cv2.QRCodeDetector()

#     # Detect QR code and bounding box
#     data, bbox, _ = detector.detectAndDecode(img)

#     if bbox is not None:
#         print("QR Code detected!")

#         corrected_img = correct_perspective(img, bbox)

#         data, _, _ = detector.detectAndDecode(corrected_img)

#         if data:
#             print("Decoded QR Code Data:", data)
#             return data
#         else:
#             print("QR Code detected, but no data found after correction.")
#             return ""
#     else:
#         print("No QR code found.")
#         return ""


def reorder_points(pts):
    """Reorders the points to ensure correct perspective transformation."""
    rect = np.zeros((4, 2), dtype="float32")
    
    # Sum of (x, y) coordinates: top-left has the smallest sum, bottom-right has the largest
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  # Top-left
    rect[2] = pts[np.argmax(s)]  # Bottom-right

    # Difference of (x, y) coordinates: top-right has the smallest difference, bottom-left has the largest
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # Top-right
    rect[3] = pts[np.argmax(diff)]  # Bottom-left

    return rect

def correct_perspective(img, bbox):
    """ Warps the QR code region to a frontal view. """
    if bbox is None or len(bbox) != 1:
        print("No valid QR")
        return img
    
    bbox = reorder_points(bbox[0])  # Reorder points

    # Increase resolution for better clarity
    width, height = 200, 200  
    dst_pts = np.array([
        [0, 0], 
        [width - 1, 0], 
        [width - 1, height - 1], 
        [0, height - 1]
    ], dtype="float32")

    # Compute the perspective transform matrix
    M = cv2.getPerspectiveTransform(bbox.astype("float32"), dst_pts)

    # Apply the perspective warp
    corrected = cv2.warpPerspective(img, M, (width, height))
    
    return corrected

def preprocess_qr(image):
    """Converts image to grayscale and applies sharpening to improve readability."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    sharpened = cv2.GaussianBlur(gray, (3, 3), 0)
    return sharpened

def check_qr(img):
    """Detects and decodes a QR code from an image."""
    detector = cv2.QRCodeDetector()

    # Detect QR code and bounding box
    data, bbox, _ = detector.detectAndDecode(img)

    if bbox is not None:
        print("QR Code detected!")

        corrected_img = correct_perspective(img, bbox)
        preprocessed_img = preprocess_qr(corrected_img)  # Apply preprocessing

        # Save for debugging
        cv2.imwrite("corrected_qr.png", corrected_img)

        data, _, _ = detector.detectAndDecode(preprocessed_img)

        if data:
            print("Decoded QR Code Data:", data)
            return data
        else:
            print("QR Code detected, but no data found after correction.")
            return ""
    else:
        print("No QR code found.")
        return ""


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

frame_count = 0

start_time = time.time()
try:
    while time.time() - start_time < video_duration:
        # Capture frame from the camera
        frame = picam2.capture_array()

       # frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) # Correct color order for opencv

        # Save the frame as a PNG.  Use a unique filename.
        #filename = f"frame_{frame_count}.png"  # Use f-strings for cleaner formatting
        #cv2.imwrite(filename, frame_bgr) # Save the frame to a file

        #frame_count += 1
        
        qr_operation = check_qr(frame)
        if qr_operation != "":
            robot.stopcar()
            print("QR code detected. Doing mentioned operation.", qr_operation)
            do_qr_operation(qr_operation)
            continue

        # Finding line centroid
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
    picam2.stop()
    cv2.destroyAllWindows()
    robot.stopcar()
    print("Driving done!")
