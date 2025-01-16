import cv2
from picamera2 import Picamera2
import time
import math
from moving_forward import Robot
import numpy as np

robot = Robot()

picam2 = Picamera2()
picam2.start()

# Video settings
frame_rate = 30
video_duration = 300
frame_width, frame_height = 640, 480

# Robot parameters
distance_to_line = 12  # Ground distance in cm, calibrated for 45-degree camera angle
turn_speed = 0x2FFF     # Adjust speed for turning
straight_speed = 0x7FFF

# Turn function
def turn(robot, angle):
    """Turns the robot based on the angle."""

    rad_angle = angle * (math.pi/180)
    #print(rad_angle)
    omega = math.pi*4
    t = abs(rad_angle / omega) - 0.075
    print("turning time: ", t)

    if rad_angle < 0:
        print(f"Turning Left by {abs(rad_angle):.2f} degrees")
        curr_time = time.time()
        robot.changespeed(turn_speed, straight_speed)
        while time.time() - curr_time < t:
            robot.turnLeft()
        
    elif rad_angle > 0:
        print(f"Turning Right by {rad_angle:.2f} degrees")
        robot.move_speed = turn_speed
        curr_time = time.time()
        robot.changespeed(straight_speed, turn_speed)
        while time.time() - curr_time < t:
            robot.turnRight()
    else:
        print("No Turn Needed")
    time.sleep(0.008)  # Adjust the sleep time for precise control
    robot.stopcar()

angle_sum = 0
counter = 0
# Recording loop
start_time = time.time()
try:
    while time.time() - start_time < video_duration:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)

        # Define Region of Interest (ROI)
        height, width = gray.shape
        roi = gray[int(height * 2 / 3):, :]  # Bottom third of the frame
        #roi = gray  # Bottom third of the frame
        #print(roi)
        
        # Thresholding to detect black line
        _, binary = cv2.threshold(roi, 60, 255, cv2.THRESH_BINARY)
        binary = np.where(binary == 0, 1, 0).astype(binary.dtype)

        # Calculate centroid of the white region (black line)
        moments = cv2.moments(binary)
        cx, cy = -1, -1
        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            cv2.circle(roi, (cx, cy), 5, (255, 0, 0), -1)  # Draw centroid for debugging
            print("Centroid: ", cx)

            # Control logic for line following
            frame_center = width // 2
            offset = cx - frame_center  # Offset from center of the frame
            print(f"Offset: {offset}")

            kp = 0.8
            # Estimate the turn angle using offset and distance_to_line
            angle = kp * math.degrees(math.atan2(offset, distance_to_line))
            #angle = angle
            
            print(f"Estimated Angle: {angle:.2f} degrees")
            angle_sum += angle
            counter += 1

            # Turn or go straight based on the angle
            #if abs(angle_norm) < 1.5:  # Small angle -> Go straight
            if abs(offset) < 65:
                robot.changespeed(straight_speed, straight_speed)
                robot.forward()
                print("Go Straight")
            else:  # Larger angle -> Turn
                turn(robot, angle)

        else:
            print("No line detected, stopping.")
            robot.stopcar()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    print("Driving done!")


print(angle_sum / counter)