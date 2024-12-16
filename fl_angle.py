import cv2
from picamera2 import Picamera2
import time
import math
from moving_forward import Robot

robot = Robot()

picam2 = Picamera2()
picam2.start()

# Video settings
frame_rate = 30
video_duration = 15
frame_width, frame_height = 640, 480

# Robot parameters
distance_to_line = 12  # Ground distance in cm, calibrated for 45-degree camera angle
turn_speed = 0x5FFF     # Adjust speed for turning
straight_speed = 0x7FFF

# Turn function
def turn(robot, angle):
    """Turns the robot based on the angle."""

    rad_angle = angle * (math.pi/180)
    #print(rad_angle)
    omega = math.pi*4
    t = abs(rad_angle / omega)
    print("turning time: ", t)

    if angle < 0:
        print(f"Turning Left by {abs(angle):.2f} degrees")
        curr_time = time.time()
        while time.time() - curr_time < t:
            #robot.pwm.channels[robot.ENA].duty_cycle = 0x5FFF + 0x1FFF
            robot.turnRight()
            #robot.turnLeft()
    elif angle > 0:
        print(f"Turning Right by {angle:.2f} degrees")
        robot.move_speed = turn_speed
        curr_time = time.time()
        while time.time() - curr_time < t:
            #robot.pwm.channels[robot.ENB].duty_cycle = 0x5FFF + 0x1FFF
            robot.turnLeft()
            #robot.turnRight()
    else:
        print("No Turn Needed")
    time.sleep(0.2)  # Adjust the sleep time for precise control
    robot.stopcar()

# Recording loop
start_time = time.time()
try:
    while time.time() - start_time < video_duration:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)

        # Define Region of Interest (ROI)
        height, width = gray.shape
        #roi = gray[int(height * 2 / 3):, :]  # Bottom third of the frame
        roi = gray  # Bottom third of the frame
        # Thresholding to detect black line
        _, binary = cv2.threshold(roi, 60, 255, cv2.THRESH_BINARY)

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

            # Estimate the turn angle using offset and distance_to_line
            angle = math.degrees(math.atan2(offset, distance_to_line))
            angle = angle
            print(f"Estimated Angle: {angle:.2f} degrees")

            # Turn or go straight based on the angle
            if abs(angle) < 25 or abs(offset) < 45:  # Small angle -> Go straight
                robot.move_speed = straight_speed
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