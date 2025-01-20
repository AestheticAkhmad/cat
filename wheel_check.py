import cv2
import time
import math
from moving_forward import Robot
import numpy as np

robot = Robot()

# Robot parameters
left_speed = 0x1FFF     # Adjust speed for turning
right_speed = 0x8FFF
video_duration = 10

start_time = time.time()
try:
    while time.time() - start_time < video_duration:
        robot.changespeed(left_speed, right_speed)
        robot.forward()

finally:
    robot.stopcar()
        