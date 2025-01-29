import time
from classes import Robot

robot = Robot()
turn_speed = 0x6FFF
robot.changespeed(turn_speed, turn_speed)
current_time = time.time()
print("Before: ", time.time())
while time.time() - current_time <= 2.5:
    robot.turnRight()
print("After: ", time.time())

robot.stopcar()
