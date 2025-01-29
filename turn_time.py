import time
from moving_forward import Robot

robot = Robot()

current_time = time.time()
print("Before: ", time.time())
while time.time() - current_time <= 2:
    robot.turnRight()
print("After: ", time.time())

robot.stopcar()
