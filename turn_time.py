import time
from moving_forward import Robot

robot = Robot()

start_time = time.time()
robot.turnRight()
end_time = time.time()

execution_time = end_time - start_time
print(f"Execution time: {execution_time:.4f} seconds")