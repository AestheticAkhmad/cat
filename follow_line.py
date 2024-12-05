import cv2
from picamera2 import Picamera2
import time
from moving_forward import Robot

robot = Robot()

picam2 = Picamera2()
picam2.start()

# Video settings
frame_rate = 30
video_duration = 15
frame_width, frame_height = 640, 480
i = 0
# Recording loop
start_time = time.time()
try:
    while time.time() - start_time < video_duration:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        gray = cv2.GaussianBlur(gray, (7,7), 0)

        # Define Region of Interest (ROI)
        height, width = gray.shape
        roi = gray[int(height * 2 / 3):, :]  # Bottom third of the frame
        time.sleep(0.25)

        # Thresholding to detect black line
        _, binary = cv2.threshold(roi, 60, 255, cv2.THRESH_BINARY)
        #print(binary)
        # if cv2.imwrite(f'binary_output{i}.png', binary):
        #     print("Image saved successfully!")
        # else:
        #     print("Failed to save the image.")
        
        i += 1

        # Calculate centroid of the white region (black line)
        moments = cv2.moments(binary)
        cx, cy = -1, -1
        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            cv2.circle(roi, (cx, cy), 5, (255, 0, 0), -1)  # Draw centroid for debugging

        print(moments["m00"])
    
        # Control logic for line following
            frame_center = width // 2
            if cx > 0:
                offset = cx - frame_center
                print("offset: ", offset)

                if offset >= -5 and offset <= 5:
                    robot.move_speed = 0x7FFF
                    #robot.forward()
                    print("Go Straight")
                elif offset < -5:
                    robot.move_speed = 0x5FFF
                    #robot.turnRight()
                    print("Turn Left")
                elif offset > 5:
                    robot.move_speed = 0x5FFF
                    #robot.turnLeft()
                    print("Turn Right")
            else:
                time.sleep(0.5)
                robot.stopcar()
        
        # else:
        #     stopcar()
        #     print("stopped car")

        robot.stopcar()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


finally:
    picam2.stop()
    cv2.destroyAllWindows()
    print("Driving done!")