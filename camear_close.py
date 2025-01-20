from picamera2 import Picamera2
picam2 = Picamera2()
print("Camera initialized successfully!")
picam2.close()  # Important to release the camera

print(f"Error initializing camera: {e}")