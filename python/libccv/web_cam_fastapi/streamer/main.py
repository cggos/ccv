import cv2
import zmq
import time
import base64
from camera import Camera

# 1. Initialize ZeroMQ Context and Publisher
context = zmq.Context()
publisher = context.socket(zmq.PUB)
# Bind to all interfaces on port 5555 using TCP
publisher.bind("tcp://0.0.0.0:5555")

print("Streamer (ZMQ) initialized at tcp://0.0.0.0:5555")

# 2. Initialize Camera
try:
    camera = Camera(0)
    print("Camera initialized.")
except Exception as e:
    print(f"Error opening camera: {e}")
    exit(1)

# 3. Initialize Pedestrian Detector (HOG)
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


def image_processing(frame):
    # --- Algorithm: Pedestrian Detection ---
    # Resize for faster processing if needed
    # frame = cv2.resize(frame, (640, 480))
    
    # Detect people
    # weights, boxes via detectMultiScale
    boxes, weights = hog.detectMultiScale(frame, winStride=(8, 8), padding=(8, 8), scale=1.05)
    
    # Draw bounding boxes
    for (x, y, w, h) in boxes:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    # Add text
    cv2.putText(frame, f'People Count: {len(boxes)}', (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    return frame


def main_loop():
    print("Starting processing loop... (Press Ctrl+C to stop)")
    try:
        while True:
            frame = camera.get_frame()
            if frame is None:
                time.sleep(0.1)
                continue
            
            frame = image_processing(frame)
            
            # --- Network: Publish Frame via ZMQ ---
            # Encode frame to JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            
            # Publish data
            # Topic: "video", Payload: bytes
            publisher.send_multipart([b"video", buffer.tobytes()])
            
            # Control framerate (optional, prevents flooding)
            # time.sleep(0.01)            
    except KeyboardInterrupt:
        print("Stopping streamer...")
    finally:
        camera.release()
        publisher.close()
        context.term()


if __name__ == "__main__":
    main_loop()
