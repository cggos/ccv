import cv2
import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import StreamingResponse
from fastapi.templating import Jinja2Templates
import os
import contextlib

from camera import Camera
from processor import GaussianBlurProcessor

# Global variables for resources
camera_resource = None
processor_resource = None

@contextlib.asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize resources
    global camera_resource, processor_resource
    try:
        camera_resource = Camera(0)
        print("Camera initialized.")
    except Exception as e:
        print(f"Error initializing camera: {e}")
    
    processor_resource = GaussianBlurProcessor(kernel_size=(21, 21), sigma=0)
    print("Processor initialized.")
    
    yield
    
    # Shutdown: Clean up resources
    if camera_resource:
        camera_resource.release()
        print("Camera released.")

app = FastAPI(lifespan=lifespan)

# Ensure templates directory exists
templates_dir = os.path.join(os.path.dirname(__file__), "templates")
templates = Jinja2Templates(directory=templates_dir)

def generate_frames():
    global camera_resource, processor_resource
    
    if camera_resource is None:
        print("Camera not available.")
        return

    while True:
        frame = camera_resource.get_frame()
        if frame is None:
            break
        
        # Apply the Separate Algorithm
        processed_frame = processor_resource.process(frame)
        
        # Encode for Display (View logic)
        ret, buffer = cv2.imencode('.jpg', processed_frame)
        if not ret:
            continue
        
        frame_bytes = buffer.tobytes()
        
        # Yield frame for streaming
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.get("/")
def index(request: Request):
    """Serve the static website with real-time preview."""
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/video_feed")
def video_feed():
    """Stream audio/video."""
    return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
