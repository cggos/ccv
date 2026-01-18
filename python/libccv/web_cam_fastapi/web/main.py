import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import StreamingResponse
from fastapi.templating import Jinja2Templates
import os
import zmq

app = FastAPI()

# Configuration
# Assuming the streamer is on localhost for this demo. 
# In a real LAN, use the IP of the machine running the streamer, e.g., "tcp://192.168.1.5:5555"
ZMQ_STREAMER_ADDRESS = "tcp://localhost:5555"

# Ensure templates directory exists
templates_dir = os.path.join(os.path.dirname(__file__), "templates")
templates = Jinja2Templates(directory=templates_dir)

def get_zmq_stream():
    """Connect using a separate ZMQ context for the generator."""
    # Note: ZMQ Contexts are thread-safe, but sockets are not. 
    # It's safest to create a new socket for this streaming connection or manage it carefully.
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.connect(ZMQ_STREAMER_ADDRESS)
    subscriber.setsockopt(zmq.SUBSCRIBE, b"video")
    
    print(f"Connected to ZMQ Stream at {ZMQ_STREAMER_ADDRESS}")
    
    try:
        while True:
            # Receive multipart: [topic, body]
            topic, frame_bytes = subscriber.recv_multipart()
            
            # Yield for MJPEG
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    except Exception as e:
        print(f"ZMQ Error: {e}")
    finally:
        subscriber.close()
        context.term()

@app.get("/")
def index(request: Request):
    """Serve the static website."""
    # We pass the same template
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/video_feed")
def video_feed():
    """Proxy the stream from the ZMQ Subscriber."""
    return StreamingResponse(get_zmq_stream(), media_type="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
