"""
    pip install opencv-contrib-python
"""

import cv2
import matplotlib.pyplot as plt

font = cv2.FONT_HERSHEY_SIMPLEX

# haarcascades_path = os.path.dirname(cv2.__file__) + "../haarcascade_frontalface_default.xml"
# face_cascade = cv2.CascadeClassifier(haarcascades_path)

print(cv2.data)

face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
)
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_eye.xml")

# Load the image
gray = cv2.imread(
    "/home/cg/Downloads/WIDER_train/images/13--Interview/13_Interview_Interview_Sequences_13_900.jpg",
    0,
)
plt.figure(figsize=(12, 8))
plt.imshow(gray, cmap="gray")
plt.show()

# Detect faces
faces = face_cascade.detectMultiScale(
    gray, scaleFactor=1.1, minNeighbors=5, flags=cv2.CASCADE_SCALE_IMAGE
)

# For each face
for x, y, w, h in faces:
    # Draw rectangle around the face
    cv2.rectangle(gray, (x, y), (x + w, y + h), (255, 255, 255), 3)

plt.figure(figsize=(12, 8))
plt.imshow(gray, cmap="gray")
plt.show()
