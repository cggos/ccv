import cv2
import numpy as np
import matplotlib.pyplot as plt

import dlib
from imutils import face_utils

gray = cv2.imread('/home/cg/Downloads/WIDER_train/images/13--Interview/13_Interview_Interview_Sequences_13_900.jpg', 0)
im = np.float32(gray) / 255.0
# Calculate gradient
gx = cv2.Sobel(im, cv2.CV_32F, 1, 0, ksize=1)
gy = cv2.Sobel(im, cv2.CV_32F, 0, 1, ksize=1)
mag, angle = cv2.cartToPolar(gx, gy, angleInDegrees=True)

plt.figure(figsize=(12, 8))
plt.imshow(mag)
plt.show()

face_detect = dlib.get_frontal_face_detector()
rects = face_detect(gray, 1)
for (i, rect) in enumerate(rects):
    (x, y, w, h) = face_utils.rect_to_bb(rect)
cv2.rectangle(gray, (x, y), (x + w, y + h), (255, 255, 255), 3)

plt.figure(figsize=(12, 8))
plt.imshow(gray, cmap='gray')
plt.show()
