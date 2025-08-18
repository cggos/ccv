import cv2
import dlib
import matplotlib.pyplot as plt

# https://github.com/davisking/dlib-models/blob/master/mmod_human_face_detector.dat.bz2
dnnFaceDetector = dlib.cnn_face_detection_model_v1("/home/cg/Downloads/mmod_human_face_detector.dat")

gray = cv2.imread('/home/cg/Downloads/WIDER_train/images/13--Interview/13_Interview_Interview_Sequences_13_900.jpg', 0)

rects = dnnFaceDetector(gray, 1)

for (i, rect) in enumerate(rects):
    x1 = rect.rect.left()
    y1 = rect.rect.top()
    x2 = rect.rect.right()
    y2 = rect.rect.bottom()
    # Rectangle around the face
    cv2.rectangle(gray, (x1, y1), (x2, y2), (255, 255, 255), 3)

plt.figure(figsize=(12, 8))
plt.imshow(gray, cmap='gray')
plt.show()
