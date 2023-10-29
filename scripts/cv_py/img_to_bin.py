import cv2

img = cv2.imread('/Users/gavin.gao/Pictures/lawn00.png')
print(type(img))
img.tofile('/Users/gavin.gao/Pictures/lawn00.bin')
