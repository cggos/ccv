#!/usr/bin/env python

import sys
import cv2

if __name__ == '__main__':
    img_dir = sys.argv[1]
    idx_s = sys.argv[2]

    cap = cv2.VideoCapture(4)
    width = 1280
    height = 720

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    i = int(idx_s)
    while True:
        ret, frame = cap.read()
        k = cv2.waitKey(1)
        if k == 27:
            break
        elif k == ord('s'):
            img_name = img_dir + f"/rs_{i:0>5d}.jpg"  # jpg for PASCAL VOC dataset format
            print(f"saved {img_name}")
            cv2.imwrite(img_name, frame)
            i += 1
        cv2.imshow("capture", frame)
    cap.release()
    cv2.destroyAllWindows()
