import sys
import cv2


def main():
    img_path = sys.argv[1]
    img = cv2.imread(img_path)
    print(type(img))
    img.tofile('out_img.bin')


if __name__ == "__main__":
    main()
