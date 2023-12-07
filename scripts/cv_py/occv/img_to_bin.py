import argparse
import cv2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('img_in', help='input image')
    args = parser.parse_args()

    img_path = args.img_in
    img = cv2.imread(img_path)
    print(type(img))
    img.tofile('out_img.bin')


if __name__ == "__main__":
    main()
