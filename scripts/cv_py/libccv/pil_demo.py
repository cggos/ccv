#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
from PIL import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('img_in', help='input image')
    args = parser.parse_args()

    img_path = args.img_in
    pil_im = Image.open(img_path).convert('L')

    box = (50, 50, 400, 400)
    region = pil_im.crop(box)
    region = region.transpose(Image.ROTATE_180)
    pil_im.paste(region, box)

    pil_im = pil_im.rotate(45)

    pil_im.thumbnail((128, 128))

    pil_im = pil_im.resize((128, 128))

    pil_im.show()


if __name__ == "__main__":
    main()
