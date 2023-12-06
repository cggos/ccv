#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from PIL import Image


def main():
    img_path = sys.argv[1]
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
