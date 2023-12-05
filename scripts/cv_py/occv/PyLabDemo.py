#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PIL import Image
# from pylab import *

import matplotlib.pylab as pl

im = pl.array(Image.open('../../data/lena.bmp'))

print
im.shape, im.dtype

pl.imshow(im)
x = [100, 100, 400, 400]
y = [200, 500, 200, 500]
pl.plot(x, y, 'r*')
pl.title('Plotting: "lena.bmp"')
pl.axis('on')

print
'Please click 3 points'
x = pl.ginput(3)
print
'you clicked: ', x

pl.figure()
im = pl.array(Image.open('../../data/lena.bmp').convert('L'), 'f')

print
im.shape, im.dtype

pl.gray()
pl.contour(im, origin='image')

pl.figure()
pl.hist(im.flatten(), 128)

pl.show()
