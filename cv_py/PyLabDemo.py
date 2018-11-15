#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PIL import Image
from pylab import *

im = array(Image.open('./data/lena.bmp'))

print im.shape, im.dtype

imshow(im)
x = [100, 100, 400, 400]
y = [200, 500, 200, 500]
plot(x, y, 'r*')
title('Plotting: "lena.bmp"')
axis('on')

print 'Please click 3 points'
x = ginput(3)
print 'you clicked: ', x


figure()
im = array(Image.open('./data/lena.bmp').convert('L'), 'f')

print im.shape, im.dtype

gray()
contour(im, origin='image')


figure()
hist(im.flatten(), 128)

show()
