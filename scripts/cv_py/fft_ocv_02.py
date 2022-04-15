#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('../../data/lena.bmp', 0)

img_f_o = np.fft.fft2(img)
img_f_c = np.fft.fftshift(img_f_o)

img_amplitude = np.abs(img_f_c)
img_phase = np.angle(img_f_c)

img_real = img_amplitude * np.cos(img_phase)
img_imag = img_amplitude * np.sin(img_phase)

img_new_complex = np.zeros(img.shape, dtype=complex)
img_new_complex.real = np.array(img_real)
img_new_complex.imag = np.array(img_imag)

img_new_f_o = np.fft.ifftshift(img_new_complex)

img_new = np.fft.ifft2(img_new_f_o)
img_new = np.abs(img_new)

img_back = (img_new - np.amin(img_new)) / (np.amax(img_new) - np.amin(img_new))

plt.figure()
plt.imshow(img_back, 'gray'), plt.title('amplitude + phase')
plt.xticks([]), plt.yticks([])
plt.show()
