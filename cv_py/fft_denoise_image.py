#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
# import matplotlib.image as mpimg
from scipy import fftpack
from scipy import ndimage
from PIL import Image


def plot_spectrum(im_fft):
    from matplotlib.colors import LogNorm
    # A logarithmic colormap
    plt.imshow(np.abs(im_fft), norm=LogNorm(vmin=5))
    plt.colorbar()


image = Image.open('lena.bmp').convert('L')
im = np.asarray(image)

im_blur = ndimage.gaussian_filter(im, 4)

im_fft = fftpack.fft2(im)

im_fft2 = im_fft.copy()
r, c = im_fft2.shape

keep_fraction = 0.1
im_fft2[int(r*keep_fraction):int(r*(1-keep_fraction))] = 0
im_fft2[:, int(c*keep_fraction):int(c*(1-keep_fraction))] = 0

im_new = fftpack.ifft2(im_fft2).real


plt.figure()
plt.imshow(im_blur, plt.cm.gray)
plt.title('Blurred image')

plt.figure()
plot_spectrum(im_fft2)
plt.title('Fourier transform')

plt.figure()
plt.imshow(im_new, plt.cm.gray)
plt.title('Reconstructed Image')

plt.figure()
plt.imshow(im, cmap='gray')
plt.title('origin image')

plt.show()


