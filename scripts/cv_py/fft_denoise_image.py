#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
# import matplotlib.image as mpimg
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from mpl_toolkits.mplot3d import Axes3D
from scipy import fftpack
from scipy import ndimage
from PIL import Image


def plot_spectrum(im_fft):
    from matplotlib.colors import LogNorm
    # A logarithmic colormap
    plt.imshow(np.abs(im_fft), norm=LogNorm(vmin=5))
    plt.colorbar()


img = Image.open('../../data/lena.bmp').convert('L')
im = np.asarray(img)

im_blur = ndimage.gaussian_filter(im, 4)

rows, cols = im.shape
mask = np.ones(im.shape, np.uint8)
# mask[rows/2-30:rows/2+30, cols/2-30:cols/2+30] = 1

# ==========================================
im_f_o = fftpack.fft2(im)
im_f_c = fftpack.fftshift(im_f_o)

im_f_filter_c = im_f_c * mask

im_f_filter_o = fftpack.ifftshift(im_f_filter_c)

im_new = fftpack.ifft2(im_f_filter_o).real

# ==========================================
im_f_c_2 = np.abs(im_f_c) ** 2

spectrum_o = np.log(np.abs(im_f_o))
spectrum_c = np.log(np.abs(im_f_filter_c))

phase_o = np.angle(im_f_o)
phase_c = np.angle(im_f_filter_c)

# ===========================================
plt.figure()
plt.subplot(221), plt.imshow(im, cmap='gray'), plt.title('origin image')
plt.subplot(222), plot_spectrum(im_f_filter_c), plt.title('Fourier transform')
plt.subplot(223), plt.imshow(abs(im_new), plt.cm.gray), plt.title('Reconstructed Image')
plt.subplot(224), plt.imshow(np.log10(im_f_c_2)), plt.title('Power Spectrum')

plt.figure()
plt.imshow(im_blur, plt.cm.gray)
plt.title('Gauss Blurred image')

plt.show()
