# !/usr/bin/env python
# -*- coding:utf-8 -*-

from setuptools import setup
# or
# from distutils.core import setup
from setuptools import find_packages

setup(
    name='libccv',
    version='1.0.0',
    description='Open Chenguang Computer Vision library',

    author='Gavin Gao',
    author_email='cggos@outlook.com',
    url='https://cv.cgabc.xyz',

    packages=find_packages(),
    platforms=["any"],
    install_requires=['numpy', 'scipy', 'matplotlib', 'pillow', 'opencv-python', 'pydot'],
    python_requires='>3.6',
    entry_points={
        'console_scripts': [
            'ccv_img_stat = libccv.img_statistic:main',
            'ccv_img_brightness = libccv.img_brightness:main',
            'ccv_img_to_bin = libccv.img_to_bin:main',
            'ccv_img_fisheye_mask = libccv.img_fisheye_mask:main',
            'ccv_img_match_graph = libccv.img_match_graph:main',
            'ccv_pil_demo = libccv.pil_demo:main',
            'ccv_pylab_demo = libccv.pylab_demo:main',
            'ccv_hough_detect_line = libccv.hough_detect_line:main'
        ]
    }
)
