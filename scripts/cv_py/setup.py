# !/usr/bin/env python
# -*- coding:utf-8 -*-

from setuptools import setup
# or
# from distutils.core import setup
from setuptools import find_packages

setup(
    name='occv',
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
            'occv_img_stat = occv.img_statistic:main',
            'occv_img_brightness = occv.img_brightness:main',
            'occv_img_to_bin = occv.img_to_bin:main',
            'occv_img_fisheye_mask = occv.img_fisheye_mask:main',
            'occv_img_match_graph = occv.img_match_graph:main',
            'occv_pil_demo = occv.pil_demo:main',
            'occv_pylab_demo = occv.pylab_demo:main',
            'occv_hough_detect_line = occv.hough_detect_line:main'
        ]
    }
)
