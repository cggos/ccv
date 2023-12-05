# !/usr/bin/env python
# -*- coding:utf-8 -*-

from setuptools import setup
# or
# from distutils.core import setup
from setuptools import find_packages

setup(
    name='occv',
    version='1.0',
    description='Open Chenguang Computer Vision library',
    author='Gavin Gao',
    author_email='cggos@outlook.com',
    url='https://cv.cgabc.xyz',
    packages=find_packages(),
)
