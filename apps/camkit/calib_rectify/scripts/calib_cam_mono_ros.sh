#!/usr/bin/env bash

rosrun camera_calibration cameracalibrator.py \
    --size 7x6\
    --square 0.0288 \
    image:=/camera/image_raw
