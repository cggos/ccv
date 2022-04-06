#!/usr/bin/env bash

rosrun camera_calibration cameracalibrator.py \
    --approximate=0.1 \
    --size 11x8 \
    --square 0.03 \
    right:=/stereo/right/image_raw \
    left:=/stereo/left/image_raw
