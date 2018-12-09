#!/usr/bin/env bash

./build/calibrate -w 7 -h 6 -s 0.028 -n 42 -d ../ros_wrapper/src/capture_stereo_clib_imgs/imgs/ -l left -r right -o cam_stereo.yml
