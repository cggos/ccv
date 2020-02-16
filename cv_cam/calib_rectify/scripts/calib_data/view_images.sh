#!/usr/bin/env bash

rosrun image_view image_view image:=/mynteye/left/image_raw &
rosrun image_view image_view image:=/mynteye/right/image_raw

# rosrun image_view image_view image:=/mynteye_throttle/left/image_raw &
# rosrun image_view image_view image:=/mynteye_throttle/right/image_raw
