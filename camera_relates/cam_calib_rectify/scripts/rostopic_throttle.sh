#!/usr/bin/env bash

rosrun topic_tools throttle messages /mynteye/left/image_raw 5 /mynteye_throttle/left/image_raw &
rosrun topic_tools throttle messages /mynteye/right/image_raw 5 /mynteye_throttle/right/image_raw
