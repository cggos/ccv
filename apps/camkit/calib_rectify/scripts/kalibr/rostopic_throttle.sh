#!/usr/bin/env bash

rosrun topic_tools throttle messages /D435I/infra1/image_rect_raw  5 /D435I/infra1/image_rect_raw_throttle &
rosrun topic_tools throttle messages /T265/fisheye1/image_raw 5 /T265/fisheye1/image_raw_throttle 

