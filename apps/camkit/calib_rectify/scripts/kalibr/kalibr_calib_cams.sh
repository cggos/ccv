#!/usr/bin/env bash

calib_data_dir=../../calib_data

kalibr_calibrate_cameras \
    --target ${calib_data_dir}/aprilgrid.yaml \
    --bag rs_fisheye_ircam_images.bag \
    --bag-from-to 3 58 \
    --models pinhole-radtan pinhole-equi \
    --topics /D435I/infra1/image_rect_raw_throttle /T265/fisheye1/image_raw_throttle
