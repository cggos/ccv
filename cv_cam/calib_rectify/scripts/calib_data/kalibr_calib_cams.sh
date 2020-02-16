#!/usr/bin/env bash

calib_data_dir=calib_data

kalibr_calibrate_cameras \
    --target ${calib_data_dir}/aprilgrid.yaml \
    --bag images.bag \
    --bag-from-to 3 38 \
    --models pinhole-equi pinhole-equi \
    --topics /mynteye_throttle/left/image_raw /mynteye_throttle/right/image_raw
