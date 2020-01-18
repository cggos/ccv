#!/usr/bin/env bash

calib_data_dir=calib_data

kalibr_bagextractor \
    --image-topics /mynteye_throttle/left/image_raw /mynteye_throttle/right/image_raw \
    --output-folder images/ \
    --bag images_to_ext.bag
