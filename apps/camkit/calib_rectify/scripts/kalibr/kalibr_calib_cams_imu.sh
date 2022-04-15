#!/usr/bin/env bash

calib_data_dir=calib_data

kalibr_calibrate_imu_camera \
    --target ${calib_data_dir}/aprilgrid.yaml \
    --cam camchain-images.yaml \
    --imu ${calib_data_dir}/imu_bmi088.yaml \
    --bag images_imu.bag \
    --bag-from-to 2 26 \
    --imu-models scale-misalignment \
    --timeoffset-padding 0.1
