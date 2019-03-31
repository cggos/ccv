#!/usr/bin/env bash

kalibr_calibrate_cameras \
    --bag mynteye-d-stereo.bag --bag-from-to 5 23 \
    --topics /mynteye/left/image_mono /mynteye/right/image_mono \
    --models ds-none ds-none \
    --target aprilgrid_6x6_36.yaml
