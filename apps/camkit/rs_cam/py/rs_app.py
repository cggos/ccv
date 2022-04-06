#!/usr/bin/python
import sys
import os
import cv2
import numpy as np

if os.name == 'posix':
    import tty
    import termios

import rs_device
import rs_cam
from rs_imu import IMUWrapper
from rs_cam import CamWrapper

sys.path.append("..")

import imtools


def image_exposure_control(cam: CamWrapper, img_raw):
    # img_raw.shape[2]

    print("Exp Time [Color]: %s" % cam.get_exposure_time(1))
    print("Gain [Color]: %s" % cam.get_gain(1))

    bright_thr = 256 * 0.02
    bright_des = 256 * 0.4
    bright_cur = imtools.brightness_by_gray(img_raw)
    print("brightness 00: %f" % bright_cur)

    exp_time_min = 50

    if abs(bright_cur - bright_des) > bright_thr:
        print("> bright_thr")
        exp_time = cam.get_exposure_time(1)
        exp_time = exp_time * (bright_des / bright_cur)
        cam.set_exposure_time(exp_time, 1)
        if exp_time < exp_time_min:
            print("auto gain")
            cam.set_gain(0, 1) # auto gain


def use_imu(serial_no):
    imu = IMUWrapper()
    if not imu.enable_imu_device(serial_no):
        print('Failed to enable device.')
        return -1


def use_cam():
    cam = CamWrapper()

    print("sensor_profiles: %s" % cam.get_sensor_profiles())

    cam.start()
    cam.set_emitter_enable(False)
    # IR cam
    # cam.set_exposure_time(0, 0)
    # cam.set_gain(128, 0)
    # Color cam
    cam.set_exposure_time(20, 1)
    # cam.set_gain(16, 1) # max: 128

    print("IR emitter: %s" % cam.get_emitter_enable())
    print("Exp Time [IR]: %s" % cam.get_exposure_time(0))
    print("Exp Time [Color]: %s" % cam.get_exposure_time(1))
    print("Gain [IR]: %s" % cam.get_gain(0))
    print("Gain [Color]: %s" % cam.get_gain(1))

    try:
        while True:
            # print("brightness: %s" % cam.get_brightness())

            color_frame, depth_frame, ir1_frame, ir2_frame = cam.get_fames()

            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                cv2.imshow('RealSense Color', color_image)

            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                colorizer_depth = np.asanyarray(rs_cam.colorizer_depth(depth_frame).get_data())
                show_depths = np.hstack((depth_colormap, colorizer_depth))
                cv2.imshow('RealSense Depths', show_depths)

            if ir1_frame:
                ir1_image = np.asanyarray(ir1_frame.get_data())
                cv2.imshow('RealSense IR', ir1_image)

            image_exposure_control(cam, color_image)

            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        cam.stop()


def main():
    if any([help_str in sys.argv for help_str in ['-h', '--help', '/?']]):
        print("Usage:", sys.argv[0], "[Options]")
        print
        print('[Options]:')
        print('-i : /path/to/accel.txt [/path/to/gyro.txt]')
        print('-s : serial number of device to calibrate.')
        print('-g : show graph of norm values - original values in blue and corrected in green.')
        print
        print('If -i option is given, calibration is done using previosly saved files')
        print('Otherwise, an interactive process is followed.')
        sys.exit(1)

    try:
        serial_no = ''
        for idx in range(len(sys.argv)):
            if sys.argv[idx] == '-s':
                serial_no = sys.argv[idx + 1]

        print('waiting for realsense device...')

        rs_device.wait_for_rs_device(serial_no)

        old_settings = None
        print('os.name: %s' % os.name)
        if os.name == 'posix':
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        use_cam()

        if os.name == 'posix':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    except Exception as e:
        print('\nError: %s' % e)
    finally:
        if os.name == 'posix' and old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    main()
