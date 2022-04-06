#!/usr/bin/python
import os
import sys
import time
import enum
import threading
import numpy as np
import pyrealsense2 as rs

COLOR_RED = "\033[1;31m"
COLOR_BLUE = "\033[1;34m"
COLOR_CYAN = "\033[1;36m"
COLOR_GREEN = "\033[0;32m"
COLOR_RESET = "\033[0;0m"
COLOR_BOLD = "\033[;1m"
COLOR_REVERSE = "\033[;7m"

# g = 9.80665 # SI Gravity page 52 of https://nvlpubs.nist.gov/nistpubs/Legacy/SP/nistspecialpublication330e2008.pdf
g = 9.8016

is_data = None
get_key = None
if os.name == 'posix':
    import select
    import tty
    import termios

    is_data = lambda: select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
    get_key = lambda: sys.stdin.read(1)

elif os.name == 'nt':
    import msvcrt

    is_data = msvcrt.kbhit
    get_key = lambda: msvcrt.getch()

else:
    raise Exception('Unsupported OS: %s' % os.name)


class IMUWrapper:
    class Status(enum.Enum):
        idle = 0,
        rotate = 1,
        wait_to_stable = 2,
        collect_data = 3

    def __init__(self):
        self.pipeline = None
        self.imu_sensor = None
        self.status = self.Status(
            self.Status.idle)  # 0 - idle, 1 - rotate to position, 2 - wait to stable, 3 - pick data
        self.thread = threading.Condition()
        self.step_start_time = time.time()
        self.time_to_stable = 3
        self.time_to_collect = 2
        self.samples_to_collect = 1000
        self.rotating_threshold = 0.1
        self.moving_threshold_factor = 0.1
        self.collected_data_gyro = []
        self.collected_data_accel = []
        self.callback_lock = threading.Lock()
        self.max_norm = np.linalg.norm(np.array([0.5, 0.5, 0.5]))
        self.line_length = 20
        self.is_done = False
        self.is_data = False

    def escape_handler(self):
        self.thread.acquire()
        self.status = self.Status.idle
        self.is_done = True
        self.thread.notify()
        self.thread.release()
        sys.exit(-1)

    def imu_callback(self, frame):
        if not self.is_data:
            self.is_data = True

        with self.callback_lock:
            try:
                if is_data():
                    c = get_key()
                    if c == '\x1b':  # x1b is ESC
                        self.escape_handler()

                if self.status == self.Status.idle:
                    return
                pr = frame.get_profile()
                data = frame.as_motion_frame().get_motion_data()
                data_np = np.array([data.x, data.y, data.z])
                elapsed_time = time.time() - self.step_start_time

                ## Status.collect_data
                if self.status == self.Status.collect_data:
                    sys.stdout.write('\r %15s' % self.status)
                    part_done = len(self.collected_data_accel) / float(self.samples_to_collect)
                    # sys.stdout.write(': %-3.1f (secs)' % (self.time_to_collect - elapsed_time))

                    color = COLOR_GREEN
                    if pr.stream_type() == rs.stream.gyro:
                        self.collected_data_gyro.append(np.append(frame.get_timestamp(), data_np))
                        is_moving = any(abs(data_np) > self.rotating_threshold)
                    else:
                        is_in_norm = np.linalg.norm(data_np - self.crnt_bucket) < self.max_norm
                        if is_in_norm:
                            self.collected_data_accel.append(np.append(frame.get_timestamp(), data_np))
                        else:
                            color = COLOR_RED
                        is_moving = abs(np.linalg.norm(data_np) - g) / g > self.moving_threshold_factor

                        sys.stdout.write(color)
                        sys.stdout.write('[' + '.' * int(part_done * self.line_length) + ' ' * int(
                            (1 - part_done) * self.line_length) + ']')
                        sys.stdout.write(COLOR_RESET)

                    if is_moving:
                        print('WARNING: MOVING')
                        self.status = self.Status.rotate
                        return

                    # if elapsed_time > self.time_to_collect:
                    if part_done >= 1:
                        self.status = self.Status.collect_data
                        sys.stdout.write('\n\nDirection data collected.')
                        self.thread.acquire()
                        self.status = self.Status.idle
                        self.thread.notify()
                        self.thread.release()
                        return

                if pr.stream_type() == rs.stream.gyro:
                    return
                sys.stdout.write('\r %15s' % self.status)
                crnt_dir = np.array(data_np) / np.linalg.norm(data_np)
                crnt_diff = self.crnt_direction - crnt_dir
                is_in_norm = np.linalg.norm(data_np - self.crnt_bucket) < self.max_norm

                ## Status.rotate
                if self.status == self.Status.rotate:
                    sys.stdout.write(': %35s' % (np.array2string(crnt_diff, precision=4, suppress_small=True)))
                    sys.stdout.write(': %35s' % (np.array2string(abs(crnt_diff) < 0.1)))
                    if is_in_norm:
                        self.status = self.Status.wait_to_stable
                        sys.stdout.write('\r' + ' ' * 90)
                        self.step_start_time = time.time()
                        return

                ## Status.wait_to_stable
                if self.status == self.Status.wait_to_stable:
                    sys.stdout.write(': %-3.1f (secs)' % (self.time_to_stable - elapsed_time))
                    if not is_in_norm:
                        self.status = self.Status.rotate
                        return
                    if elapsed_time > self.time_to_stable:
                        self.collected_data_gyro = []
                        self.collected_data_accel = []
                        self.status = self.Status.collect_data
                        self.step_start_time = time.time()
                        return
                return
            except Exception as e:
                print('ERROR?' + str(e))
                self.thread.acquire()
                self.status = self.Status.idle
                self.thread.notify()
                self.thread.release()

    def get_measurements(self, buckets, bucket_labels):
        measurements = []
        print('-------------------------')
        print('*** Press ESC to Quit ***')
        print('-------------------------')
        for bucket, bucket_label in zip(buckets, bucket_labels):
            self.crnt_bucket = np.array(bucket)
            self.crnt_direction = np.array(bucket) / np.linalg.norm(np.array(bucket))
            print('\nAlign to direction: ', self.crnt_direction, ' ', bucket_label)
            self.status = self.Status.rotate
            self.thread.acquire()
            while (not self.is_done and self.status != self.Status.idle):
                self.thread.wait(3)
                if not self.is_data:
                    raise Exception('No IMU data. Check connectivity.')
            if self.is_done:
                raise Exception('User Abort.')
            measurements.append(np.array(self.collected_data_accel))
        return np.array(measurements), np.array(self.collected_data_gyro)

    def enable_imu_device(self, serial_no):
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial_no)
        try:
            self.pipeline.start(cfg)
        except Exception as e:
            print('ERROR: ', str(e))
            return False

        # self.sync_imu_by_this_stream = rs.stream.any
        active_imu_profiles = []

        active_profiles = dict()
        self.imu_sensor = None
        for sensor in self.pipeline.get_active_profile().get_device().sensors:
            for pr in sensor.get_stream_profiles():
                if pr.stream_type() == rs.stream.gyro and pr.format() == rs.format.motion_xyz32f:
                    active_profiles[pr.stream_type()] = pr
                    self.imu_sensor = sensor
                if pr.stream_type() == rs.stream.accel and pr.format() == rs.format.motion_xyz32f:
                    active_profiles[pr.stream_type()] = pr
                    self.imu_sensor = sensor
            if self.imu_sensor:
                break
        if not self.imu_sensor:
            print('No IMU sensor found.')
            return False
        print('\n'.join(['FOUND %s with fps=%s' % (str(ap[0]).split('.')[1].upper(), ap[1].fps()) for ap in
                         active_profiles.items()]))
        active_imu_profiles = list(active_profiles.values())
        if len(active_imu_profiles) < 2:
            print('Not all IMU streams found.')
            return False
        self.imu_sensor.stop()
        self.imu_sensor.close()
        self.imu_sensor.open(active_imu_profiles)
        self.imu_start_loop_time = time.time()
        self.imu_sensor.start(self.imu_callback)

        # Make the device use the original IMU values and not already calibrated:
        if self.imu_sensor.supports(rs.option.enable_motion_correction):
            self.imu_sensor.set_option(rs.option.enable_motion_correction, 0)
        return True
