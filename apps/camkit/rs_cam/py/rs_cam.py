#!/usr/bin/python
import pyrealsense2 as rs


def colorizer_depth(depth_frame):
    colorizer = rs.colorizer()
    return colorizer.colorize(depth_frame)


class CamWrapper:
    def __init__(self):
        self.pipeline = None
        self.config = None
        self.sensors = None
        self.align = None

    def start(self, is_align=False):
        ctx = rs.context()
        device = ctx.devices[0]
        serial_number = device.get_info(rs.camera_info.serial_number)

        print("serial_number: %s" % serial_number)

        # Configure depth and color streams
        self.config = rs.config()
        self.config.enable_device(serial_number)

        # enable 2 sensors
        # self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # color: 424x240, 640x480, 1280x720; FPS: 6 10 15 30 60
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # 1280x720@6FPS, 640x480@15FPS, 848x480
        # self.config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
        # self.config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

        # Align objects
        if is_align:
            align_to = rs.stream.color
            self.align = rs.align(align_to)

        # Start streaming
        self.pipeline = rs.pipeline()
        self.pipeline.start(self.config)

        # 0: depth, 1: color
        self.sensors = self.pipeline.get_active_profile().get_device().query_sensors()

        for i in range(len(self.sensors)):
            sensor = self.sensors[i]
            if sensor.is_color_sensor():
                print("sensor %d is color sensor" % i)
            if sensor.is_depth_sensor():
                print("sensor %d is depth sensor" % i)

    def get_sensor_profiles(self, dev_idx=0):
        ctx = rs.context()
        device = ctx.devices[0]
        return device.sensors[dev_idx].profiles

    def get_fames(self):
        frames = self.pipeline.wait_for_frames()
        if self.align is not None:
            frames = self.align.process(frames)
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        ir1 = frames.get_infrared_frame(1)  # Left IR Camera, it allows 0, 1 or no input
        ir2 = None  # frames.get_infrared_frame(2)  # Right IR camera
        return color, depth, ir1, ir2

    def set_exposure_time(self, exposure_time, sensor_idx=1):
        if exposure_time == 0:
            self.sensors[sensor_idx].set_option(rs.option.enable_auto_exposure, True)
            self.sensors[sensor_idx].set_option(rs.option.auto_exposure_priority, True)
        else:
            self.sensors[sensor_idx].set_option(rs.option.enable_auto_exposure, False)
            self.sensors[sensor_idx].set_option(rs.option.exposure, exposure_time)

    def set_gain(self, gain_value, sensor_idx=1):
        if gain_value == 0:
            self.sensors[sensor_idx].set_option(rs.option.gain, True)
        else:
            self.sensors[sensor_idx].set_option(rs.option.gain, gain_value)

    def get_gain(self, sensor_idx=1):
        return self.sensors[sensor_idx].get_option(rs.option.gain)

    def get_exposure_time(self, color_frame):
        """
        for test
        Args:
            color_frame ():
        Returns:
        """
        if color_frame.supports_frame_metadata(rs.frame_metadata_value.actual_exposure):
            return color_frame.get_frame_metadata(rs.frame_metadata_value.actual_exposure)
        else:
            return 0

    def get_exposure_time(self, sensor_idx=1):
        return self.sensors[sensor_idx].get_option(rs.option.exposure)

    def get_brightness(self, sensor_idx=1):
        return self.sensors[sensor_idx].get_option(rs.option.brightness)

    def set_emitter_enable(self, is_enable=True):
        depth_sensor = self.sensors[0]
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, is_enable)

    def get_emitter_enable(self):
        depth_sensor = self.sensors[0]
        if depth_sensor.supports(rs.option.emitter_enabled):
            return depth_sensor.get_option(rs.option.emitter_enabled)
        else:
            return None

    def stop(self):
        # Stop streaming
        self.pipeline.stop()
