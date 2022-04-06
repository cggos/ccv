#!/usr/bin/python
from __future__ import print_function
import numpy as np
import binascii
import struct
import time
import pyrealsense2 as rs

max_float = struct.unpack('f', b'\xff\xff\xff\xff')[0]
max_int = struct.unpack('i', b'\xff\xff\xff\xff')[0]
max_uint8 = struct.unpack('B', b'\xff')[0]


def int_to_bytes(num, length=4, order='big'):
    res = bytearray(length)
    for i in range(length):
        res[i] = num & 0xff
        num >>= 8
    if num:
        raise OverflowError("Number {} doesn't fit into {} bytes.".format(num, length))
    if order == 'little':
        res.reverse()
    return res


def bytes_to_uint(bytes_array, order='little'):
    bytes_array = list(bytes_array)
    bytes_array.reverse()
    if order == 'little':
        return struct.unpack('>i', struct.pack('BBBB', *([0] * (4 - len(bytes_array))) + bytes_array))[0] & 0xffffffff
    else:
        return struct.unpack('>i', struct.pack('BBBB', *([0] * (4 - len(bytes_array))) + bytes_array))[0] & 0xffffffff


class CHeader:
    def __init__(self, version, table_type):
        self.buffer = np.ones(16, dtype=np.uint8) * 255
        self.buffer[0] = int(version[0], 16)
        self.buffer[1] = int(version[1], 16)
        self.buffer.dtype = np.uint16
        self.buffer[1] = int(table_type, 16)

    def size(self):
        return 16

    def set_data_size(self, size):
        self.buffer.dtype = np.uint32
        self.buffer[1] = size

    def set_crc32(self, crc32):
        self.buffer.dtype = np.uint32
        self.buffer[3] = crc32 % (1 << 32)  # convert from signed to unsigned 32 bit

    def get_buffer(self):
        self.buffer.dtype = np.uint8
        return self.buffer


def bitwise_int_to_float(ival):
    return struct.unpack('f', struct.pack('i', ival))[0]


def bitwise_float_to_int(fval):
    return struct.unpack('i', struct.pack('f', fval))[0]


def parse_buffer(buffer):
    cmd_size = 24
    header_size = 16

    buffer.dtype = np.uint32
    tab1_size = buffer[3]
    buffer.dtype = np.uint8
    print('tab1_size (all_data): ', tab1_size)

    tab1 = buffer[cmd_size:cmd_size + tab1_size]  # 520 == epprom++
    tab1.dtype = np.uint32
    tab2_size = tab1[1]
    tab1.dtype = np.uint8
    print('tab2_size (calibration_table): ', tab2_size)

    tab2 = tab1[header_size:header_size + tab2_size]  # calibration table
    tab2.dtype = np.uint32
    tab3_size = tab2[1]
    tab2.dtype = np.uint8
    print('tab3_size (calibration_table): ', tab3_size)

    tab3 = tab2[header_size:header_size + tab3_size]  # D435 IMU Calib Table
    tab3.dtype = np.uint32
    tab4_size = tab3[1]
    tab3.dtype = np.uint8
    print('tab4_size (D435_IMU_Calib_Table): ', tab4_size)

    tab4 = tab3[header_size:header_size + tab4_size]  # calibration data
    return tab1, tab2, tab3, tab4


def get_IMU_Calib_Table(X, product_line):
    version = ['0x02', '0x01']
    table_type = '0x20'

    if product_line == 'L500':
        version = ['0x05', '0x01']
        table_type = '0x243'

    header = CHeader(version, table_type)

    header_size = header.size()
    data_size = 37 * 4 + 96
    size_of_buffer = header_size + data_size  # according to table "D435 IMU Calib Table" here: https://user-images.githubusercontent.com/6958867/50902974-20507500-1425-11e9-8ca5-8bd2ac2d0ea1.png
    assert (size_of_buffer % 4 == 0)
    buffer = np.ones(size_of_buffer, dtype=np.uint8) * 255

    use_extrinsics = False
    use_intrinsics = True

    data_buffer = np.ones(data_size, dtype=np.uint8) * 255
    data_buffer.dtype = np.float32

    data_buffer[0] = bitwise_int_to_float(np.int32(int(use_intrinsics)) << 8 |
                                          np.int32(int(use_extrinsics)))

    intrinsic_vector = np.zeros(24, dtype=np.float32)
    intrinsic_vector[:9] = X[:3, :3].T.flatten()
    intrinsic_vector[9:12] = X[:3, 3]
    intrinsic_vector[12:21] = X[3:, :3].flatten()
    intrinsic_vector[21:24] = X[3:, 3]

    data_buffer[13:13 + X.size] = intrinsic_vector
    data_buffer.dtype = np.uint8

    header.set_data_size(data_size)

    header.set_crc32(binascii.crc32(data_buffer))
    buffer[:header_size] = header.get_buffer()
    buffer[header_size:] = data_buffer
    return buffer


def get_calibration_table(d435_imu_calib_table):
    version = ['0x02', '0x00']
    table_type = '0x20'

    header = CHeader(version, table_type)

    d435_imu_calib_table_size = d435_imu_calib_table.size
    sn_table_size = 32
    data_size = d435_imu_calib_table_size + sn_table_size

    header_size = header.size()
    size_of_buffer = header_size + data_size  # according to table "D435 IMU Calib Table" in "https://sharepoint.ger.ith.intel.com/sites/3D_project/Shared%20Documents/Arch/D400/FW/D435i_IMU_Calibration_eeprom_0_52.xlsx"
    assert (size_of_buffer % 4 == 0)
    buffer = np.ones(size_of_buffer, dtype=np.uint8) * 255

    data_buffer = np.ones(data_size, dtype=np.uint8) * 255
    data_buffer[:d435_imu_calib_table_size] = d435_imu_calib_table

    header.set_data_size(data_size)
    header.set_crc32(binascii.crc32(data_buffer))

    buffer[:header_size] = header.get_buffer()
    buffer[header_size:header_size + data_size] = data_buffer
    return buffer


def get_eeprom(calibration_table):
    version = ['0x01', '0x01']
    table_type = '0x09'

    header = CHeader(version, table_type)

    DC_MM_EEPROM_SIZE = 520
    # data_size = calibration_table.size

    header_size = header.size()
    size_of_buffer = DC_MM_EEPROM_SIZE
    data_size = size_of_buffer - header_size
    # size_of_buffer = header_size + data_size

    assert (size_of_buffer % 4 == 0)
    buffer = np.ones(size_of_buffer, dtype=np.uint8) * 255

    header.set_data_size(data_size)
    buffer[header_size:header_size + calibration_table.size] = calibration_table
    header.set_crc32(binascii.crc32(buffer[header_size:]))

    buffer[:header_size] = header.get_buffer()

    return buffer


def write_eeprom_to_camera(eeprom, serial_no=''):
    # DC_MM_EEPROM_SIZE = 520
    DC_MM_EEPROM_SIZE = eeprom.size
    DS5_CMD_LENGTH = 24

    MMEW_Cmd_bytes = b'\x14\x00\xab\xcd\x50\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'

    buffer = np.ones([DC_MM_EEPROM_SIZE + DS5_CMD_LENGTH, ], dtype=np.uint8) * 255
    cmd = np.array(struct.unpack('I' * 6, MMEW_Cmd_bytes), dtype=np.uint32)
    cmd.dtype = np.uint16
    cmd[0] += DC_MM_EEPROM_SIZE
    cmd.dtype = np.uint32
    cmd[3] = DC_MM_EEPROM_SIZE  # command 1 = 0x50
    # command 2 = 0
    # command 3 = size
    cmd.dtype = np.uint8
    buffer[:len(cmd)] = cmd
    buffer[len(cmd):len(cmd) + eeprom.size] = eeprom

    debug = get_debug_device(serial_no)
    if not debug:
        print('Error getting RealSense Device.')
        return
    # tab1, tab2, tab3, tab4 = parse_buffer(buffer)

    rcvBuf = debug.send_and_receive_raw_data(bytearray(buffer))
    if rcvBuf[0] == buffer[4]:
        print('SUCCESS: saved calibration to camera.')
    else:
        print('FAILED: failed to save calibration to camera.')
        print(rcvBuf)


def get_debug_device(serial_no):
    ctx = rs.context()
    devices = ctx.query_devices()
    found_dev = False
    for dev in devices:
        if len(serial_no) == 0 or serial_no == dev.get_info(rs.camera_info.serial_number):
            found_dev = True
            break
    if not found_dev:
        print('No RealSense device found' + str('.' if len(serial_no) == 0 else ' with serial number: ' + serial_no))
        return 0

    # print(a few basic information about the device)
    print('  Device PID: ', dev.get_info(rs.camera_info.product_id))
    print('  Device name: ', dev.get_info(rs.camera_info.name))
    print('  Serial number: ', dev.get_info(rs.camera_info.serial_number))
    print('  Firmware version: ', dev.get_info(rs.camera_info.firmware_version))
    debug = rs.debug_protocol(dev)
    return debug


def l500_send_command(dev, op_code, param1=0, param2=0, param3=0, param4=0, data=[], retries=1):
    for i in range(retries):
        try:
            debug_device = rs.debug_protocol(dev)
            gvd_command_length = 0x14 + len(data)
            magic_number1 = 0xab
            magic_number2 = 0xcd

            buf = bytearray()
            buf += bytes(int_to_bytes(gvd_command_length, 2))
            # buf += bytes(int_to_bytes(0, 1))
            buf += bytes(int_to_bytes(magic_number1, 1))
            buf += bytes(int_to_bytes(magic_number2, 1))
            buf += bytes(int_to_bytes(op_code))
            buf += bytes(int_to_bytes(param1))
            buf += bytes(int_to_bytes(param2))
            buf += bytes(int_to_bytes(param3))
            buf += bytes(int_to_bytes(param4))
            buf += bytearray(data)
            l = list(buf)
            res = debug_device.send_and_receive_raw_data(buf)

            if res[0] == op_code:
                res1 = res[4:]
                return res1
            else:
                raise Exception("send_command return error", res[0])
        except:
            if i < retries - 1:
                time.sleep(0.1)
            else:
                raise


def wait_for_rs_device(serial_no):
    ctx = rs.context()

    start = int(round(time.time() * 1000))
    now = int(round(time.time() * 1000))

    while now - start < 5000:
        devices = ctx.query_devices()
        for dev in devices:
            pid = str(dev.get_info(rs.camera_info.product_id))
            if len(serial_no) == 0 or serial_no == dev.get_info(rs.camera_info.serial_number):
                # print(a few basic information about the device)
                print('  Device PID: ', dev.get_info(rs.camera_info.product_id))
                print('  Device name: ', dev.get_info(rs.camera_info.name))
                print('  Serial number: ', dev.get_info(rs.camera_info.serial_number))
                print('  Product Line: ', dev.get_info(rs.camera_info.product_line))
                print('  Firmware version: ', dev.get_info(rs.camera_info.firmware_version))

                return dev
        time.sleep(5)
        now = int(round(time.time() * 1000))
    raise Exception('No RealSense device' + str('.' if len(serial_no) == 0 else ' with serial number: ' + serial_no))
