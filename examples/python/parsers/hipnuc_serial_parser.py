import struct
import logging

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Constant definitions
CHSYNC1 = 0x5A
CHSYNC2 = 0xA5
CH_HDR_SIZE = 6

GRAVITY = 9.80665
R2D = 57.29577951308232


# Data item identifiers
FRAME_TAG_HI91 = 0x91
FRAME_TAG_HI92 = 0x92
FRAME_TAG_HI81 = 0x81

class hipnuc_frame:
    def __init__(self):
        self.reset()

    def reset(self):
        self.temperature = None
        self.pressure = None
        self.system_time_ms = None
        self.acc = None
        self.gyr = None
        self.mag = None
        self.quat = None
        self.sync_time = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.ins_status = None
        self.gpst_wn = None
        self.gpst_tow = None
        self.utc_year = None
        self.utc_month = None
        self.utc_day = None
        self.utc_hour = None
        self.utc_min = None
        self.utc_msec = None
        self.ins_lon = None
        self.ins_lat = None
        self.ins_msl = None
        self.pdop = None
        self.hdop = None
        self.solq_pos = None
        self.nv_pos = None
        self.solq_heading = None
        self.nv_heading = None
        self.diff_age = None
        self.undulation = None
        self.vel_enu = None
        self.acc_enu = None

    def to_dict(self):
        """Return a dictionary representation of non-null fields"""
        return {k: v for k, v in self.__dict__.items() if v is not None}

class hipnuc_parser:
    def __init__(self):
        self.CHSYNC1 = CHSYNC1
        self.CHSYNC2 = CHSYNC2
        self.CH_HDR_SIZE = CH_HDR_SIZE
        self.buffer = bytearray()
        self.frame = hipnuc_frame()

    @staticmethod
    def crc16_update(crc, data):
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                temp = crc << 1
                if crc & 0x8000:
                    temp ^= 0x1021
                crc = temp
        return crc & 0xFFFF

    def parse_item(self, item_type, data, ofs):
        try:
            if item_type == FRAME_TAG_HI91:
                self._parse_hi91(data, ofs)
                return ofs + 76
            elif item_type == FRAME_TAG_HI92:
                self._parse_hi92(data, ofs)
                return ofs + 48
            elif item_type == FRAME_TAG_HI81:
                self._parse_hi81(data, ofs)
                return ofs + 104
            else:
                logging.warning(f"Unknown item type: {item_type}")
                return ofs + 1
        except struct.error as e:
            logging.error(f"Error parsing data: {e}")
            return ofs + 1

    def _parse_hi91(self, data, ofs):
        self.frame.sync_time = struct.unpack_from('<H', data, ofs + 1)[0] * 1e-3
        self.frame.temperature = struct.unpack_from('<b', data, ofs + 3)[0]
        self.frame.pressure = struct.unpack_from('<f', data, ofs + 4)[0]
        self.frame.system_time_ms = struct.unpack_from('<I', data, ofs + 8)[0]
        self.frame.acc = struct.unpack_from('<3f', data, ofs + 12)
        self.frame.gyr = struct.unpack_from('<3f', data, ofs + 24)
        self.frame.mag = struct.unpack_from('<3f', data, ofs + 36)
        self.frame.roll = struct.unpack_from('<f', data, ofs + 48)[0]
        self.frame.pitch = struct.unpack_from('<f', data, ofs + 52)[0]
        self.frame.yaw = struct.unpack_from('<f', data, ofs + 56)[0]
        self.frame.quat = struct.unpack_from('<4f', data, ofs + 60)

    def _parse_hi92(self, data, ofs):
        # self.frame.status = struct.unpack_from('<H', data, ofs + 1)[0]
        self.frame.temperature = struct.unpack_from('<b', data, ofs + 3)[0]
        self.frame.sync_time = struct.unpack_from('<H', data, ofs + 4)[0] * 1e-3
        self.frame.pressure = struct.unpack_from('<h', data, ofs + 6)[0] + 100000
        self.frame.gyr = [x * 0.001 * R2D for x in struct.unpack_from('<3h', data, ofs + 10)]
        self.frame.acc = [x * 0.0048828 / GRAVITY for x in struct.unpack_from('<3h', data, ofs + 16)]
        self.frame.mag = [x * 0.030517 for x in struct.unpack_from('<3h', data, ofs + 22)]
        self.frame.roll = struct.unpack_from('<i', data, ofs + 28)[0] * 0.001
        self.frame.pitch = struct.unpack_from('<i', data, ofs + 32)[0] * 0.001
        self.frame.yaw = struct.unpack_from('<i', data, ofs + 36)[0] * 0.001
        self.frame.quat = [x * 0.0001 for x in struct.unpack_from('<4h', data, ofs + 40)]
        
    def _parse_hi81(self, data, ofs):
        self.frame.ins_status = struct.unpack_from('<B', data, ofs + 3)[0]
        self.frame.gpst_wn = struct.unpack_from('<H', data, ofs + 4)[0]
        self.frame.gpst_tow = struct.unpack_from('<I', data, ofs + 6)[0] * 1e-3
        self.frame.sync_time = struct.unpack_from('<H', data, ofs + 10)[0] * 1e-3
        self.frame.gyr = [x * 0.001 * R2D for x in struct.unpack_from('<3h', data, ofs + 12)]  # Convert to degrees per second
        self.frame.acc = [x * 0.0048828 / GRAVITY for x in struct.unpack_from('<3h', data, ofs + 18)]  # Convert to G
        self.frame.mag = [x * 0.030517 for x in struct.unpack_from('<3h', data, ofs + 24)]  # Convert to uT
        self.frame.pressure = struct.unpack_from('<h', data, ofs + 30)[0] + 100000

        self.frame.temperature = struct.unpack_from('<b', data, ofs + 34)[0]
        self.frame.utc_year = struct.unpack_from('<B', data, ofs + 35)[0]
        self.frame.utc_month = struct.unpack_from('<B', data, ofs + 36)[0]
        self.frame.utc_day = struct.unpack_from('<B', data, ofs + 37)[0]
        self.frame.utc_hour = struct.unpack_from('<B', data, ofs + 38)[0]
        self.frame.utc_min = struct.unpack_from('<B', data, ofs + 39)[0]
        self.frame.utc_msec = struct.unpack_from('<H', data, ofs + 40)[0]
        self.frame.roll = struct.unpack_from('<h', data, ofs + 42)[0] * 0.01  # Convert to degrees
        self.frame.pitch = struct.unpack_from('<h', data, ofs + 44)[0] * 0.01  # Convert to degrees
        self.frame.yaw = struct.unpack_from('<H', data, ofs + 46)[0] * 0.01  # Convert to degrees
        self.frame.quat = [x * 0.0001 for x in struct.unpack_from('<4h', data, ofs + 48)]  # Convert to quaternion
        self.frame.ins_lon = struct.unpack_from('<i', data, ofs + 56)[0] * 1e-7  # Convert to degrees
        self.frame.ins_lat = struct.unpack_from('<i', data, ofs + 60)[0] * 1e-7  # Convert to degrees
        self.frame.ins_msl = struct.unpack_from('<i', data, ofs + 64)[0] * 1e-3  # Convert to meters
        self.frame.pdop = struct.unpack_from('<B', data, ofs + 68)[0] * 0.1  # Convert to unit
        self.frame.hdop = struct.unpack_from('<B', data, ofs + 69)[0] * 0.1  # Convert to unit
        self.frame.solq_pos = struct.unpack_from('<B', data, ofs + 70)[0]
        self.frame.nv_pos = struct.unpack_from('<B', data, ofs + 71)[0]
        self.frame.solq_heading = struct.unpack_from('<B', data, ofs + 72)[0]
        self.frame.nv_heading = struct.unpack_from('<B', data, ofs + 73)[0]
        self.frame.diff_age = struct.unpack_from('<B', data, ofs + 74)[0]
        self.frame.undulation = struct.unpack_from('<h', data, ofs + 75)[0] * 0.01  # Convert to meters
        self.frame.vel_enu = [x * 0.01 for x in struct.unpack_from('<3h', data, ofs + 78)]  # Convert to meters per second
        self.frame.acc_enu = [x * 0.0048828 / GRAVITY for x in struct.unpack_from('<3h', data, ofs + 84)]  # Convert to G
        self.frame.system_time_ms = struct.unpack_from('<I', data, ofs + 90)[0]

    def parse_data(self, data):
        """Parse data"""
        ofs = 0
        while ofs < len(data):
            item_type = data[ofs]
            ofs = self.parse_item(item_type, data, ofs)

    def parse(self, new_data):
        """Decode new data and return successfully parsed frames"""
        self.buffer += new_data
        frames = []
        while len(self.buffer) >= self.CH_HDR_SIZE:
            if self.buffer[0] == self.CHSYNC1 and self.buffer[1] == self.CHSYNC2:
                length = struct.unpack_from('<H', self.buffer, 2)[0]
                if len(self.buffer) >= self.CH_HDR_SIZE + length:
                    frame = self.buffer[:self.CH_HDR_SIZE + length]
                    crc_calculated = self.crc16_update(0, frame[:4] + frame[6:])
                    crc_received = struct.unpack_from('<H', frame, 4)[0]
                    if crc_calculated == crc_received:
                        self.frame.reset()  # Reset data
                        self.frame.frame_type = frame[6]  # 获取帧类型并保存到实例中
                        self.parse_data(frame[self.CH_HDR_SIZE:])
                        frames.append(self.frame)  # Add parsed IMU data to list
                    else:
                        logging.error("CRC check failed")
                    del self.buffer[:self.CH_HDR_SIZE + length]
                else:
                    break
            else:
                del self.buffer[0]
        return frames


    @staticmethod
    def print_parsed_data(data):
        """Format and print IMU data in a professional and compact format"""
        if data.frame_type is not None:
            data_fields = [
                ("Frame Type",             f"HI{data.frame_type:02X}"),
                ("Temperature (C)",        f"{data.temperature:<6}" if data.temperature is not None else None),
                ("Pressure (Pa)",          f"{data.pressure:<9.3f}" if data.pressure is not None else None),
                ("System_time_ms",         f"{data.system_time_ms:<9}" if data.system_time_ms is not None else None),
                ("Sync_time (s)",          f"{data.sync_time:<9.3f}" if data.sync_time is not None else None),
                ("Roll (deg)",             f"{data.roll:<9.3f}" if data.roll is not None else None),
                ("Pitch (deg)",            f"{data.pitch:<9.3f}" if data.pitch is not None else None),
                ("Yaw (deg)",              f"{data.yaw:<9.3f}" if data.yaw is not None else None),
                ("INS Status",             f"{data.ins_status:<9}" if data.ins_status is not None else None),
                ("GPS Week No.",           f"{data.gpst_wn:<9}" if data.gpst_wn is not None else None),
                ("GPS TOW (s)",            f"{data.gpst_tow:<9} s" if data.gpst_tow is not None else None),
                ("UTC Time",               f"20{data.utc_year:<2}-{data.utc_month:02}-{data.utc_day:02} {data.utc_hour:02}:{data.utc_min:02}:{data.utc_msec:06.3f}" if all([data.utc_year, data.utc_month, data.utc_day, data.utc_hour, data.utc_min, data.utc_msec]) else None),
                ("INS Longitude (deg)",    f"{data.ins_lon:<12.7f}" if data.ins_lon is not None else None),
                ("INS Latitude (deg)",     f"{data.ins_lat:<12.7f}" if data.ins_lat is not None else None),
                ("INS MSL (m)",            f"{data.ins_msl:<9.3f}" if data.ins_msl is not None else None),
                ("PDOP",                   f"{data.pdop:<9.1f}" if data.pdop is not None else None),
                ("HDOP",                   f"{data.hdop:<9.1f}" if data.hdop is not None else None),
                ("Position Quality",       f"{data.solq_pos:<9}" if data.solq_pos is not None else None),
                ("Sat No. ",               f"{data.nv_pos:<9}" if data.nv_pos is not None else None),
                ("Heading Quality",        f"{data.solq_heading:<9}" if data.solq_heading is not None else None),
                ("NV Heading",             f"{data.nv_heading:<9}" if data.nv_heading is not None else None),
                ("Diff Age",               f"{data.diff_age:<9}" if data.diff_age is not None else None),
                ("Undulation",             f"{data.undulation:<9}" if data.undulation is not None else None),
                ("Acceleration (G)",       f"({data.acc[0]:<9.3f}, {data.acc[1]:<9.3f}, {data.acc[2]:<9.3f})" if data.acc is not None else None),
                ("Gyroscope (deg/s)",        f"({data.gyr[0]:<9.3f}, {data.gyr[1]:<9.3f}, {data.gyr[2]:<9.3f})" if data.gyr is not None else None),
                ("Magnetometer (uT)",      f"({data.mag[0]:<9.3f}, {data.mag[1]:<9.3f}, {data.mag[2]:<9.3f})" if data.mag is not None else None),
                ("Quaternion",             f"({data.quat[0]:<9.3f}, {data.quat[1]:<9.3f}, {data.quat[2]:<9.3f}, {data.quat[3]:<9.3f})" if data.quat is not None else None),
                ("Velocity ENU (m/s)",     f"({data.vel_enu[0]:<9.3f}, {data.vel_enu[1]:<9.3f}, {data.vel_enu[2]:<9.3f})" if data.vel_enu is not None else None),
                ("Acceleration ENU (m/s²)", f"({data.acc_enu[0]:<9.3f}, {data.acc_enu[1]:<9.3f}, {data.acc_enu[2]:<9.3f})" if data.acc_enu is not None else None),
            ]

            # Print the data fields
            for label, value in data_fields:
                if value is not None:
                    print(f"{label:<24}: {value}")





# Example usage
if __name__ == "__main__":
    decoder = hipnuc_parser()
    
    example_data_hi91 = bytes.fromhex('5A A5 4C 00 14 BB 91 08 15 23 09 A2 C4 47 08 15 1C 00 CC E8 61 BE 9A 35 56 3E 65 EA 72 3F 31 D0 7C BD 75 DD C5 BB 6B D7 24 BC 89 88 FC 40 01 00 6A 41 AB 2A 70 C2 96 D4 50 41 ED 03 43 41 41 F4 F4 C2 CC CA F8 BE 73 6A 19 BE F0 00 1C 3D 8D 37 5C 3F')
    example_data_hi92 = bytes.fromhex('5A A5 30 00 E3 A4 92 23 23 1C 7C 11 61 FF 23 23 00 00 01 00 00 00 EB FE 7B 00 B6 07 CD 03 70 FC A3 FC 56 1F 00 00 CF 0D 00 00 C4 07 00 00 F0 26 27 01 C0 02 C2 00 ')
    example_data_hi81 = bytes.fromhex('5A A5 68 00 70 04 81 00 02 01 15 09 43 C8 3E 02 00 00 83 00 3C 00 36 01 8F FB E8 04 31 04 7C 02 B2 F9 BF FE 60 79 00 10 24 18 07 1C 0A 1B EB 74 9A 0E 8A 0E 2E 18 20 20 3B 10 19 04 65 F1 99 6A 5D 45 31 61 CE 17 06 D1 00 00 0C 07 01 1C 00 00 00 B9 FC 01 00 00 02 00 00 00 C6 FF 08 01 E7 FF 7F 12 41 00 4E 61 CE 17 04 CF 00 00 00 00')
    

    print("Input binary data length for HI91:", len(example_data_hi91))
    frames_hi91 = decoder.parse(example_data_hi91)
    print(f"Total frames parsed for HI91: {len(frames_hi91)}")
    for i, frame in enumerate(frames_hi91):
        hipnuc_parser.print_parsed_data(frame)
    
    print("Input binary data length for HI92:", len(example_data_hi92))
    frames_hi92 = decoder.parse(example_data_hi92)
    print(f"Total frames parsed for HI91: {len(example_data_hi92)}")
    for i, frame in enumerate(frames_hi92):
        hipnuc_parser.print_parsed_data(frame)

    print("Input binary data length for HI81:", len(example_data_hi81))
    frames_hi81 = decoder.parse(example_data_hi81)
    print(f"Total frames parsed for HI81: {len(frames_hi81)}")
    for i, frame in enumerate(frames_hi81):
        hipnuc_parser.print_parsed_data(frame)


