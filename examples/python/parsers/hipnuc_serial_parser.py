import struct
import logging
import json

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

CHSYNC1 = 0x5A
CHSYNC2 = 0xA5
CH_HDR_SIZE = 6

GRAVITY = 9.80665
R2D = 57.29577951308232

FRAME_TAG_HI91 = 0x91
FRAME_TAG_HI81 = 0x81
FRAME_TAG_HI83 = 0x83

HI83_BMAP_ACC_B = 1 << 0
HI83_BMAP_GYR_B = 1 << 1
HI83_BMAP_MAG_B = 1 << 2
HI83_BMAP_RPY = 1 << 3
HI83_BMAP_QUAT = 1 << 4
HI83_BMAP_SYSTEM_TIME = 1 << 5
HI83_BMAP_UTC = 1 << 6
HI83_BMAP_AIR_PRESSURE = 1 << 7
HI83_BMAP_TEMPERATURE = 1 << 8
HI83_BMAP_INCLINATION = 1 << 9
HI83_BMAP_HSS = 1 << 10
HI83_BMAP_HSS_FRQ = 1 << 11
HI83_BMAP_VEL_ENU = 1 << 12
HI83_BMAP_ACC_ENU = 1 << 13
HI83_BMAP_INS_LON_LAT_MSL = 1 << 14
HI83_BMAP_GNSS_QUALITY_NV = 1 << 15
HI83_BMAP_OD_SPEED = 1 << 16
HI83_BMAP_UNDULATION = 1 << 17
HI83_BMAP_DIFF_AGE = 1 << 18
HI83_BMAP_NODE_ID = 1 << 19
HI83_BMAP_GNSS_LON_LAT_MSL = 1 << 30
HI83_BMAP_GNSS_VEL = 1 << 31

class hipnuc_parser:
    def __init__(self):
        self.CHSYNC1 = CHSYNC1
        self.CHSYNC2 = CHSYNC2
        self.CH_HDR_SIZE = CH_HDR_SIZE
        self.buffer = bytearray()

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

    def _parse_hi91(self, data, ofs):
        return {
            "type": "HI91",
            "main_status": struct.unpack_from('<H', data, ofs + 1)[0],
            "temperature": struct.unpack_from('<b', data, ofs + 3)[0],
            "air_pressure": struct.unpack_from('<f', data, ofs + 4)[0],
            "system_time": struct.unpack_from('<I', data, ofs + 8)[0],
            "acc": [x * GRAVITY for x in struct.unpack_from('<3f', data, ofs + 12)],
            "gyr": list(struct.unpack_from('<3f', data, ofs + 24)),
            "mag": list(struct.unpack_from('<3f', data, ofs + 36)),
            "roll": struct.unpack_from('<f', data, ofs + 48)[0],
            "pitch": struct.unpack_from('<f', data, ofs + 52)[0],
            "yaw": struct.unpack_from('<f', data, ofs + 56)[0],
            "quat": list(struct.unpack_from('<4f', data, ofs + 60)),
        }, ofs + 76

    def _parse_hi81(self, data, ofs):
        utc_year = struct.unpack_from('<B', data, ofs + 35)[0]
        utc_month = struct.unpack_from('<B', data, ofs + 36)[0]
        utc_day = struct.unpack_from('<B', data, ofs + 37)[0]
        utc_hour = struct.unpack_from('<B', data, ofs + 38)[0]
        utc_min = struct.unpack_from('<B', data, ofs + 39)[0]
        utc_msec = struct.unpack_from('<H', data, ofs + 40)[0]
        utc_sec = utc_msec // 1000
        utc_ms = utc_msec % 1000
        utc = f"20{utc_year:02}-{utc_month:02}-{utc_day:02} {utc_hour:02}:{utc_min:02}:{utc_sec:02}.{utc_ms:03}"

        return {
            "type": "HI81",
            "main_status": struct.unpack_from('<H', data, ofs + 1)[0],
            "ins_status": struct.unpack_from('<B', data, ofs + 3)[0],
            "gpst_wn": struct.unpack_from('<H', data, ofs + 4)[0],
            "gpst_tow": struct.unpack_from('<I', data, ofs + 6)[0],
            "gyr": [x * 0.001 * R2D for x in struct.unpack_from('<3h', data, ofs + 12)],
            "acc": [x * 0.0048828 for x in struct.unpack_from('<3h', data, ofs + 18)],
            "mag": [x * 0.030517 for x in struct.unpack_from('<3h', data, ofs + 24)],
            "air_pressure": struct.unpack_from('<h', data, ofs + 30)[0],
            "temperature": struct.unpack_from('<b', data, ofs + 34)[0],
            "utc": utc,
            "roll": struct.unpack_from('<h', data, ofs + 42)[0] * 0.01,
            "pitch": struct.unpack_from('<h', data, ofs + 44)[0] * 0.01,
            "yaw": struct.unpack_from('<H', data, ofs + 46)[0] * 0.01,
            "quat": [x * 0.0001 for x in struct.unpack_from('<4h', data, ofs + 48)],
            "ins_lon": struct.unpack_from('<i', data, ofs + 56)[0] * 1e-7,
            "ins_lat": struct.unpack_from('<i', data, ofs + 60)[0] * 1e-7,
            "ins_msl": struct.unpack_from('<i', data, ofs + 64)[0] * 1e-3,
            "pdop": struct.unpack_from('<B', data, ofs + 68)[0] * 0.1,
            "hdop": struct.unpack_from('<B', data, ofs + 69)[0] * 0.1,
            "solq_pos": struct.unpack_from('<B', data, ofs + 70)[0],
            "nv_pos": struct.unpack_from('<B', data, ofs + 71)[0],
            "solq_heading": struct.unpack_from('<B', data, ofs + 72)[0],
            "nv_heading": struct.unpack_from('<B', data, ofs + 73)[0],
            "diff_age": struct.unpack_from('<B', data, ofs + 74)[0],
            "undulation": struct.unpack_from('<h', data, ofs + 75)[0] * 0.01,
            "ant_status": struct.unpack_from('<B', data, ofs + 77)[0],
            "vel_enu": [x * 0.01 for x in struct.unpack_from('<3h', data, ofs + 78)],
            "acc_enu": [x * 0.0048828 for x in struct.unpack_from('<3h', data, ofs + 84)],
            "gnss_lon": struct.unpack_from('<i', data, ofs + 90)[0] * 1e-7,
            "gnss_lat": struct.unpack_from('<i', data, ofs + 94)[0] * 1e-7,
            "gnss_msl": struct.unpack_from('<i', data, ofs + 98)[0] * 1e-3,
        }, ofs + 104

    def _parse_hi83(self, data, ofs):
        bitmap = struct.unpack_from('<I', data, ofs + 4)[0]
        idx = ofs + 8

        result = {
            "type": "HI83",
            "main_status": struct.unpack_from('<H', data, ofs + 1)[0],
            "ins_status": struct.unpack_from('<B', data, ofs + 3)[0],
            "data_bitmap": bitmap,
        }

        if bitmap & HI83_BMAP_ACC_B:
            result["acc"] = list(struct.unpack_from('<3f', data, idx))
            idx += 12
        if bitmap & HI83_BMAP_GYR_B:
            result["gyr"] = list(struct.unpack_from('<3f', data, idx))
            idx += 12
        if bitmap & HI83_BMAP_MAG_B:
            result["mag"] = list(struct.unpack_from('<3f', data, idx))
            idx += 12
        if bitmap & HI83_BMAP_RPY:
            rpy = struct.unpack_from('<3f', data, idx)
            result["roll"] = rpy[0]
            result["pitch"] = rpy[1]
            result["yaw"] = rpy[2]
            idx += 12
        if bitmap & HI83_BMAP_QUAT:
            result["quat"] = list(struct.unpack_from('<4f', data, idx))
            idx += 16
        if bitmap & HI83_BMAP_SYSTEM_TIME:
            result["system_time_us"] = struct.unpack_from('<Q', data, idx)[0]
            idx += 8
        if bitmap & HI83_BMAP_UTC:
            year = data[idx]
            month = data[idx + 1]
            day = data[idx + 2]
            hour = data[idx + 3]
            minute = data[idx + 4]
            sec_ms = struct.unpack_from('<H', data, idx + 5)[0]
            rev = data[idx + 7]
            utc_sec = sec_ms // 1000
            utc_ms = sec_ms % 1000
            result["utc"] = f"20{year:02}-{month:02}-{day:02} {hour:02}:{minute:02}:{utc_sec:02}.{utc_ms:03}"
            result["utc_rev"] = rev
            idx += 8
        if bitmap & HI83_BMAP_AIR_PRESSURE:
            result["air_pressure"] = struct.unpack_from('<f', data, idx)[0]
            idx += 4
        if bitmap & HI83_BMAP_TEMPERATURE:
            result["temperature"] = struct.unpack_from('<f', data, idx)[0]
            idx += 4
        if bitmap & HI83_BMAP_INCLINATION:
            result["inclination"] = list(struct.unpack_from('<3f', data, idx))
            idx += 12
        if bitmap & HI83_BMAP_HSS:
            result["hss"] = list(struct.unpack_from('<3f', data, idx))
            idx += 12
        if bitmap & HI83_BMAP_HSS_FRQ:
            result["hss_frq"] = list(struct.unpack_from('<3f', data, idx))
            idx += 12
        if bitmap & HI83_BMAP_VEL_ENU:
            result["vel_enu"] = list(struct.unpack_from('<3f', data, idx))
            idx += 12
        if bitmap & HI83_BMAP_ACC_ENU:
            result["acc_enu"] = list(struct.unpack_from('<3f', data, idx))
            idx += 12
        if bitmap & HI83_BMAP_INS_LON_LAT_MSL:
            result["ins_lon_lat_msl"] = list(struct.unpack_from('<3d', data, idx))
            idx += 24
        if bitmap & HI83_BMAP_GNSS_QUALITY_NV:
            solq_pos, nv_pos, solq_heading, nv_heading = struct.unpack_from('<4B', data, idx)
            result["solq_pos"] = solq_pos
            result["nv_pos"] = nv_pos
            result["solq_heading"] = solq_heading
            result["nv_heading"] = nv_heading
            idx += 4
        if bitmap & HI83_BMAP_OD_SPEED:
            result["od_speed"] = struct.unpack_from('<f', data, idx)[0]
            idx += 4
        if bitmap & HI83_BMAP_UNDULATION:
            result["undulation"] = struct.unpack_from('<f', data, idx)[0]
            idx += 4
        if bitmap & HI83_BMAP_DIFF_AGE:
            result["diff_age"] = struct.unpack_from('<f', data, idx)[0]
            idx += 4
        if bitmap & HI83_BMAP_NODE_ID:
            result["node_id"] = struct.unpack_from('<B', data, idx)[0]
            idx += 4
        if bitmap & HI83_BMAP_GNSS_LON_LAT_MSL:
            result["gnss_lon_lat_msl"] = list(struct.unpack_from('<3d', data, idx))
            idx += 24
        if bitmap & HI83_BMAP_GNSS_VEL:
            result["gnss_vel"] = list(struct.unpack_from('<3f', data, idx))
            idx += 12

        return result, idx

    def parse_data(self, data):
        packets = []
        ofs = 0
        while ofs < len(data):
            item_type = data[ofs]
            try:
                if item_type == FRAME_TAG_HI91:
                    packet, ofs = self._parse_hi91(data, ofs)
                    packets.append(packet)
                elif item_type == FRAME_TAG_HI81:
                    packet, ofs = self._parse_hi81(data, ofs)
                    packets.append(packet)
                elif item_type == FRAME_TAG_HI83:
                    packet, ofs = self._parse_hi83(data, ofs)
                    packets.append(packet)
                else:
                    logging.warning(f"Unknown item type: {item_type}")
                    ofs += 1
            except struct.error as e:
                logging.error(f"Error parsing data: {e}")
                ofs += 1
        return packets

    def parse(self, new_data):
        self.buffer += new_data
        packets = []
        while len(self.buffer) >= self.CH_HDR_SIZE:
            if self.buffer[0] == self.CHSYNC1 and self.buffer[1] == self.CHSYNC2:
                length = struct.unpack_from('<H', self.buffer, 2)[0]
                if len(self.buffer) >= self.CH_HDR_SIZE + length:
                    frame = self.buffer[:self.CH_HDR_SIZE + length]
                    crc_calculated = self.crc16_update(0, frame[:4] + frame[6:])
                    crc_received = struct.unpack_from('<H', frame, 4)[0]
                    if crc_calculated == crc_received:
                        packets.extend(self.parse_data(frame[self.CH_HDR_SIZE:]))
                    else:
                        logging.error("CRC check failed")
                    del self.buffer[:self.CH_HDR_SIZE + length]
                else:
                    break
            else:
                del self.buffer[0]
        return packets


    @staticmethod
    def print_parsed_data(data):
        if not data:
            return
        print(json.dumps(data, ensure_ascii=False))





# Example usage
if __name__ == "__main__":
    decoder = hipnuc_parser()
    
    example_data_hi91 = bytes.fromhex('5A A5 4C 00 14 BB 91 08 15 23 09 A2 C4 47 08 15 1C 00 CC E8 61 BE 9A 35 56 3E 65 EA 72 3F 31 D0 7C BD 75 DD C5 BB 6B D7 24 BC 89 88 FC 40 01 00 6A 41 AB 2A 70 C2 96 D4 50 41 ED 03 43 41 41 F4 F4 C2 CC CA F8 BE 73 6A 19 BE F0 00 1C 3D 8D 37 5C 3F')
    example_data_hi81 = bytes.fromhex('5A A5 68 00 70 04 81 00 02 01 15 09 43 C8 3E 02 00 00 83 00 3C 00 36 01 8F FB E8 04 31 04 7C 02 B2 F9 BF FE 60 79 00 10 24 18 07 1C 0A 1B EB 74 9A 0E 8A 0E 2E 18 20 20 3B 10 19 04 65 F1 99 6A 5D 45 31 61 CE 17 06 D1 00 00 0C 07 01 1C 00 00 00 B9 FC 01 00 00 02 00 00 00 C6 FF 08 01 E7 FF 7F 12 41 00 4E 61 CE 17 04 CF 00 00 00 00')
    

    print("Input binary data length for HI91:", len(example_data_hi91))
    packets_hi91 = decoder.parse(example_data_hi91)
    print(f"Total packets parsed for HI91: {len(packets_hi91)}")
    for packet in packets_hi91:
        hipnuc_parser.print_parsed_data(packet)
    

    print("Input binary data length for HI81:", len(example_data_hi81))
    packets_hi81 = decoder.parse(example_data_hi81)
    print(f"Total packets parsed for HI81: {len(packets_hi81)}")
    for packet in packets_hi81:
        hipnuc_parser.print_parsed_data(packet)
