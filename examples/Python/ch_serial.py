import struct

# Frame header and footer constants
CHSYNC1 = 0x5A
CHSYNC2 = 0xA5
CH_HDR_SIZE = 6
MAXRAWLEN = 512

# Data item identifiers
K_ITEM_ID = 0x90
K_ITEM_ACC_RAW = 0xA0
K_ITEM_GYR_RAW = 0xB0
K_ITEM_MAG_RAW = 0xC0
K_ITEM_ROTATION_EUL = 0xD0
K_ITEM_ROTATION_QUAT = 0xD1
K_ITEM_PRESSURE = 0xF0
K_ITEM_IMUSOL = 0x91

class IMUData:
    def __init__(self):
        self.temp = 0
        self.prs = 0
        self.ts = 0
        self.acc = [0, 0, 0]
        self.gyr = [0, 0, 0]
        self.mag = [0, 0, 0]
        self.eul = [0, 0, 0]
        self.quat = [0, 0, 0, 0]

class CHSerialDecoder:
    def __init__(self):
        self.CHSYNC1 = CHSYNC1
        self.CHSYNC2 = CHSYNC2
        self.CH_HDR_SIZE = CH_HDR_SIZE
        self.buffer = bytearray()
        self.imu_data = IMUData()

    def crc16_update(self, crc, data):
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                temp = crc << 1
                if crc & 0x8000:
                    temp ^= 0x1021
                crc = temp
        return crc & 0xFFFF

    def parse_data(self, data):
        ofs = 0
        while ofs < len(data):
            item_type = data[ofs]
            if item_type == K_ITEM_ID:
                self.imu_data.rev = data[ofs + 1]
                ofs += 2
            elif item_type == K_ITEM_ACC_RAW:
                self.imu_data.acc = [
                    struct.unpack('<h', data[ofs + 1:ofs + 3])[0] / 1000.0,
                    struct.unpack('<h', data[ofs + 3:ofs + 5])[0] / 1000.0,
                    struct.unpack('<h', data[ofs + 5:ofs + 7])[0] / 1000.0
                ]
                ofs += 7
            elif item_type == K_ITEM_GYR_RAW:
                self.imu_data.gyr = [
                    struct.unpack('<h', data[ofs + 1:ofs + 3])[0] / 10.0,
                    struct.unpack('<h', data[ofs + 3:ofs + 5])[0] / 10.0,
                    struct.unpack('<h', data[ofs + 5:ofs + 7])[0] / 10.0
                ]
                ofs += 7
            elif item_type == K_ITEM_MAG_RAW:
                self.imu_data.mag = [
                    struct.unpack('<h', data[ofs + 1:ofs + 3])[0] / 10.0,
                    struct.unpack('<h', data[ofs + 3:ofs + 5])[0] / 10.0,
                    struct.unpack('<h', data[ofs + 5:ofs + 7])[0] / 10.0
                ]
                ofs += 7
            elif item_type == K_ITEM_ROTATION_EUL:
                self.imu_data.eul = [
                    struct.unpack('<h', data[ofs + 1:ofs + 3])[0] / 100.0,
                    struct.unpack('<h', data[ofs + 3:ofs + 5])[0] / 100.0,
                    struct.unpack('<h', data[ofs + 5:ofs + 7])[0] / 10.0
                ]
                ofs += 7
            elif item_type == K_ITEM_ROTATION_QUAT:
                self.imu_data.quat = [
                    struct.unpack('<f', data[ofs + 1:ofs + 5])[0],
                    struct.unpack('<f', data[ofs + 5:ofs + 9])[0],
                    struct.unpack('<f', data[ofs + 9:ofs + 13])[0],
                    struct.unpack('<f', data[ofs + 13:ofs + 17])[0]
                ]
                ofs += 17
            elif item_type == K_ITEM_PRESSURE:
                self.imu_data.prs = struct.unpack('<f', data[ofs + 1:ofs + 5])[0]
                ofs += 5
            elif item_type == K_ITEM_IMUSOL:
                self.imu_data.rev = data[ofs + 1]
                self.imu_data.temp = data[ofs + 3]
                self.imu_data.prs = struct.unpack('<f', data[ofs + 4:ofs + 8])[0]
                self.imu_data.ts = struct.unpack('<I', data[ofs + 8:ofs + 12])[0]
                self.imu_data.acc = [
                    struct.unpack('<f', data[ofs + 12:ofs + 16])[0],
                    struct.unpack('<f', data[ofs + 16:ofs + 20])[0],
                    struct.unpack('<f', data[ofs + 20:ofs + 24])[0]
                ]
                self.imu_data.gyr = [
                    struct.unpack('<f', data[ofs + 24:ofs + 28])[0],
                    struct.unpack('<f', data[ofs + 28:ofs + 32])[0],
                    struct.unpack('<f', data[ofs + 32:ofs + 36])[0]
                ]
                self.imu_data.mag = [
                    struct.unpack('<f', data[ofs + 36:ofs + 40])[0],
                    struct.unpack('<f', data[ofs + 40:ofs + 44])[0],
                    struct.unpack('<f', data[ofs + 44:ofs + 48])[0]
                ]
                self.imu_data.eul = [
                    struct.unpack('<f', data[ofs + 48:ofs + 52])[0],
                    struct.unpack('<f', data[ofs + 52:ofs + 56])[0],
                    struct.unpack('<f', data[ofs + 56:ofs + 60])[0]
                ]
                self.imu_data.quat = [
                    struct.unpack('<f', data[ofs + 60:ofs + 64])[0],
                    struct.unpack('<f', data[ofs + 64:ofs + 68])[0],
                    struct.unpack('<f', data[ofs + 68:ofs + 72])[0],
                    struct.unpack('<f', data[ofs + 72:ofs + 76])[0]
                ]
                ofs += 76
            else:
                ofs += 1

    def decode(self, new_data):
        self.buffer += new_data
        while len(self.buffer) >= self.CH_HDR_SIZE:
            if self.buffer[0] == self.CHSYNC1 and self.buffer[1] == self.CHSYNC2:
                length = struct.unpack_from('<H', self.buffer, 2)[0]
                if len(self.buffer) >= self.CH_HDR_SIZE + length:
                    frame = self.buffer[:self.CH_HDR_SIZE + length]
                    crc_calculated = self.crc16_update(0, frame[:4] + frame[6:])
                    crc_received = struct.unpack_from('<H', frame, 4)[0]
                    if crc_calculated == crc_received:
                        self.parse_data(frame[self.CH_HDR_SIZE:])
                    del self.buffer[:self.CH_HDR_SIZE + length]
                else:
                    break
            else:
                del self.buffer[0]
