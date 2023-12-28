import sys
import time
import serial
import os
import platform
from serial.tools import list_ports
from ch_serial import CHSerialDecoder

def clear_screen():
    """Clear the screen content"""
    os.system('cls' if os.name == 'nt' else 'clear')

def print_imu_data(imu_data):
    """Format and print IMU data"""
    print("Temperature  : {:>6} C".format(imu_data.temp))
    print("Pressure     : {:>9.3f} Pa".format(imu_data.prs))
    print("Acceleration : X={:>9.3f}, Y={:>9.3f}, Z={:>9.3f} m/s^2".format(imu_data.acc[0], imu_data.acc[1], imu_data.acc[2]))
    print("Gyroscope    : X={:>9.3f}, Y={:>9.3f}, Z={:>9.3f} dps".format(imu_data.gyr[0], imu_data.gyr[1], imu_data.gyr[2]))
    print("Magnetometer : X={:>9.3f}, Y={:>9.3f}, Z={:>9.3f} uT".format(imu_data.mag[0], imu_data.mag[1], imu_data.mag[2]))
    print("Euler Angles : Roll={:>9.3f}, Pitch={:>9.3f}, Yaw={:>9.3f} deg".format(imu_data.eul[0], imu_data.eul[1], imu_data.eul[2]))
    print("Quaternion   : W={:>9.3f}, X={:>9.3f}, Y={:>9.3f}, Z={:>9.3f}".format(imu_data.quat[0], imu_data.quat[1], imu_data.quat[2], imu_data.quat[3]))
    print("Timestamp    : {:>6} ms".format(imu_data.ts))

# Get the system platform (Linux or Windows)
system_platform = platform.system()

if len(sys.argv) == 1:
    print("Usage: python main.py <port> [baudrate]")
    print("Example: python main.py COM3 115200")
    sys.exit(1)

port = sys.argv[1]

if len(sys.argv) > 2:
    baudrate = sys.argv[2]
else:
    baudrate = 115200  # Default baud rate

try:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = int(baudrate)
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE

    decoder = CHSerialDecoder()

    max_update_rate = 1.0 / 25  # 25Hz
    last_update_time = 0

    while True:
        if not ser.is_open:
            ser.open()

        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            decoder.decode(data)

            current_time = time.time()
            if current_time - last_update_time >= max_update_rate:
                clear_screen()
                print_imu_data(decoder.imu_data)
                last_update_time = current_time

except KeyboardInterrupt:
    print("Program interrupted by user")
except serial.SerialException as e:
    print("Error: {e}")
    if system_platform == "Linux":
        # List available ports on Linux
        print("Available ports on Linux:")
        ports = list_ports.comports()
        for port in ports:
            print(port.device)
    elif system_platform == "Windows":
        # List available ports on Windows
        print("Available ports on Windows:")
        for port_info in list_ports.comports():
            print(port_info.device)
finally:
    if ser.is_open:
        ser.close()
        print("Serial port closed")
