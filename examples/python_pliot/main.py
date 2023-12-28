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
    print(f"Temperature  : {imu_data.temp:>6} °C")
    print(f"Pressure     : {imu_data.prs:>9.3f} Pa")
    print(f"Acceleration : X={imu_data.acc[0]:>9.3f}, Y={imu_data.acc[1]:>9.3f}, Z={imu_data.acc[2]:>9.3f} m/s^2")
    print(f"Gyroscope    : X={imu_data.gyr[0]:>9.3f}, Y={imu_data.gyr[1]:>9.3f}, Z={imu_data.gyr[2]:>9.3f} °/s")
    print(f"Magnetometer : X={imu_data.mag[0]:>9.3f}, Y={imu_data.mag[1]:>9.3f}, Z={imu_data.mag[2]:>9.3f} μT")
    print(f"Euler Angles : Roll={imu_data.eul[0]:>9.3f}, Pitch={imu_data.eul[1]:>9.3f}, Yaw={imu_data.eul[2]:>9.3f} °")
    print(f"Quaternion   : W={imu_data.quat[0]:>9.3f}, X={imu_data.quat[1]:>9.3f}, Y={imu_data.quat[2]:>9.3f}, Z={imu_data.quat[3]:>9.3f}")
    print(f"Timestamp    : {imu_data.ts:>6} ms")

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
    print(f"Error: {e}")
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
