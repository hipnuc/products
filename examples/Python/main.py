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

def configure_serial(port, baudrate):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = int(baudrate)
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    return ser

def print_imu_data(imu_data):
    """Format and print IMU data"""
    print("Temperature  : {:>6} C".format(imu_data.temp))
    print("Pressure     : {:>9.3f} Pa".format(imu_data.prs))
    print("Acceleration : X={:>9.3f}, Y={:>9.3f}, Z={:>9.3f} G".format(imu_data.acc[0], imu_data.acc[1], imu_data.acc[2]))
    print("Gyroscope    : X={:>9.3f}, Y={:>9.3f}, Z={:>9.3f} dps".format(imu_data.gyr[0], imu_data.gyr[1], imu_data.gyr[2]))
    print("Magnetometer : X={:>9.3f}, Y={:>9.3f}, Z={:>9.3f} uT".format(imu_data.mag[0], imu_data.mag[1], imu_data.mag[2]))
    print("Euler Angles : Roll={:>9.3f}, Pitch={:>9.3f}, Yaw={:>9.3f} deg".format(imu_data.eul[0], imu_data.eul[1], imu_data.eul[2]))
    print("Quaternion   : W={:>9.3f}, X={:>9.3f}, Y={:>9.3f}, Z={:>9.3f}".format(imu_data.quat[0], imu_data.quat[1], imu_data.quat[2], imu_data.quat[3]))
    print("Timestamp    : {:>6} ms".format(imu_data.ts))


def list_available_ports():
    print("Available ports:")
    ports = list_ports.comports()
    for port in ports:
        print(port.device)

def print_usage_and_exit():
    """Print usage and exit the program"""
    system_platform = platform.system()
    print("Usage: python main.py <port> [<baudrate>] | list")
    if system_platform == "Linux":
        print("Examples:")
        print("    Connect to a specific port:")
        print("    python main.py /dev/ttyUSB0 115200")
        print("    List available ports:")
        print("    python main.py list")
    elif system_platform == "Windows":
        print("Examples:")
        print("    Connect to a specific port:")
        print("    python main.py COM3 115200")
        print("    List available ports:")
        print("    python main.py list")
    sys.exit(1)

def main():
    system_platform = platform.system()

    if len(sys.argv) == 1:
        print_usage_and_exit()
    elif len(sys.argv) == 2 and sys.argv[1] == 'main.py':
        print_usage_and_exit()

    if len(sys.argv) == 2 and sys.argv[1] == 'list':
        list_available_ports()
        sys.exit(0)

    port = sys.argv[1]
    baudrate = sys.argv[2] if len(sys.argv) > 2 else '115200'  # Default baud rate as a string

    if not baudrate.isdigit() or int(baudrate) <= 0:
        print("Invalid baudrate.")
        baudrate = '115200'  # Default baud rate as a string
        print_usage_and_exit()  # Exit if baudrate is invalid

    ser = configure_serial(port, baudrate)
    decoder = CHSerialDecoder()

    max_update_rate = 1.0 / 25
    last_update_time = 0

    try:
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
        if "Permission" in str(e):
            print("Error: Insufficient permissions to access the serial port.")
            print("To run this script with superuser privileges, use the 'sudo' command:\n"
                  "Example: sudo python main.py {} {}".format(port, baudrate))
        else:
            print("Serial Error: {}".format(e))

            list_available_ports()
    except PermissionError as e:
        print("Error: {}".format(e))
        print("To run this script with superuser privileges, use the 'sudo' command:\n"
              "    Example: sudo python main.py {} {}".format(port, baudrate))
        sys.exit(1)
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()
