import sys
import time
import serial
import os
import argparse
from serial.tools import list_ports
from hipnuc_serial_parser import hipnuc_parser


def check_python_version():
    """Check if the Python version is at least 3.6"""
    print("Your Python version is {}.{}.{}.".format(sys.version_info.major, sys.version_info.minor, sys.version_info.micro))
    if sys.version_info < (3, 6):
        print("This script requires Python 3.6 or higher. Please update your Python version.")
        sys.exit(1)

def clear_screen():
    """Clear the screen content"""
    os.system('cls' if os.name == 'nt' else 'clear')

def configure_serial(port, baudrate):
    """Configure the serial port with given parameters"""
    try:
        ser = serial.Serial(port=port, baudrate=int(baudrate), bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        print(f"Serial port {ser.name} opened with baud rate {baudrate}")
        return ser
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port}. {e}")
        sys.exit(1)

def list_available_ports():
    """List all available serial ports with manufacturer information"""
    print("Available ports:")
    for port in list_ports.comports():
        print("Device: {:<20} Manufacturer: {}".format(port.device, port.manufacturer or 'Unknown'))

def main():
    """Main function to run the serial reader"""
    check_python_version()

    parser = argparse.ArgumentParser(description="HiPNUC Python Example")
    parser.add_argument('-l', '--list-ports', action='store_true', help="List all available serial ports and exit")
    parser.add_argument('-p', '--port', help="The serial port to connect to (e.g., COM3 or /dev/ttyUSB0)", required=False)
    parser.add_argument('-b', '--baudrate', default='115200', help="The baud rate for the serial connection (default: 115200)")
    args = parser.parse_args()

    if args.list_ports:
        list_available_ports()
        sys.exit(0)

    if not args.port or not args.baudrate:
        parser.print_help()
        sys.exit(1)

    port = args.port
    baudrate = args.baudrate

    if not baudrate.isdigit() or int(baudrate) <= 0:
        parser.error("Invalid baudrate. Baudrate must be a positive integer.")

    ser = configure_serial(port, baudrate)
    parser = hipnuc_parser()
    decode_frequency = 5  # Call decode function 5 times per second
    decode_interval = 1.0 / decode_frequency
    last_decode_time = 0
    frame_count = 0
    frame_rate = 0
    last_count_time = time.time()

    try:
        while True:
            if not ser.is_open:
                ser.open()

            current_time = time.time()
            if current_time - last_decode_time >= decode_interval:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    frames = parser.decode(data)
                    frame_count += len(frames)
                    clear_screen()
                    print(f"frame decoded count: {len(frames)}, frame_rate: {frame_rate}Hz")
                    
                    if frames:  # Check if the list is empty
                        parser.print_parsed_data(frames[-1])  # Only print the last frame data parsed
                    last_decode_time = current_time

            # Count frames once per second
            if current_time - last_count_time >= 1.0:
                frame_rate = frame_count
                frame_count = 0
                last_count_time = current_time

    except KeyboardInterrupt:
        clear_screen()
        print("Program interrupted by user")
    except serial.SerialException as e:
        if "Permission" in str(e):
            print("Error: Insufficient permissions to access the serial port.")
            print(f"To run this script with superuser privileges, use the 'sudo' command:\nExample: sudo python main.py -p {port} -b {baudrate}")
        else:
            print(f"Serial Error: {e}")
    except PermissionError as e:
        print(f"Error: {e}")
        print(f"To run this script with superuser privileges, use the 'sudo' command:\nExample: sudo python main.py -p {port} -b {baudrate}")
        sys.exit(1)
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()
