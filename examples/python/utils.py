import sys
import os
import serial

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
