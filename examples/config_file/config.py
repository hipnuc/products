import serial
import time
import argparse
import os
import yaml
import select
            

def read_all_data(ser, timeout = 0.1):
    end_time = time.time() + timeout
    buffer = ""

    while time.time() < end_time:
        while ser.in_waiting > 0:
            data = ser.readline(ser.in_waiting).decode('latin1')
            buffer += data
        time.sleep(0.1)
    return buffer

def send_command(ser, command, expected_response, retries=5):
    while retries > 0:

        if not command.endswith('\r\n'):
            command += '\r\n'

        ser.write(command.encode())

        response = read_all_data(ser)
   
        if response.strip().split('\n')[-1].lower() == expected_response.lower():
            print(f"scmd:    {command}")
            print(f"rev :    {response}")
            return True
        retries -= 1
        print(f"Retrying ...{retries} attempts left")
    
    return False

def auto_detect_baudrate(port):
    common_baudrate = [9600, 115200, 230400, 256000, 460800, 921600]
    for baudrate in common_baudrate:
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(0.1)
            response = send_command(ser,'AT+EOUT=0', 'OK')
            ser.close()
            if response is True:
                return baudrate
        except(serial.SerialException, UnicodeDecodeError):
            continue
    return None


def config_module(port, baudrate, config_file):
    if not os.path.exists(config_file):
        print(f"Configuration file '{config_file}' does not exist, Exiting program")
        return

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(0.1)
        
        if not send_command(ser, 'AT+EOUT=0', 'ok'):
            print("Failed to reveive 'ok' for AT+EOUT=0")
            ser.close()
            return 

        if not send_command(ser, 'AT+INFO', 'ok'):
            print("Failed to receive 'ok' for AT+INFO")
            ser.close()
            return

        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
            commands = config.get('commands', [])
            for command in commands:
                if not send_command(ser, command, 'ok'):
                    print(f"Faild to receive 'ok' for command: {command}")
                    ser.close()
                    return

        command = 'REBOOT\r\n'
        ser.write(command.encode())
        time.sleep(1)

        ser.close()

    except serial.SerialException as e:
        print(f"error:{e}")
    except UnicodeDecodeError as e:
        print(f"Decoding Error:{e}")

def read_imu_info(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout = 1)
        time.sleep(2)

        if not send_command(ser, 'AT+INFO', 'ok'):
            print("Failed to receive 'ok' for AT+INFO")
            ser.close()
            return

    except serial.SerialException as e:
        print(f"error:{e}")


if __name__=="__main__":

    parser = argparse.ArgumentParser(description="Serial Port Communication")
    parser.add_argument("--port", type=str, required=True, help="Serial port (e.g., /dev/ttyUSB0 or COM3)")
    parser.add_argument("--config", type=str, required=True, help="Patn to the YAML configuration file")
    args = parser.parse_args()

    port = args.port
    baudrate = auto_detect_baudrate(port)
    if baudrate is None:
        print("Faild to detect baudrate, Exiting program.")
        exit()

    config_file = args.config
    config_module(port, baudrate, config_file)

    print("Reconnect the IMU and check the new configuration!")
    print("Reconnect the IMU and check the new configuration!")

    baudrate = auto_detect_baudrate(port)
    if baudrate is None:
        print("Faild to detect baudrate, Exiting program.")
    else:
        print(f"new baud rate: {baudrate}")

    read_imu_info(port, baudrate)

