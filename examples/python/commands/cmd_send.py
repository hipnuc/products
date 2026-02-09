import sys
import os
import time
import serial
import click
from utils import configure_serial, send_command_and_wait_for_response

def _send_commands_from_file(ser, filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith("#") or stripped.startswith(";"):
                continue
            print(f"Send command: {stripped}")
            success, response = send_command_and_wait_for_response(ser, stripped, ignore_non_text=False)
            if response:
                print(response)
            if not success:
                return False
    return True

@click.command(name='write', short_help='Send command(s) to the module')
@click.option('--port', '-p', required=True, help="The serial port to connect to (e.g., COM3 or /dev/ttyUSB0)")
@click.option('--baudrate', '-b', default='115200', help="The baud rate for the serial connection (default: 115200)")
@click.argument('args', nargs=-1, required=True)
def cmd_write(port, baudrate, args):
    if not baudrate.isdigit() or int(baudrate) <= 0:
        raise click.BadParameter("Invalid baudrate. Baudrate must be a positive integer.")

    ser = configure_serial(port, baudrate)

    try:
        if not ser.is_open:
            ser.open()

        max_retries = 3
        for attempt in range(max_retries):
            success, response = send_command_and_wait_for_response(ser, "LOG DISABLE", ignore_non_text=True)
            if success:
                break
            if attempt < max_retries - 1:
                time.sleep(0.2)
        else:
            print("Error: Failed to send command 'LOG DISABLE' after multiple attempts.")
            return

        target = " ".join(args).strip()
        if len(args) == 1 and os.path.isfile(target):
            success = _send_commands_from_file(ser, target)
            if not success:
                print("Error: Command file execution failed.")
                return
        else:
            success, response = send_command_and_wait_for_response(ser, target, ignore_non_text=False)
            if response:
                print(response)
            if not success:
                print(f"Error: Failed to send command '{target}'")
                return

        send_command_and_wait_for_response(ser, "LOG ENABLE", ignore_non_text=True)
        
    except serial.SerialException as e:
        if "Permission" in str(e):
            print("Error: Insufficient permissions to access the serial port.")
            cmd_str = " ".join(args)
            print(f"To run this script with superuser privileges, use the 'sudo' command:\nExample: sudo python main.py write --port {port} --baudrate {baudrate} \"{cmd_str}\"")
        else:
            print(f"Serial Error: {e}")
    except PermissionError as e:
        print(f"Error: {e}")
        cmd_str = " ".join(args)
        print(f"To run this script with superuser privileges, use the 'sudo' command:\nExample: sudo python main.py write --port {port} --baudrate {baudrate} \"{cmd_str}\"")
        sys.exit(1)

if __name__ == "__main__":
    cmd_write()
