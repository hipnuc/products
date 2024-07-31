import sys
import time
import serial
import click
from utils import configure_serial

def send_command_and_wait_for_response(ser, command_str, timeout=0.2, ignore_non_text=False):
    """
    Send command and wait for response.

    """
    if not command_str.endswith("\r\n"):
        command_str += "\r\n"

    ser.write(command_str.encode())
    time.sleep(timeout)

    response = b''
    while ser.in_waiting > 0:
        response += ser.read(ser.in_waiting)

    if ignore_non_text:
        # Ignore non-text data
        response = b''.join([bytes([b]) for b in response if 32 <= b <= 126 or b in (10, 13)])

    try:
        decoded_response = response.decode().strip()
        return "OK" in decoded_response, decoded_response
    except UnicodeDecodeError:
        return False, "Error: Unable to decode response."

@click.command(name='send', short_help='Send a command to the module')
@click.option('--port', '-p', required=True, help="The serial port to connect to (e.g., COM3 or /dev/ttyUSB0)")
@click.option('--baudrate', '-b', default='115200', help="The baud rate for the serial connection (default: 115200)")
@click.argument('command', type=str, required=True)  # Capture the command as a single string
def cmd_send(port, baudrate, command):
    """Send a command to the module"""
    if not baudrate.isdigit() or int(baudrate) <= 0:
        raise click.BadParameter("Invalid baudrate. Baudrate must be a positive integer.")

    ser = configure_serial(port, baudrate)

    try:
        if not ser.is_open:
            ser.open()

        # Send AT+EOUT=0 and ignore non-text data in the response, with retries
        max_retries = 3
        for attempt in range(max_retries):
            success, response = send_command_and_wait_for_response(ser, "AT+EOUT=0", ignore_non_text=True)
            if success:
                break
            if attempt < max_retries - 1:
                time.sleep(0.2)  # Wait before retrying
        else:
            print("Error: Failed to send command 'AT+EOUT=0' after multiple attempts.")
            return

        # Send command and display response
        success, response = send_command_and_wait_for_response(ser, command, ignore_non_text=False)
        if not success:
            print(f"Error: Failed to send command '{command}'")
            return
        print(response)

        success, response = send_command_and_wait_for_response(ser, "SAVECONFIG", ignore_non_text=True)
        if not success:
            print("Error: Failed to send command 'SAVECONFIG'")
            return

        success, response = send_command_and_wait_for_response(ser, "AT+EOUT=1", ignore_non_text=True)
        if not success:
            print("Error: Failed to send command 'AT+EOUT=1'")
            return
        
    except serial.SerialException as e:
        if "Permission" in str(e):
            print("Error: Insufficient permissions to access the serial port.")
            print(f"To run this script with superuser privileges, use the 'sudo' command:\nExample: sudo python main.py send-command --port {port} --baudrate {baudrate} \"{command}\"")
        else:
            print(f"Serial Error: {e}")
    except PermissionError as e:
        print(f"Error: {e}")
        print(f"To run this script with superuser privileges, use the 'sudo' command:\nExample: sudo python main.py send-command --port {port} --baudrate {baudrate} \"{command}\"")
        sys.exit(1)

if __name__ == "__main__":
    send_command()
