import os
import time
import threading
import click
import serial
from serial.tools import list_ports
from utils import send_command_and_wait_for_response

@click.command(name='list', short_help='List all available serial ports')
@click.option('--details', is_flag=True, default=False, help="Show extra port details")
def cmd_list(details):
    """
    List all available serial ports with manufacturer information.

    Usage:
        python main.py list-ports

    This command lists all the available serial ports on your system along with their manufacturer information.
    """
    ports = list_ports.comports()
    num_ports = len(ports)

    if num_ports == 0:
        print("No available serial ports found.")
        return

    print(f"Found {num_ports} available serial port(s):")
    for port in ports:
        if os.name == 'posix':
            permissions = "Unknown"
            if os.access(port.device, os.R_OK) and os.access(port.device, os.W_OK):
                permissions = "Read/Write"
            elif os.access(port.device, os.R_OK):
                permissions = "Read-only"
            elif os.access(port.device, os.W_OK):
                permissions = "Write-only"
            else:
                permissions = "No access"
            manufacturer = port.manufacturer if details else "Unknown"
            print(f"Device: {port.device:<20} Manufacturer: {manufacturer:<20} Permissions: {permissions}")
        else:
            description = port.description if details else ""
            if description:
                print(f"Device: {port.device:<20} Description: {description}")
            else:
                print(f"Device: {port.device}")

@click.command(name='probe', short_help='Auto probe device port and baudrate')
@click.option('--ports', default="", help="Comma separated ports, e.g., COM3,COM5")
def cmd_probe(ports):
    port_list = list_ports.comports()
    if not port_list:
        print("No available serial ports found.")
        return

    baud_rates = [115200, 921600, 9600, 460800, 230400, 256000]
    deadline = time.monotonic() + 3.0

    allow_ports = {p.strip().upper() for p in ports.split(",") if p.strip()}

    def try_probe(device_name, baud, result):
        try:
            ser = serial.Serial(port=device_name, baudrate=int(baud), timeout=0.05, write_timeout=0.05)
        except serial.SerialException:
            return
        try:
            try:
                ser.write(b"LOG DISABLE\r\n")
            except (serial.SerialTimeoutException, serial.SerialException):
                return

            try:
                ser.write(b"AT+INFO\r\n")
            except (serial.SerialTimeoutException, serial.SerialException):
                return

            buf = b""
            until = time.monotonic() + 0.25
            while time.monotonic() < until:
                try:
                    waiting = ser.in_waiting
                    if waiting:
                        buf += ser.read(waiting)
                    else:
                        time.sleep(0.01)
                except (serial.SerialTimeoutException, serial.SerialException):
                    return
            text = buf.decode(errors="ignore")
            if "OK" in text:
                result["ok"] = True
                result["port"] = device_name
                result["baud"] = baud
                result["text"] = text.strip()
                try:
                    ser.write(b"LOG ENABLE\r\n")
                except (serial.SerialTimeoutException, serial.SerialException):
                    pass
        finally:
            ser.close()

    for port in port_list:
        device_name = port.device
        if os.name == "nt":
            dev_upper = device_name.upper()
            if not dev_upper.startswith("COM"):
                continue
            if allow_ports and dev_upper not in allow_ports:
                continue
        for baud in baud_rates:
            if time.monotonic() > deadline:
                print("Probe timeout.")
                return
            print(f"Probing {device_name} @ {baud} ...")
            result = {"ok": False, "port": None, "baud": None, "text": ""}
            t = threading.Thread(target=try_probe, args=(device_name, baud, result), daemon=True)
            t.start()
            t.join(0.4)
            if result["ok"]:
                print("\n========== Device Found ==========")
                print(f"Port: {result['port']}")
                print(f"Baud Rate: {result['baud']}")
                print("Device Info:")
                print(result["text"])
                print("\nExample commands:")
                print(f"python main.py read -p {result['port']} -b {result['baud']}")
                print(f"python main.py write -p {result['port']} -b {result['baud']} \"AT+INFO\"")
                print("==================================")
                return

    print("No compatible device found on any port.")


if __name__ == "__main__":
    cmd_list()
