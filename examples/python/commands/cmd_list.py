import click
import serial
from serial.tools import list_ports
import os

@click.command(name='list', short_help='List all available serial ports')
def cmd_list():
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
        manufacturer = port.manufacturer or 'Unknown'
        permissions = "Unknown"
        
        if os.name == 'posix':  # Check permissions in Linux environment
            if os.access(port.device, os.R_OK) and os.access(port.device, os.W_OK):
                permissions = "Read/Write"
            elif os.access(port.device, os.R_OK):
                permissions = "Read-only"
            elif os.access(port.device, os.W_OK):
                permissions = "Write-only"
            else:
                permissions = "No access"

        print(f"Device: {port.device:<20} Manufacturer: {manufacturer:<20} Permissions: {permissions}")

if __name__ == "__main__":
    cmd_list()
