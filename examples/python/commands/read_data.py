import sys
import time
import serial
import click
from utils import clear_screen
from parsers.hipnuc_serial_parser import hipnuc_parser
from parsers.hipnuc_nmea_parser import hipnuc_nmea_parser



@click.command(name='read', short_help='Read data from the specified serial port')
@click.option('--port', '-p', required=True, help="The serial port to connect to (e.g., COM3 or /dev/ttyUSB0)")
@click.option('--baudrate', '-b', default='115200', help="The baud rate for the serial connection (default: 115200)")
def cmd_read(port, baudrate):
    if not baudrate.isdigit() or int(baudrate) <= 0:
        raise click.BadParameter("Invalid baudrate. Baudrate must be a positive integer.")

    serial_parser = hipnuc_parser()
    nmea_parser = hipnuc_nmea_parser()

    frame_count = 0
    frame_rate = 0
    last_frame_time = time.time()
    last_display_time = time.time()
    display_interval = 0.2  # Update display every 0.2 seconds

    latest_hipnuc_frame = None
    latest_nmea_frames = []

    try:
        with serial.Serial(port, int(baudrate), timeout=1) as ser:
            while True:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    
                    try:
                        hipnuc_frames = serial_parser.parse(data)
                        nmea_frames = nmea_parser.parse(data.decode('ascii', errors='ignore'))
                        
                        frame_count += len(hipnuc_frames) + len(nmea_frames)
                        
                        if hipnuc_frames:
                            latest_hipnuc_frame = hipnuc_frames[-1]
                        if nmea_frames:
                            latest_nmea_frames = nmea_frames
                        
                        current_time = time.time()
                        if current_time - last_frame_time >= 1.0:
                            frame_rate = frame_count
                            frame_count = 0
                            last_frame_time = current_time

                        # Update display at fixed interval
                        if current_time - last_display_time >= display_interval:
                            clear_screen()
                            
                            if latest_hipnuc_frame:
                                serial_parser.print_parsed_data(latest_hipnuc_frame)
                            if latest_nmea_frames:
                                nmea_parser.print_parsed_data(latest_nmea_frames)
                            
                            print(f"Frame rate: {frame_rate} Hz")
                            last_display_time = current_time
                            
                    except Exception as e:
                        print(f"Error parsing data: {e}")

                time.sleep(0.001)  # Small delay to prevent CPU overuse

    except KeyboardInterrupt:
        print("Program interrupted by user")
    except (serial.SerialException, PermissionError) as e:
        print(f"Error: {e}")
        print(f"To run this script with superuser privileges, use the 'sudo' command:")
        print(f"Example: sudo python main.py read --port {port} --baudrate {baudrate}")
        sys.exit(1)

if __name__ == "__main__":
    cmd_read()
