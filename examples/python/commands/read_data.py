import sys
import time
import json
import serial
import click
from parsers.hipnuc_serial_parser import hipnuc_parser
from parsers.hipnuc_nmea_parser import hipnuc_nmea_parser



@click.command(name='read', short_help='Read data from the specified serial port')
@click.option('--port', '-p', required=True, help="The serial port to connect to (e.g., COM3 or /dev/ttyUSB0)")
@click.option('--baudrate', '-b', default='115200', help="The baud rate for the serial connection (default: 115200)")
@click.option('--record-raw', '-r', default=None, help="Record raw serial data to a binary file")
@click.option('--record-json', '-j', default=None, help="Record parsed data to a JSONL file")
def cmd_read(port, baudrate, record_raw, record_json):
    if not baudrate.isdigit() or int(baudrate) <= 0:
        raise click.BadParameter("Invalid baudrate. Baudrate must be a positive integer.")

    serial_parser = hipnuc_parser()
    nmea_parser = hipnuc_nmea_parser()

    frame_count = 0
    frame_rate = 0
    last_frame_time = time.time()
    last_display_time = time.time()
    display_interval = 0.1

    latest_hipnuc_packet = None
    latest_nmea_packets = []

    raw_fp = None
    json_fp = None

    try:
        with serial.Serial(port, int(baudrate), timeout=1) as ser:
            if record_raw:
                raw_fp = open(record_raw, "wb")
                print(f"Raw data will be recorded to: {record_raw}")
            else:
                raw_fp = None

            if record_json:
                json_fp = open(record_json, "w", encoding="utf-8")
                print(f"JSON data will be recorded to: {record_json}")
            else:
                json_fp = None

            ser.write(b"LOG ENABLE\r\n")
            while True:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    if raw_fp:
                        raw_fp.write(data)
                        raw_fp.flush()
                    
                    try:
                        hipnuc_packets = serial_parser.parse(data)
                        nmea_packets = nmea_parser.parse(data.decode('ascii', errors='ignore'))
                        
                        frame_count += len(hipnuc_packets) + len(nmea_packets)
                        
                        if hipnuc_packets:
                            latest_hipnuc_packet = hipnuc_packets[-1]
                        if nmea_packets:
                            latest_nmea_packets = nmea_packets

                        if json_fp:
                            for packet in hipnuc_packets:
                                json_fp.write(json.dumps(packet, ensure_ascii=False) + "\n")
                            for packet in nmea_packets:
                                json_fp.write(json.dumps(packet, ensure_ascii=False) + "\n")
                            json_fp.flush()
                        
                        current_time = time.time()
                        if current_time - last_frame_time >= 1.0:
                            frame_rate = frame_count
                            frame_count = 0
                            last_frame_time = current_time

                        # Update display at fixed interval
                        if current_time - last_display_time >= display_interval:
                            if latest_hipnuc_packet:
                                print(json.dumps(latest_hipnuc_packet, ensure_ascii=False))
                            if latest_nmea_packets:
                                for packet in latest_nmea_packets:
                                    print(json.dumps(packet, ensure_ascii=False))
                            print(json.dumps({"type": "status", "frame_rate_hz": frame_rate}, ensure_ascii=False))
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
    finally:
        try:
            if raw_fp:
                raw_fp.close()
        except Exception:
            pass
        try:
            if json_fp:
                json_fp.close()
        except Exception:
            pass

if __name__ == "__main__":
    cmd_read()
