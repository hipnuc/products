import pynmea2
import logging
from datetime import datetime
from typing import Dict, List, Optional
from dataclasses import dataclass

logging.basicConfig(level=logging.WARNING, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

@dataclass
class SXTData:
    message_id: str
    talker_id: str
    utc_time: datetime
    longitude: float
    latitude: float
    altitude: float
    yaw: float
    pitch: float
    speed_heading: float
    horizontal_speed: float
    roll: float
    pos_solq: int
    heading_solq: int
    pos_nv: int
    heading_nv: int
    wb_x: float
    wb_y: float
    wb_z: float
    vel_e: float
    vel_n: float
    vel_u: float
    ins_status: int
    ant_status: int


class hipnuc_nmea_parser:
    SUPPORTED_SENTENCES = {'GGA', 'RMC', 'VTG', 'GSA', 'GSV', 'SXT'}
    SXT_FIELD_COUNT = 22

    def __init__(self):
        self.buffer = ""

    def parse(self, input_data: str) -> List[Dict]:
        parsed_data = []
        
        self.buffer += input_data
        
        sentences = self._extract_nmea_sentences()
        
        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue
            
            try:
                if sentence.startswith('$GPSXT') or sentence.startswith('$GNSXT'):
                    result = self._parse_sxt(sentence)
                else:
                    msg = pynmea2.parse(sentence, check=True)  # Enable checksum validation
                    if msg.sentence_type in self.SUPPORTED_SENTENCES:
                        result = self._convert_to_dict(msg)
                    else:
                        result = None
                
                if result:
                    parsed_data.append(result)
            except pynmea2.ParseError as e:
                logger.warning(f"Parse error: {e}")
            except pynmea2.ChecksumError as e:
                logger.warning(f"Checksum error: {e}")
            except Exception as e:
                logger.error(f"Unexpected error while parsing NMEA string: {e}")
        
        return parsed_data

    def _extract_nmea_sentences(self) -> List[str]:
        """
        Extract NMEA sentences from the buffer and validate their checksums.

        Returns:
            List[str]: A list of valid NMEA sentences.
        """
        sentences = []
        start = 0
        while True:
            # Find the start of the next NMEA sentence
            start = self.buffer.find('$', start)
            if start == -1:
                break
            
            # Find the end of the sentence
            end = self.buffer.find('\n', start)
            if end == -1:
                break
            
            # Extract the sentence
            sentence = self.buffer[start:end].strip()
            
            # Check if the sentence contains a checksum
            if '*' in sentence:
                # Split the sentence and checksum
                sentence_parts = sentence.rsplit('*', 1)
                if len(sentence_parts) == 2:
                    sentence_body, checksum = sentence_parts
                    
                    # Validate the checksum
                    if self._validate_checksum(sentence_body + '*', checksum):
                        sentences.append(sentence)
                
                # Move the buffer pointer to the next sentence
                self.buffer = self.buffer[end+1:]
                start = 0
            else:
                # If no checksum found, move to the next potential sentence
                start = end + 1
        
        return sentences

    def _convert_to_dict(self, msg: pynmea2.NMEASentence) -> Dict:
        return {
            'message_id': msg.sentence_type,
            **{field[1]: getattr(msg, field[1], None) for field in msg.fields}
        }

    def _parse_sxt(self, nmea_string: str) -> Optional[Dict]:
        parts = nmea_string.split(',')
        if len(parts) != self.SXT_FIELD_COUNT:
            logger.warning(f"Unexpected number of fields in SXT sentence: {len(parts)}")
            return None

        try:
            # Extract and validate checksum
            checksum = parts[-1].split('*')[1]
            if not self._validate_checksum(nmea_string, checksum):
                logger.warning("Checksum validation failed")
                return None

            parts[-1] = parts[-1].split('*')[0]
            
            sxt_data = SXTData(
                message_id='SXT',
                talker_id=parts[0][1:3],
                utc_time=datetime.strptime(parts[1], "%Y%m%d%H%M%S.%f"),
                longitude=float(parts[2]),
                latitude=float(parts[3]),
                altitude=float(parts[4]),
                yaw=float(parts[5]),
                pitch=float(parts[6]),
                speed_heading=float(parts[7]),
                horizontal_speed=float(parts[8]),
                roll=float(parts[9]),
                pos_solq=int(parts[10]),
                heading_solq=int(parts[11]),
                pos_nv=int(parts[12]),
                heading_nv=int(parts[13]),
                wb_x=float(parts[14]),
                wb_y=float(parts[15]),
                wb_z=float(parts[16]),
                vel_e=float(parts[17]),
                vel_n=float(parts[18]),
                vel_u=float(parts[19]),
                ins_status=int(parts[20]),
                ant_status=int(parts[21])
            )
            return vars(sxt_data)
        except ValueError as e:
            logger.error(f"Error parsing SXT sentence: {e}")
            return None
        except IndexError as e:
            logger.error(f"Index error parsing SXT sentence: {e}")
            return None

    def _validate_checksum(self, nmea_string: str, checksum: str) -> bool:
        calculated_checksum = 0
        for char in nmea_string[1:]:  # Skip the leading '$'
            if char == '*':
                break
            calculated_checksum ^= ord(char)
        return "{:02X}".format(calculated_checksum) == checksum.upper()
    
    @staticmethod
    def print_parsed_data(data: List[Dict]):
        if not data:
            print("No parsed NMEA data available.")
            return

        for idx, item in enumerate(data, 1):
            if not item:
                print(f"\n--- Sentence {idx}: Empty NMEA data ---")
                continue

            print(f"=== {item.get('message_id', 'Unknown')} ===")
            
            for key, value in item.items():
                if key != 'message_id':
                    print(f"{key}: {value}")
            
            print()



if __name__ == "__main__":
    parser = hipnuc_nmea_parser()

    # Example string with multiple NMEA sentences
    example_string = """$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
    $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
    $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
    $GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75
    $GPSXT,20230310090529.59,116.45784882,39.90572287,158.2289,359.87,-4.99,359.87,0.001,171.25,1,0,15,15,0.056,-0.040,0.017,-0.001,-0.000,0.002,8,0*43\n"""

    print("Parsing multiple NMEA sentences:")
    parsed_data = parser.parse(example_string)
    
    hipnuc_nmea_parser.print_parsed_data(parsed_data)

    print(f"\nTotal number of successfully parsed sentences: {len(parsed_data)}")