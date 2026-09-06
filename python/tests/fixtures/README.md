# Protocol fixtures

`hipnuc_protocol.json` contains synthetic, fixed byte vectors for decoder tests.
These are public-protocol examples, not device recordings or
evidence of hardware behavior. The frame CRC is CRC-16/XMODEM over the first
four header bytes followed by the payload; the stored CRC bytes are excluded.
The checked-in values were calculated independently with `binascii.crc_hqx`.

Each entry contains `raw_hex`, its exact `payload_length`, selected decoded SI
`expected` values, and selected unscaled `wire` values from the protocol.
HI83 includes current 64-bit microseconds, historical 32-bit milliseconds, and
an unknown bitmap example. Python preserves a reliable prefix and marks the
last case incomplete. It does not guess field sizes after an unknown bitmap bit.
HI81's reserved tail deliberately includes tag
bytes to verify that it is not interpreted as GNSS or another packet.

The current Python decoder defaults to a maximum 506-byte payload, matching
the C SDK's 512-byte receive buffer minus the 6-byte header. This is a
configurable resource bound, not the wire protocol's 16-bit length limit.
HI83 bits 0–19 and 30–31 match the field dictionary already present in this
public repository; undocumented bits 20–29 remain unknown. No private field
definitions or device capture data are included.

HI91/HI81 can share an outer frame as consecutive fixed-size subpackets.
Python requires HI83 to be the final subpacket: it uses the remaining payload
length to distinguish the two historical time layouts. It rejects additional
trailing subpackets rather than guessing their boundaries.

Physical fields use SI conversions with temperature in degrees Celsius and
latitude/longitude in degrees. HI91 and Modbus use the product's 9.8 m/s² per G
encoding constant; HI83 already carries m/s². Cross-protocol tests start with
one native acceleration and independently specify each wire representation.
HI81 heading is clockwise from north and is
kept separate from the configurable IMU Euler yaw. Quaternion order is WXYZ,
representing body-to-navigation rotation; the navigation coordinate system
must be obtained from the device configuration. Missing/invalid UTC does not
receive a guessed date. `Sample.complete` concerns parsing completeness; fix
quality and sensor status must be checked separately.

After a corrupted binary header or checksum, the Python stream decoder
quarantines ASCII until a checksum-valid binary or NMEA frame establishes a
new boundary, or the caller explicitly resets it. NMEA inside the announced
damaged binary span is excluded. A session containing only ASCII after such
corruption may therefore time out; it must not accept embedded payload text
as a command acknowledgement. Corruption is observable in decoder statistics.
