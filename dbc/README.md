[English](README.md) | [中文](README_zh.md)

# CAN database

[J1939.dbc](J1939.dbc) describes the HiPNUC J1939 data messages for CAN
analysis tools such as CANalyzer, CANoe, PCAN-Explorer, asammdf and MATLAB.
To read the values without a database, run `hihost can read` (see
[python/](../python/README.md)).

## What it covers

- 13 data PGNs, 0xFF10 to 0xFF4A: Classic CAN, 29-bit identifiers, 8 bytes,
  little-endian.
- By default an IMU module sends acceleration (FF34), angular rate (FF37),
  roll/pitch (FF3D) and heading (FF41) every 10 ms. Switch on the others with
  `hihost can period`, for example `hihost can period quat 10 -i can0 --id 8`
  for the quaternion (FF46).
- FF10, FF14, FF18 and FF26 come from INS models, FF3A from models with a
  magnetometer, and FF43 not from every model.
- CANFD83 frames of the CAN FD models are not covered; decode them with the
  Python or C decoder.

## Node address

The identifiers use priority 3 and the factory node address 8, for example
`0x0CFF3408`. The database is marked as J1939, so tools that support J1939
databases match the PGN and decode a module at any node address. With
cantools, mask out the priority and address:

```sh
candump can0 | python3 -m cantools decode -m 0x03FFFF00 J1939.dbc
```

```python
import cantools

db = cantools.database.load_file("J1939.dbc", frame_id_mask=0x03FFFF00)
values = db.get_message_by_frame_id(msg.arbitration_id).decode(msg.data)
```

Tools that compare whole identifiers decode node address 8 only; for another
address, edit the identifiers in the tool.

## Values

- Acceleration is in G (the firmware uses 1 G = 9.8 m/s²), angular rate in
  deg/s, magnetic field in µT and angles in degrees.
- `Heading_CW` runs clockwise from 0 to 360°; `Euler_Yaw` is ±180° in the
  output axes setting. With `COORD 0` both give the same heading.
- FF2F: year, month and day are 0 until the module has UTC; the time fields
  then count from power-up, modulo 24 hours.
- The FF18 quality and status fields come with value names.

See your product's IMU or INS manual for the PGN list, periods and the
configuration protocol.
