[English](README.md) | [中文](README_zh.md)

# CAN database

Import [J1939.dbc](J1939.dbc) into a DBC-capable CAN analysis tool to view
HiPNUC measurements. Vector CANdb++ can edit the database.

- Classic CAN, 29-bit identifiers, eight-byte payloads, little-endian signals.
- The 13 message identifiers use priority **3** and source address **0x08**.
  If your device uses another address or priority, update the identifiers in
  your tool or a copy of the database.
- Dynamic CANFD83 packets are not covered. Use the Python or C decoder for them.

The database retains wire units: acceleration **G** (1 G = 9.8 m/s²), angular
velocity **deg/s**, magnetic field **µT** and angles **degrees**. DBC signal
ranges do not establish product accuracy or a valid solution.

`Heading_CW` and `Euler_Yaw` are different quantities. `Year_Offset_2000` is the
year minus 2000 only when the date is available; a zero year/month/day means
the remaining time fields are uptime modulo 24 hours. FF43 supplies temperature;
its reserved bytes are **not pressure**. GNSS quality does not establish INS
position validity, and separate PGNs do not form a synchronized sample.

See your product's IMU/INS manual for supported messages, coordinate settings
and status values.
