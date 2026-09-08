# EtherCAT example (HI15)

[English](README.md) | [中文](README_zh.md)

Read HI15 acceleration, angular velocity, quaternion and temperature on Linux
using IgH EtherCAT Master **1.6**. Edit the parameters at the top of `main.c`,
then build and run. Defaults: master 0, alias 0, position 0, DC enabled, 1 kHz
bus cycle and 20 Hz display. Ctrl-C stops and releases the master.

## Prepare and run

1. Install IgH 1.6, including its userspace development library, a C compiler
   and CMake 3.16 or newer. Follow the [IgH installation instructions](https://gitlab.com/etherlab.org/ethercat/-/blob/stable-1.6/INSTALL.md)
   to configure a dedicated Ethernet interface and start the master service.
2. Connect and power the HI15. Run `ethercat slaves -v` and confirm the position,
   vendor ID `0x00131415` and product code `0x00009253`. Close other applications
   using this master.
3. From this directory:

```sh
cmake -S . -B build && cmake --build build
sudo ./build/userexample
```

`sudo` permits access to the master device; it is unnecessary when your system
already grants your account that access. For IgH installed in a custom prefix,
add `-DCMAKE_PREFIX_PATH=/path/to/igh` to the CMake command.

An ordinary Linux kernel can be used to try reading. DC at 1 kHz is sensitive to
scheduling latency; this example does not guarantee real-time timing. For
sustained synchronized operation, follow [IgH's real-time guidance](https://etherlab.org/en_GB/getting-started).
There is no need to compile a custom kernel just to start using the example.

## Data and integration

`hi15.c` holds the fixed PDO mapping and master lifecycle; `main.c` is the
receive/process/read/send loop. Copy these files and `hi15.h` into your own IgH
application, keeping one owner of the master and one cyclic receive loop.

- Acceleration: **m/s²**; angular velocity: **rad/s**; temperature: **°C**.
- Quaternion: **WXYZ**, in the device's configured reference frame. The example
  does not change the device configuration or transform coordinates.
- `system_time`: device uptime in **milliseconds**, wrapping at 32 bits.
- Only complete PDO exchanges from an operational slave are displayed. Repeated
  device timestamps are possible; an exchange is not necessarily a new sample.
- RxPDO `0x7000:01` is reserved and is written as zero.

The [HI15 ESI file](https://download.hipnuc.com/esi/hi15_esi.zip) describes the
device identity and PDOs for master configuration. This C example uses the fixed
mapping directly and does not load the XML at runtime.

If no slave appears, check power, cabling and the master interface. If the
program reports `No valid PDO`, check the reported AL state, matching device
position/identity and the master log. Measurements resume when valid exchanges
return; previous readings are not presented as current data.
