# HiPNUC Linux tools

[English](README.md) | [中文](README_zh.md)

Two command line tools built on the C core in `c/hipnuc`:

- `hihost` — serial: find the device, show and record the data stream, send
  ASCII commands, update the firmware.
- `canhost` — SocketCAN: J1939 and CANFD83 decoding, register access, trigger
  frames, firmware update over CAN.

Both are Linux only (termios, SocketCAN). `common/` holds the helpers they
share (logger, Intel HEX loader, INI reader).

## Build

```sh
cmake -S c/tools -B build/tools -DCMAKE_BUILD_TYPE=Release
cmake --build build/tools -j
build/tools/hihost/hihost --help
build/tools/canhost/canhost --help
```

Each tool also builds on its own: `cmake -S c/tools/hihost -B build/hihost`.
Requires CMake 3.10 and a C99 compiler.

## hihost

```sh
hihost list                                # serial ports
hihost probe --save                        # find port/baud, write ./hihost.ini
hihost read                                # live JSON display
hihost -r raw.bin -j data.jsonl read       # record raw bytes and JSON lines
hihost write "LOG VERSION"                 # one ASCII command
hihost write hihost/device_setup.ini       # commands from a file
hihost update firmware.hex                 # serial firmware update
```

Port and baud come from `-p`/`-b`, otherwise from the first of `$HIHOST_CONF`,
`./hihost.ini`, `~/.hihost.ini` (keys `port=` and `baud=`). Nothing is written
unless you pass `probe --save`. Serial access usually needs membership in the
`dialout` group.

`read` prints one JSON object per frame (SI units, the same keys as the
Python SDK). `update` works on a running device or on one already in the
bootloader.

## canhost

```sh
sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up

canhost device list
canhost device probe                       # J1939 address claim scan
canhost stream read                        # JSON line per decoded frame
canhost stream record -o run.jsonl         # same, to a file with rx_time_us
canhost trigger sync --count 1             # trigger the PGNs from canhost.ini
canhost config reg read 0x70
canhost config reg write 0x06 1
canhost action run version                 # reset / save need --yes
canhost firmware update -f app.hex         # CAN firmware update, all target nodes
canhost -n 8,9 stream read                 # override the node list
```

Configuration is read from the first of `$CANHOST_CONF`, `./canhost.ini`,
`~/.canhost.ini`, `/etc/canhost.ini`; `canhost/canhost.ini` documents every
key (interface, node list, host source address, CAN FD, `sync.<pgn>` periods).
Only frames whose J1939 source address is in the node list are decoded.
Register replies are matched against the addressed node, so several devices
can share one bus.
