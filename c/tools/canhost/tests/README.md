# CAN CLI checks

These tests run the real parser, command handlers, JSON formatter, J1939
register matching and CAN updater against a fake transport and clock. They
never open a physical CAN interface.

```sh
cmake -S c/tools/canhost -B build/canhost-tests -DCANHOST_BUILD_TESTS=ON
cmake --build build/canhost-tests
ctest --test-dir build/canhost-tests --output-on-failure
```

Coverage includes invalid arguments before I/O, passive scan, all 8-bit receive
addresses, finishing the current receive batch at a count limit or Ctrl-C,
file protection, flush/close failure, oversized JSON and buffer reuse, foreign
register replies, rejected requests and inconsistent echoes. CAN update tests
distinguish a missing application-start reply from send errors, receive errors
and explicit SDO aborts.

SocketCAN/hardware timing, physical multi-node buses and actual firmware
execution still need equipment tests. Upgrade fixtures are synthetic test
bytes, not firmware images to flash on a product.
