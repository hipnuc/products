# Development checks

From the repository root:

```sh
cmake -S c -B build/c -DHIPNUC_BUILD_TESTS=ON
cmake --build build/c --config Release
ctest --test-dir build/c -C Release --output-on-failure
```

Windows defaults to MSVC when installed; MinGW uses `-G "MinGW Makefiles"`.
Tests cover binary/NMEA/CAN semantics, malformed input,
JSON sizing, independent copied-file consumers, C-only parent CMake integration,
and C++ headers. The real serial example entry points are tested for attitude-only
data, idle/recovery notices, transport failures and Ctrl-C using a fake transport.
Serial stream tests use a deterministic transport; Linux port tests use pseudoterminals,
including descriptor zero, custom baud rates, independent connections, total
timeouts (also after early OS wakeups) and disconnect cleanup.
No test opens a physical serial port.

The standalone `c/tests` build remains available. For a C-only desktop build,
use `cmake -S c -B build/c-only -DHIPNUC_BUILD_CPP_EXAMPLE=OFF`; host tests require
C++. The default standalone build includes examples, but no tests.

Related checks: `stm32/tests` exercises the actual board receive code through
peripheral stubs; `ros/tests` checks message conversion and provides integration
tests for a sourced ROS installation. CAN CLI, recording and firmware-update
tests are maintained with the Python SDK in `../../python/tests`.

Host tests and builds do not prove device throughput or board timing. Before a
hardware release, separately check sustained UART/CAN reception under printing
load, overflow reporting and disconnect/reopen. Build Keil in a temporary copy to avoid IDE edits
to customer project files. Never automatically flash a board as part of CI.
