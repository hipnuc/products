# Development checks

From the repository root:

```sh
cmake -S c/tests -B build/c-tests
cmake --build build/c-tests --config Release
ctest --test-dir build/c-tests -C Release --output-on-failure
cmake -S c/examples -B build/examples
cmake --build build/examples --config Release
cmake -S c/tools -B build/tools
cmake --build build/tools --config Release
python c/tests/check_tools.py build/tools
```

Windows defaults to MSVC when installed; MinGW uses `-G "MinGW Makefiles"`.
Tests cover binary/NMEA/CAN semantics, malformed input, firmware-update replies,
JSON sizing, independent copied-file consumers and C++ headers. Serial stream
tests use a deterministic transport; Linux port tests use pseudoterminals,
including descriptor zero, custom baud rates, independent connections, total
timeouts (also after early OS wakeups) and disconnect cleanup. Updater CLI
tests cover cleanup, cancellation, failed input purge and failure without reset.
No test opens a physical serial port.

Related checks: `stm32/tests` exercises the actual board receive code through
peripheral stubs; `ros/tests` checks message conversion and provides integration
tests for a sourced ROS installation. SocketCAN tool tests are under
`../tools/canhost/tests`. Python checks remain in `../../python/tests`.

Host tests and builds do not prove device throughput or board timing. Before a
hardware release, separately check sustained UART/CAN reception under printing
load, overflow reporting, disconnect/reopen, and firmware updates using an image
confirmed for the exact target. Build Keil in a temporary copy to avoid IDE edits
to customer project files. Never automatically flash a board as part of CI.
