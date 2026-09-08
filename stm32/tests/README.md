# STM32 development checks

The host tests compile the actual board and application sources with a small
fake STM32 peripheral API. They test receive bookkeeping and control flow;
they do not emulate UART/CAN electrical behavior or measure MCU timing.

```sh
cmake -S stm32/tests -B build/stm32-tests
cmake --build build/stm32-tests
ctest --test-dir build/stm32-tests --output-on-failure
```

On Windows with MinGW, add `-G "MinGW Makefiles"` to the first command.

Coverage includes DMA normal and counter wrap, pending TC and CNDTR races,
exactly full and overwritten buffers, reset after dropped UART bytes, CAN
producer/consumer interrupt interleaving, full-address selection, and
startup/stop/recovery reporting. The CAN register model also checks that FIFO
release preserves an overflow arriving after the ISR's initial flag check.
UART checks cover ORE/FE/NE in DMA and byte-interrupt mode, errors during a
DMA snapshot and command transmission, interruption between SR and DR reads,
partial-frame reset and subsequent recovery. Clearing a UART error must pause
DMA requests before the CPU reads DR. These register models do not model
individual bus cycles.
The initial DMA normal-wrap, CAN full-queue and UART fault regressions fail
on the previous implementations.

For Keil, copy `stm32/` and `c/hipnuc/` into a temporary directory preserving
their relative layout, then build the copied projects. Build serial once with
DMA and once with `HIPNUC_BOARD_USE_DMA=0`; build CAN separately. This avoids
the IDE changing the user's `.uvprojx` or `.uvoptx` files. Check warnings,
linker RAM/flash totals and call-graph stack usage, not just the exit code.

Board acceptance still requires continuous data at the intended rate while
printing and application processing run, deliberate receive starvation,
disconnect/recovery, and CAN traffic from multiple source addresses. Save
the baudrate/bitrate, message selection, console setting and observed loss
counts with the result. No host test proves these timing conditions.
