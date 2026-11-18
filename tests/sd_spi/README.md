# SD SPI protocol tests

These tests include the production `src/blockdevice/sd.c` directly so that its
private command helpers and public I/O callbacks are exercised without adding
test hooks to the driver. Only the Pico SPI, GPIO, time, and mutex boundary is
mocked.

Each scenario specifies ordered MOSI bytes, MISO bytes, CS levels, and CS
transitions. Unexpected, missing, or out-of-order transfers, including premature
deselection and duplicate cleanup, fail. CRC7 and CRC16 expectations use
independent bitwise implementations rather than the driver's lookup tables.

Lifecycle scenarios check mutex acquisition and both initialization flags at
lock entry, throughout initialization SPI activity, and before unlocking. Cold
SDSC/SDHC startup with CRC enabled/disabled, retries after each top-level setup
failure, and repeated deinitialization followed by reinitialization exercise
successful read/program/trim callbacks. Initialized invalid requests must return
parameter errors without SPI activity; uninitialized requests must return
`NO_INIT` before inspecting undiscovered erase geometry.

```bash
cmake -S tests/sd_spi -B build/sd-spi -DCMAKE_BUILD_TYPE=Debug
cmake --build build/sd-spi
ctest --test-dir build/sd-spi --output-on-failure
```

## Limitations

The mock clock advances deterministically. The default command timeout is 5 ms
in tests and remains 5 s in production.
Mutex ownership checks do not simulate contention or real multicore scheduling.
