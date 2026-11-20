# SD SPI protocol tests

These tests include the production `src/blockdevice/sd.c` directly so that its
private command helpers and public I/O callbacks are exercised without adding
test hooks to the driver. Only the Pico SPI, GPIO, time, and mutex boundary is
mocked.

Each scenario specifies ordered MOSI bytes, MISO bytes, CS levels, and CS
transitions. Unexpected, missing, or out-of-order transfers, including premature
deselection and duplicate cleanup, fail. CRC7 and CRC16 expectations use
independent bitwise implementations rather than the driver's lookup tables.

Scenario rationale, protocol citations, and compatibility limitations are
documented beside the relevant expectations and assertions in
[`test_sd_spi.c`](test_sd_spi.c).

## Running

The scenarios use the production command timeout of 5 s with deterministic
mock time rather than wall-clock delays.

```bash
cmake -S tests/sd_spi -B build/sd-spi -DCMAKE_BUILD_TYPE=Debug
cmake --build build/sd-spi
ctest --test-dir build/sd-spi --output-on-failure
```

## Specification references

`SD v9.10` citations in the driver and tests refer to the SD Association's [Part
1 Physical Layer Simplified Specification, version 9.10 (December 1,
2023)](https://www.sdcard.org/downloads/pls/).

`SanDisk v2.2` citations refer to the manufacturer's *SD Card Product Manual*,
version 2.2 (November 2004), document 80-13-00169 ([archived
PDF](https://teslabs.com/openplayer/docs/docs/specs/ProductManualSDCardv2.2final.pdf)).

Page numbers are the printed document pages.

## Limitations

Host mocks do not establish physical timing compliance or hardware reliability.
Mutex ownership checks do not simulate contention or real multicore scheduling,
and heap-backed filesystem I/O does not model internal flash translation or wear.
