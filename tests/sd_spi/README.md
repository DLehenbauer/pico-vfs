# SD SPI protocol tests

These tests include the production `src/blockdevice/sd.c` directly so that its
private command helpers and public read callback are exercised without adding
test hooks to the driver. Only the Pico SPI, GPIO, time, and mutex boundary is
mocked.

Each scenario specifies ordered MOSI bytes, MISO bytes, CS levels, and CS
transitions. Unexpected, missing, or out-of-order transfers, including premature
deselection and duplicate cleanup, fail. CRC7 and CRC16 expectations use
independent bitwise implementations rather than the driver's lookup tables.

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

Comments distinguish protocol requirements from host tolerance, API policy,
and synthetic boundary fixtures. The 22-bit CSD encoding-limit fixture is not a
claim that the corresponding capacity is a valid SDXC user-area size.

The SPI1 pin assertion is instead based on the [RP2040
datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf),
section 1.4.3, Table 2, pp.12-13; host pin routing is not defined by SD.

## Limitations

Command responses (including R2/R3/R7 continuation bytes) and write-data
acknowledgments are exercised at every bit offset. Read-start tokens, read
payloads, and their CRCs remain byte-aligned; their realignment is not implemented.
Cards that also shift read blocks are not supported. With CRC disabled, the
driver can mistake a payload byte for the start token and return corrupted data
without an error. CRC checking reduces this risk but cannot guarantee detection.

The mock clock advances deterministically. The suite runs with both a 5 ms
command timeout and the production default of 5 s, including the minimum erase
timeout for a single-block trim.
