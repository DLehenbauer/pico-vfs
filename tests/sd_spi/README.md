# SD SPI protocol tests

These tests include the production `src/blockdevice/sd.c` directly so that its
private command helpers and public I/O callbacks are exercised without adding
test hooks to the driver. Only the Pico SPI, GPIO, time, and mutex boundary is
mocked.

Each scenario specifies ordered MOSI bytes, MISO bytes, CS levels, and CS
transitions. Unexpected, missing, or out-of-order transfers, including premature
deselection and duplicate cleanup, fail. CRC7 and CRC16 expectations use
independent bitwise implementations rather than the driver's lookup tables.

The trim geometry cases initialize SDSC and SDHC cards from CSD fixtures, then
check actual CMD32/CMD33/CMD38 exchanges and inward-rounded ranges. SDSC cases
cover both ERASE_BLK_EN values, independent read and write
block lengths, 512/1024/2048-byte native write blocks, minimum/maximum SECTOR_SIZE,
and non-power-of-two erase units. Ranges include a single erase unit, multiple
units, address zero, and the final complete unit at capacity. Read/program and
no-op erase still accept 512-byte logical sectors with larger trim alignment.
With ERASE_BLK_EN clear, native write lengths below 512 bytes fail initialization
in both CRC modes, without CMD16 or subsequent trim traffic. SECTOR_SIZE counts
native write blocks, as in the pinned [Linux CSD decoder](https://github.com/torvalds/linux/blob/666539eecdf4db623f5e3c3e55cf9f2dfad24aa4/drivers/mmc/core/sd.c#L137-L145).
The pinned [FreeBSD decoder](https://github.com/freebsd/freebsd-src/blob/253c83058deb5ff77dc0a3b60bf7c1d10f9ef5e8/sys/dev/mmc/mmc.c#L1674-L1675)
likewise multiplies the erase-sector count by the native write-block length
before converting to 512-byte sectors.
Public `erase_size` stays at 512: SD writes overwrite sectors and `erase()` is a
no-op. Changing it to match trim alignment would change FAT logical sectors and
littlefs on-disk geometry.

`trim()` is advisory discard, not erase-before-write or secure deletion. The
card's controller handles physical flash erase/program operations, so boundary
sectors left untrimmed remain writable. After validating the original byte
range without overflow, the driver rounds the start up and end down, following
the pinned [Linux erase alignment policy](https://github.com/torvalds/linux/blob/666539eecdf4db623f5e3c3e55cf9f2dfad24aa4/drivers/mmc/core/core.c#L1734-L1790).
Rounding outward could destroy live data outside the request. Empty ranges and
ranges containing no complete erase unit succeed without SPI activity;
out-of-bounds requests fail, and actual card errors still propagate.

FAT formatting trims the whole device after writing its initial header sectors.
Capacity need not be a multiple of the native erase unit: rejecting a partial
tail would abort formatting unnecessarily. The integration case initializes
such an SDSC geometry and runs real FAT formatting with heap-backed reads and
writes, routing trim through the SD driver and its ordered SPI mock. It then
checks actual SD reads and writes to the untrimmed tail. Other cases cover
non-power-of-two units, wide addresses, bounds/overflow rejection, empty
results, command errors, and busy-timeout scaling after rounding.

Lifecycle scenarios check mutex acquisition and both initialization flags at
lock entry, throughout initialization SPI activity, and before unlocking. Cold
SDSC/SDHC startup with CRC enabled/disabled, retries after each top-level setup
failure, and repeated deinitialization followed by reinitialization exercise
successful read/program/trim callbacks. Invalid read/program requests and
out-of-bounds trim requests must return parameter errors without SPI activity;
uninitialized requests, including empty trim, must return `NO_INIT` before
inspecting undiscovered erase geometry.

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

Command responses, including R2/R3/R7 continuation bytes, and write-data
acknowledgments are exercised at every bit offset. Read-start tokens, read
payloads, and their CRCs remain byte-aligned; their realignment is not
implemented. Cards that also shift read blocks are not supported. With CRC
disabled, the driver can mistake a payload byte for the start token and return
corrupted data without an error. CRC checking reduces this risk but cannot
guarantee detection.

The mock clock advances deterministically. The default command timeout is 5 ms
in tests and remains 5 s in production.
Mutex ownership checks do not simulate contention or real multicore scheduling.
Heap-backed filesystem I/O does not model the card's internal flash translation
or physical wear.
