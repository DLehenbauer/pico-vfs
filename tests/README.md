# Tests

The project has seven test suites:

1. unit tests `unittests` using heap memory
2. integration tests `integration` using Raspberry Pi Pico hardware
3. multi-core tests `multicore`
4. unit tests `host` on the host
5. SD SPI protocol tests `sd_spi_tests` on the host with hardware mocks
6. FatFs disk adapter tests `fat_disk_tests` on the host with a storage-free fake device
7. VFS tests `vfs_tests` on the host with heap and loopback devices

Run the following commands from the repository root. CMake's build command works
with both Ninja and Make.

Configure and build the hardware test suites:

```bash
cmake -S . -B build/pico -DPICO_BOARD=pico -DCMAKE_BUILD_TYPE=Debug
cmake --build build/pico --target unittests integration multicore
```

With OpenOCD installed and detected during configuration, and a Pico connected
through a debug probe, flash and start each suite:

```bash
cmake --build build/pico --target run_unittests
cmake --build build/pico --target run_integration
cmake --build build/pico --target run_multicore
```

Build and run the host test suites:

```bash
cmake -S . -B build/host -DPICO_PLATFORM=host -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host --target host sd_spi_tests fat_disk_tests vfs_tests
ctest --test-dir build/host --output-on-failure
```

## SD SPI protocol regression tests

These SDK-free host tests exercise the SD driver with deterministic hardware
mocks; see the [suite README](sd_spi/README.md) for details.

## FatFs disk adapter regression tests

These SDK-free host tests exercise the FatFs adapter with a storage-free fake
device; see the [suite README](fat_disk/README.md) for details.

## VFS regression tests

These SDK-free host tests execute exclusive creation, errno propagation, descriptor
recovery, and loopback I/O through the production VFS on FAT and littlefs. See the
[suite README](vfs/README.md) for details.
