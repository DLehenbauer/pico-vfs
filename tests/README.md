# Tests

The project has six test suites:

1. unit tests `unittests` using heap memory
2. integration tests `integrate` using Raspberry Pi Pico hardware
3. multi-core tests `multicore`
4. unit tests `host` on the host
5. SD SPI protocol tests `sd_spi_tests` on the host with hardware mocks
6. FatFs disk adapter tests `fat_disk_tests` on the host with a storage-free fake device

```bash
cd build
make unittests integration multicore
make run_unittests
make run_integration
make run_multicore
```

```bash
cmake -S . -B build/host -DPICO_PLATFORM=host -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host --target host sd_spi_tests fat_disk_tests
ctest --test-dir build/host --output-on-failure
```

## SD SPI protocol regression tests

These SDK-free host tests exercise the SD driver with deterministic hardware
mocks; see the [suite README](sd_spi/README.md) for details.

## FatFs disk adapter regression tests

These SDK-free host tests exercise the FatFs adapter with a storage-free fake
device; see the [suite README](fat_disk/README.md) for details.
