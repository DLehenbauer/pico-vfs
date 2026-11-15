# Tests

The project has five test suites:

1. unit tests `unittests` using heap memory
2. integration tests `integrate` using Raspberry Pi Pico hardware
3. multi-core tests `multicore`
4. unit tests `host` on the host
5. SD SPI protocol tests `sd_spi_tests` on the host with hardware mocks

```bash
cd build
make unittests integration multicore
make run_unittests
make run_integration
make run_multicore
```

```bash
cmake -S . -B build/host -DPICO_PLATFORM=host -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host --target host sd_spi_tests
ctest --test-dir build/host --output-on-failure
```

## SD SPI protocol regression tests

These host tests compile the actual SD driver against deterministic SPI, GPIO,
clock, and mutex mocks, without requiring the Pico SDK or a card:

```bash
cmake -S tests/sd_spi -B build/sd-spi -DCMAKE_BUILD_TYPE=Debug
cmake --build build/sd-spi
ctest --test-dir build/sd-spi --output-on-failure
```

The host CI workflow builds both host executables in the same CMake build and
runs both suites with CTest. The standalone commands above remain available
without the Pico SDK.
