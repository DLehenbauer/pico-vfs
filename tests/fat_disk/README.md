# FatFs disk adapter tests

These host tests compile the production `src/filesystem/fat.c` and bundled
FatFs against a recording block device and mutex mock, without requiring the
Pico SDK or storage hardware. The fake device records callback ranges and
results without allocating backing storage.

Scenario rationale and boundary calculations are documented beside the
relevant expectations and assertions in [`test_fat_disk.c`](test_fat_disk.c).

## Running

Run these commands from the repository root:

```bash
cmake -S tests/fat_disk -B build/fat-disk -DCMAKE_BUILD_TYPE=Debug
cmake --build build/fat-disk
ctest --test-dir build/fat-disk --output-on-failure
```

CI runs the suite in Debug and Release, and with AddressSanitizer and
nonrecovering UndefinedBehaviorSanitizer:

```bash
cmake -S tests/fat_disk -B build/fat-disk-sanitized \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_FLAGS="-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer"
cmake --build build/fat-disk-sanitized --target fat_disk_tests
UBSAN_OPTIONS=halt_on_error=1 ctest --test-dir build/fat-disk-sanitized --output-on-failure
```

For the combined host build, see [`../README.md`](../README.md).

## Interface references

The bundled FatFs documentation describes the adapter contracts for
[`disk_read`](../../vendor/ff15/documents/doc/dread.html),
[`disk_write`](../../vendor/ff15/documents/doc/dwrite.html), and
[`disk_ioctl`](../../vendor/ff15/documents/doc/dioctl.html).

## Limitations

The fake device does not model filesystem contents, card protocols, or physical
media behavior. Mutex checks do not simulate contention or real multicore
scheduling.
