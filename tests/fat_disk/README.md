# FatFs disk adapter tests

These host tests compile the production `src/filesystem/fat.c` and bundled
FatFs without requiring the Pico SDK or storage hardware. A recording block
device covers disk I/O ranges without allocating backing storage, while an
in-memory heap block device exercises real FatFs format, open, truncation,
logical cursor, append, metadata, and error-handling paths.

Open regressions cover injected existence-probe failures, a full FAT12 root,
and FAT32 mounted alongside a second volume. Linker wrappers inject probe errors
without adding test hooks to FatFs; a GNU-compatible linker with `--wrap` is required.

Partial-extension regressions use real FAT12 and exFAT volumes, inject transfer
and subsequent metadata-write failures, and verify the accepted prefix after
sync retries, close, and remount without clearing the file's error latch.
Write-ceiling tests simulate enormous FAT32 and exFAT files through linker
wrappers, exercising the production adapter's signed-offset limits and append
policy without allocating gigabytes of backing storage.

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

The recording and heap devices do not model card protocols or physical media
behavior. Mutex checks do not simulate contention or real multicore scheduling.
