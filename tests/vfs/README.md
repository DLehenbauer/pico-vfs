# VFS regression tests

These SDK-free host tests compile the production VFS, FAT, littlefs, heap, and
loopback implementations. Checks remain active in Release.

Run from the repository root:

```bash
cmake -S tests/vfs -B build/vfs -DCMAKE_BUILD_TYPE=Debug
cmake --build build/vfs --target vfs_tests
ctest --test-dir build/vfs --output-on-failure
```

For ASan and nonrecovering UBSan:

```bash
cmake -S tests/vfs -B build/vfs-sanitized \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_FLAGS="-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer"
cmake --build build/vfs-sanitized --target vfs_tests
UBSAN_OPTIONS=halt_on_error=1 ctest --test-dir build/vfs-sanitized --output-on-failure
```

The suite requires a GNU-compatible linker supporting `--wrap`. Wrappers bridge
host `open`, `read`, `write`, `close`, `lseek`, and `unlink` calls to the production
Newlib syscall hooks, including calls from the loopback driver. A directory-header
shim matches the firmware's `fs_dir_t` API; mutex mocks check balanced ownership
and recursive nesting without simulating contention.

This does not test host libc buffered streams, physical media, or multicore
scheduling. The suite also runs in combined host CTest and CI.
