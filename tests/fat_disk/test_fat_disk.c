#include <stdio.h>
#include <string.h>

#include "blockdevice/heap.h"
#include "filesystem/fat.h"
#include "pico/mutex.h"
#include "ff.h"
#include "diskio.h"

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);       \
            exit(EXIT_FAILURE);                                                 \
        }                                                                       \
    } while (0)

// Record byte ranges without accessing buffers, so huge logical devices and
// transfers need no backing allocation. Counters detect I/O on rejected ranges.
typedef struct {
    bd_size_t capacity;
    bd_size_t addr;
    bd_size_t size;
    unsigned reads;
    unsigned programs;
    unsigned erases;
    unsigned trims;
    bool reject_empty;
} fake_disk_t;

static bd_size_t fake_size(blockdevice_t *device) {
    fake_disk_t *disk = device->config;
    return disk->capacity;
}

static int fake_read(blockdevice_t *device, const void *buffer, bd_size_t addr, bd_size_t size) {
    (void)buffer;
    fake_disk_t *disk = device->config;
    disk->addr = addr;
    disk->size = size;
    ++disk->reads;
    return disk->reject_empty && size == 0 ? BD_ERROR_DEVICE_ERROR : 0;
}

static int fake_program(blockdevice_t *device, const void *buffer, bd_size_t addr, bd_size_t size) {
    fake_disk_t *disk = device->config;
    (void)buffer;
    CHECK(addr == disk->addr && size == disk->size);
    ++disk->programs;
    return disk->reject_empty && size == 0 ? BD_ERROR_DEVICE_ERROR : 0;
}

static int fake_erase(blockdevice_t *device, bd_size_t addr, bd_size_t size) {
    fake_disk_t *disk = device->config;
    disk->addr = addr;
    disk->size = size;
    ++disk->erases;
    return 0;
}

static int fake_trim(blockdevice_t *device, bd_size_t addr, bd_size_t size) {
    fake_disk_t *disk = device->config;
    disk->addr = addr;
    disk->size = size;
    ++disk->trims;
    return 0;
}

// Test adapter arithmetic independently of storage and card protocols. Large
// capacities here do not imply SDUC driver support or filesystem format limits.
static void test_fat_disk(size_t sector_size) {
    fake_disk_t disk = {0};
    blockdevice_t device = {
        .read = fake_read,
        .program = fake_program,
        .erase = fake_erase,
        .trim = fake_trim,
        .size = fake_size,
        .erase_size = sector_size,
        .config = &disk,
    };
    filesystem_t *fs = filesystem_fat_create();
    CHECK(fs != NULL);
    // Register drive 0 without probing for a filesystem on the storage-free fake.
    CHECK(fs->mount(fs, &device, true) == 0);

    // Straddle the 32-bit count boundary, then cover SDUC-scale geometry and
    // the byte-addressable limit without exercising any SDUC protocol.
    const bd_size_t sector_counts[] = {
        1, 8192, UINT32_MAX, UINT64_C(1) << 32,
        UINT64_C(1) << 38, UINT64_MAX / sector_size,
    };
    BYTE buffer[1] = {0}; // Callbacks record ranges without touching storage.
    for (size_t i = 0; i < sizeof(sector_counts) / sizeof(sector_counts[0]); ++i) {
        bd_size_t sectors = sector_counts[i];
        disk.capacity = sectors * sector_size;
        LBA_t count;
        // A DWORD-only store must fail even when the expected high bits are zero.
        memset(&count, 0xA5, sizeof(count));
        CHECK(disk_ioctl(0, GET_SECTOR_COUNT, &count) == RES_OK);
        CHECK(count == sectors);

        CHECK(disk_read(0, buffer, sectors - 1, 1) == RES_OK);
        CHECK(disk.addr == (sectors - 1) * sector_size && disk.size == sector_size);
        CHECK(disk_write(0, buffer, sectors - 1, 1) == RES_OK);
        CHECK(disk.addr == (sectors - 1) * sector_size && disk.size == sector_size);
        LBA_t range[2] = {sectors - 1, sectors - 1};
        CHECK(disk_ioctl(0, CTRL_TRIM, range) == RES_OK);
        CHECK(disk.addr == (sectors - 1) * sector_size && disk.size == sector_size);
        range[0] = 0;
        CHECK(disk_ioctl(0, CTRL_TRIM, range) == RES_OK);
        CHECK(disk.addr == 0 && disk.size == disk.capacity);
    }

    // An incomplete trailing sector is excluded from the reported capacity.
    disk.capacity = UINT64_MAX;
    LBA_t count = 0;
    CHECK(disk_ioctl(0, GET_SECTOR_COUNT, &count) == RES_OK);
    CHECK(count == UINT64_MAX / sector_size);

    // Inclusive endpoints spanning the 32-bit boundary still describe 3 sectors.
    LBA_t range[2] = {(UINT64_C(1) << 32) - 1, (UINT64_C(1) << 32) + 1};
    CHECK(disk_ioctl(0, CTRL_TRIM, range) == RES_OK);
    CHECK(disk.addr == range[0] * sector_size && disk.size == 3 * sector_size);

    // Exercise byte lengths that overflow 32-bit multiplication at a high LBA.
    const UINT counts[] = {UINT32_MAX / sector_size + 1, UINT32_MAX};
    for (size_t i = 0; i < sizeof(counts) / sizeof(counts[0]); ++i) {
        CHECK(disk_read(0, buffer, UINT64_C(1) << 32, counts[i]) == RES_OK);
        CHECK(disk.addr == (UINT64_C(1) << 32) * sector_size);
        CHECK(disk.size == (bd_size_t)counts[i] * sector_size);
        CHECK(disk_write(0, buffer, UINT64_C(1) << 32, counts[i]) == RES_OK);
        CHECK(disk.addr == (UINT64_C(1) << 32) * sector_size);
        CHECK(disk.size == (bd_size_t)counts[i] * sector_size);
    }

    // Both accepting and rejecting backends must see empty transfers, including
    // at the device end. Write rejection comes from program, after no-op erase.
    const LBA_t empty_starts[] = {0, 1, count};
    for (unsigned reject = 0; reject < 2; ++reject) {
        disk.reject_empty = reject;
        DRESULT expected = reject ? RES_PARERR : RES_OK;
        for (size_t i = 0; i < sizeof(empty_starts) / sizeof(empty_starts[0]); ++i) {
            unsigned reads = disk.reads, erases = disk.erases, programs = disk.programs;
            CHECK(disk_read(0, buffer, empty_starts[i], 0) == expected);
            CHECK(disk.reads == reads + 1);
            CHECK(disk.addr == empty_starts[i] * sector_size && disk.size == 0);
            CHECK(disk_write(0, buffer, empty_starts[i], 0) == expected);
            CHECK(disk.erases == erases + 1 && disk.programs == programs + 1);
            CHECK(disk.addr == empty_starts[i] * sector_size && disk.size == 0);
        }
    }
    disk.reject_empty = false;

    // Reject out-of-bounds and byte-overflowing requests before callbacks.
    unsigned reads = disk.reads, erases = disk.erases, programs = disk.programs;
    const LBA_t starts[] = {count, count - 1, count + 1, UINT64_MAX / sector_size + 1, UINT64_MAX};
    const UINT lengths[] = {1, 2, 0, 1, 1};
    for (size_t i = 0; i < sizeof(starts) / sizeof(starts[0]); ++i) {
        CHECK(disk_read(0, buffer, starts[i], lengths[i]) == RES_PARERR);
        CHECK(disk_write(0, buffer, starts[i], lengths[i]) == RES_PARERR);
    }
    CHECK(disk.reads == reads && disk.erases == erases && disk.programs == programs);

    unsigned trims = disk.trims;
    LBA_t invalid_ranges[][2] = {
        {2, 1}, {0, count}, {count, count},
        {0, UINT64_MAX}, {1, UINT64_MAX}, {UINT64_MAX, UINT64_MAX},
    };
    for (size_t i = 0; i < sizeof(invalid_ranges) / sizeof(invalid_ranges[0]); ++i) {
        CHECK(disk_ioctl(0, CTRL_TRIM, invalid_ranges[i]) == RES_PARERR);
    }
    CHECK(disk.trims == trims);

    CHECK(fs->unmount(fs) == 0);
    filesystem_fat_free(fs);
}

/* A failed open must release its private handle without requiring file_close. */
static void test_failed_open_cleanup(void) {
    blockdevice_t *heap = blockdevice_heap_create(128u * 1024u);
    filesystem_t *fs = filesystem_fat_create();
    CHECK(heap != NULL && fs != NULL);
    CHECK(fs->format(fs, heap) == 0);
    CHECK(fs->mount(fs, heap, false) == 0);
    for (unsigned i = 0; i < 3; ++i) {
        fs_file_t file = {0};
        CHECK(fs->file_open(fs, &file, "/missing.bin", O_RDONLY) == -ENOENT);
        CHECK(file.context == NULL);
    }
    fs_file_t file = {0};
    CHECK(fs->file_open(fs, &file, "/created.bin", O_CREAT | O_RDWR) == 0);
    CHECK(fs->file_close(fs, &file) == 0);
    CHECK(fs->unmount(fs) == 0);
    filesystem_fat_free(fs);
    blockdevice_heap_free(heap);
}

/* Exclusive creation must reject existing files even with truncate or append. */
static void test_exclusive_create(void) {
    blockdevice_t *heap = blockdevice_heap_create(128u * 1024u);
    filesystem_t *fs = filesystem_fat_create();
    CHECK(heap != NULL && fs != NULL);
    CHECK(fs->format(fs, heap) == 0);
    CHECK(fs->mount(fs, heap, false) == 0);
    const int flags[] = {
        O_CREAT | O_EXCL | O_RDWR,
        O_CREAT | O_EXCL | O_RDWR | O_TRUNC,
        O_CREAT | O_EXCL | O_RDWR | O_APPEND,
    };
    const char payload[] = "exclusive creation preserves existing data";
    for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); ++i) {
        fs_file_t file = {0};
        CHECK(fs->file_open(fs, &file, "/exclusive.bin", flags[i]) == 0);
        CHECK(fs->file_write(fs, &file, payload, sizeof(payload)) == sizeof(payload));
        fs_file_t duplicate = {0};
        int err = fs->file_open(fs, &duplicate, "/exclusive.bin", flags[i]);
        if (err == 0) {
            CHECK(fs->file_close(fs, &duplicate) == 0);
        }
        CHECK(err == -EEXIST);
        CHECK(duplicate.context == NULL);
        CHECK(fs->file_close(fs, &file) == 0);
        CHECK(fs->file_open(fs, &file, "/exclusive.bin", O_RDONLY) == 0);
        CHECK(fs->file_size(fs, &file) == sizeof(payload));
        char actual[sizeof(payload)] = {0};
        CHECK(fs->file_read(fs, &file, actual, sizeof(actual)) == sizeof(actual));
        CHECK(memcmp(actual, payload, sizeof(payload)) == 0);
        CHECK(fs->file_close(fs, &file) == 0);
        CHECK(fs->remove(fs, "/exclusive.bin") == 0);
    }
    CHECK(fs->unmount(fs) == 0);
    filesystem_fat_free(fs);
    blockdevice_heap_free(heap);
}

void mutex_init(mutex_t *mutex) {
    mutex->locked = false;
}

void mutex_enter_blocking(mutex_t *mutex) {
    CHECK(!mutex->locked);
    mutex->locked = true;
}

void mutex_exit(mutex_t *mutex) {
    CHECK(mutex->locked);
    mutex->locked = false;
}

int main(void) {
    test_failed_open_cleanup();
    test_exclusive_create();
    const size_t sector_sizes[] = {512, 4096};
    for (size_t i = 0; i < sizeof(sector_sizes) / sizeof(sector_sizes[0]); ++i) {
        test_fat_disk(sector_sizes[i]);
        printf("fat-disk-%zu-byte-sectors: passed\n", sector_sizes[i]);
    }
    return EXIT_SUCCESS;
}
