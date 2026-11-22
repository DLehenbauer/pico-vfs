#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#undef PATH_MAX
#include "blockdevice/heap.h"
#include "filesystem/fat.h"
#include "pico/mutex.h"
#include "ff.h"
#include "diskio.h"

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);     \
            exit(EXIT_FAILURE);                                                 \
        }                                                                       \
    } while (0)

#define TEST_OFF_MAX ((off_t)(((uintmax_t)1 <<                    \
                               (sizeof(off_t) * CHAR_BIT - 1)) - \
                              1))

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

/** Create a small, freshly formatted real FatFs volume for lifecycle regressions. */
static filesystem_t *create_heap_filesystem(blockdevice_t **out_heap) {
    blockdevice_t *heap = blockdevice_heap_create(128u * 1024u);
    filesystem_t *fs = filesystem_fat_create();
    CHECK(heap != NULL && fs != NULL && out_heap != NULL);
    CHECK(fs->format(fs, heap) == 0);
    CHECK(fs->mount(fs, heap, false) == 0);
    *out_heap = heap;
    return fs;
}

/** Unmount a lifecycle test's volume before releasing its filesystem and storage. */
static void destroy_heap_filesystem(filesystem_t *fs, blockdevice_t *heap) {
    CHECK(fs->unmount(fs) == 0);
    filesystem_fat_free(fs);
    blockdevice_heap_free(heap);
}

/** Query volume zero's free-cluster count for capacity and allocation assertions. */
static DWORD free_clusters(void) {
    FATFS *fat = NULL;
    DWORD count = 0;
    CHECK(f_getfree("0:", &count, &fat) == FR_OK);
    return count;
}

/** Verify the expected contents of a half-open byte range in a read buffer. */
static void check_bytes(const char *buffer, size_t start, size_t end, char value) {
    for (size_t i = start; i < end; ++i) {
        CHECK(buffer[i] == value);
    }
}

/** Check directory-metadata size independently of an open file's cached size. */
static void check_stat_size(filesystem_t *fs, const char *path, off_t expected) {
    struct stat info = {0};
    CHECK(fs->stat(fs, path, &info) == 0);
    CHECK(info.st_size == expected);
}

/** Verify immediate truncation metadata and cursor preservation, including beyond EOF. */
static void test_truncate_metadata_and_cursor(void) {
    blockdevice_t *heap;
    filesystem_t *fs = create_heap_filesystem(&heap);
    fs_file_t file = {0};
    CHECK(fs->file_open(fs, &file, "/truncate.bin", O_CREAT | O_RDWR) == 0);
    char payload[4099];
    memset(payload, 0x5a, sizeof(payload));
    CHECK(fs->file_write(fs, &file, payload, sizeof(payload)) ==
          (ssize_t)sizeof(payload));

    // Shrink while cursor remains at old EOF (4099 > 513).
    CHECK(fs->file_truncate(fs, &file, 513) == 0);
    CHECK(fs->file_tell(fs, &file) == (off_t)sizeof(payload));
    CHECK(fs->file_size(fs, &file) == 513);
    check_stat_size(fs, "/truncate.bin", 513);
    char read_buffer[sizeof(payload)];
    CHECK(fs->file_read(fs, &file, read_buffer, sizeof(read_buffer)) == 0);
    CHECK(fs->file_tell(fs, &file) == (off_t)sizeof(payload));
    CHECK(fs->file_close(fs, &file) == 0);

    // Closing with an overhanging cursor must preserve the truncated size.
    check_stat_size(fs, "/truncate.bin", 513);
    CHECK(fs->file_open(fs, &file, "/truncate.bin", O_RDONLY) == 0);
    CHECK(fs->file_read(fs, &file, read_buffer, sizeof(read_buffer)) == 513);
    check_bytes(read_buffer, 0, 513, 0x5a);
    CHECK(fs->file_close(fs, &file) == 0);

    // Extend and truncate to zero with a nonzero cursor while open.
    CHECK(fs->file_open(fs, &file, "/truncate.bin", O_RDWR) == 0);
    CHECK(fs->file_seek(fs, &file, 200, SEEK_SET) == 200);
    CHECK(fs->file_truncate(fs, &file, 2053) == 0);
    CHECK(fs->file_tell(fs, &file) == 200);
    CHECK(fs->file_size(fs, &file) == 2053);
    check_stat_size(fs, "/truncate.bin", 2053);

    CHECK(fs->file_truncate(fs, &file, 0) == 0);
    CHECK(fs->file_tell(fs, &file) == 200);
    CHECK(fs->file_size(fs, &file) == 0);
    check_stat_size(fs, "/truncate.bin", 0);
    CHECK(fs->file_read(fs, &file, read_buffer, 1) == 0);
    CHECK(fs->file_close(fs, &file) == 0);
    check_stat_size(fs, "/truncate.bin", 0);
    destroy_heap_filesystem(fs, heap);
}

/** Verify deferred seeks, later gap writes, and access checks even at or beyond EOF. */
static void test_cursor_and_access_modes(void) {
    blockdevice_t *heap;
    filesystem_t *fs = create_heap_filesystem(&heap);
    fs_file_t file = {0};
    CHECK(fs->file_open(fs, &file, "/cursor.bin", O_CREAT | O_RDWR) == 0);
    char initial[513];
    memset(initial, 0x5a, sizeof(initial));
    CHECK(fs->file_write(fs, &file, initial, sizeof(initial)) ==
          (ssize_t)sizeof(initial));

    // Moving the logical cursor, reading EOF, or writing zero bytes must not allocate.
    CHECK(fs->file_seek(fs, &file, 8192, SEEK_SET) == 8192);
    CHECK(fs->file_tell(fs, &file) == 8192);
    CHECK(fs->file_size(fs, &file) == (off_t)sizeof(initial));
    check_stat_size(fs, "/cursor.bin", (off_t)sizeof(initial));
    CHECK(fs->file_seek(fs, &file, -100, SEEK_CUR) == 8092);
    CHECK(fs->file_seek(fs, &file, 100, SEEK_CUR) == 8192);
    CHECK(fs->file_seek(fs, &file, 100, SEEK_END) == 613);
    CHECK(fs->file_seek(fs, &file, -513, SEEK_END) == 0);
    CHECK(fs->file_seek(fs, &file, 8192, SEEK_SET) == 8192);
    CHECK(fs->file_read(fs, &file, initial, 1) == 0);
    CHECK(fs->file_write(fs, &file, initial, 0) == 0);
    CHECK(fs->file_tell(fs, &file) == 8192);
    CHECK(fs->file_size(fs, &file) == (off_t)sizeof(initial));
    check_stat_size(fs, "/cursor.bin", (off_t)sizeof(initial));
    CHECK(fs->file_close(fs, &file) == 0);
    check_stat_size(fs, "/cursor.bin", (off_t)sizeof(initial));

    // Only the later payload write grows the file to the overhanging cursor.
    // Check preserved data and payload, not the unspecified contents of the gap.
    CHECK(fs->file_open(fs, &file, "/cursor.bin", O_RDWR) == 0);
    CHECK(fs->file_seek(fs, &file, 3000, SEEK_SET) == 3000);
    CHECK(fs->file_truncate(fs, &file, 1200) == 0);
    CHECK(fs->file_tell(fs, &file) == 3000);
    CHECK(fs->file_size(fs, &file) == 1200);
    check_stat_size(fs, "/cursor.bin", 1200);
    CHECK(fs->file_write(fs, &file, "TAIL", 4) == 4);
    CHECK(fs->file_tell(fs, &file) == 3004);
    CHECK(fs->file_size(fs, &file) == 3004);
    check_stat_size(fs, "/cursor.bin", 3004);
    CHECK(fs->file_seek(fs, &file, 0, SEEK_SET) == 0);
    char grown[3004];
    CHECK(fs->file_read(fs, &file, grown, sizeof(grown)) ==
          (ssize_t)sizeof(grown));
    check_bytes(grown, 0, 513, 0x5a);
    CHECK(memcmp(grown + 3000, "TAIL", 4) == 0);
    CHECK(fs->file_close(fs, &file) == 0);

    // Access-mode errors take precedence over otherwise valid EOF behavior.
    CHECK(fs->file_open(fs, &file, "/cursor.bin", O_RDONLY) == 0);
    CHECK(fs->file_seek(fs, &file, 5000, SEEK_SET) == 5000);
    CHECK(fs->file_tell(fs, &file) == 5000);
    CHECK(fs->file_read(fs, &file, grown, 1) == 0);
    CHECK(fs->file_write(fs, &file, "x", 1) == -EACCES);
    CHECK(fs->file_write(fs, &file, "x", 0) == -EACCES);
    CHECK(fs->file_truncate(fs, &file, 10) == -EINVAL);
    CHECK(fs->file_close(fs, &file) == 0);

    CHECK(fs->file_open(fs, &file, "/cursor.bin", O_WRONLY) == 0);
    CHECK(fs->file_read(fs, &file, grown, 1) == -EACCES);
    CHECK(fs->file_seek(fs, &file, 3004, SEEK_SET) == 3004);
    CHECK(fs->file_read(fs, &file, grown, 1) == -EACCES);
    CHECK(fs->file_close(fs, &file) == 0);
    destroy_heap_filesystem(fs, heap);
}

/** Verify every append chooses current EOF despite truncation or an explicit seek. */
static void test_append_after_truncate(void) {
    blockdevice_t *heap;
    filesystem_t *fs = create_heap_filesystem(&heap);
    fs_file_t file = {0};
    CHECK(fs->file_open(fs, &file, "/append.bin",
                        O_CREAT | O_RDWR | O_APPEND) == 0);
    CHECK(fs->file_tell(fs, &file) == 0);
    CHECK(fs->file_write(fs, &file, "0123456789", 10) == 10);
    // Truncation preserves cursor 10, but the append must start at the new EOF 4.
    CHECK(fs->file_truncate(fs, &file, 4) == 0);
    CHECK(fs->file_tell(fs, &file) == 10);
    CHECK(fs->file_write(fs, &file, "AB", 2) == 2);
    CHECK(fs->file_tell(fs, &file) == 6);
    CHECK(fs->file_size(fs, &file) == 6);
    check_stat_size(fs, "/append.bin", 6);

    // Seeking backward cannot turn an append write into an overwrite.
    CHECK(fs->file_seek(fs, &file, 0, SEEK_SET) == 0);
    CHECK(fs->file_write(fs, &file, "CD", 2) == 2);
    CHECK(fs->file_tell(fs, &file) == 8);
    CHECK(fs->file_size(fs, &file) == 8);
    check_stat_size(fs, "/append.bin", 8);
    // Reading from zero verifies both appends kept the original surviving prefix.
    CHECK(fs->file_seek(fs, &file, 0, SEEK_SET) == 0);
    char result[8];
    CHECK(fs->file_read(fs, &file, result, sizeof(result)) ==
          (ssize_t)sizeof(result));
    CHECK(memcmp(result, "0123ABCD", sizeof(result)) == 0);
    CHECK(fs->file_close(fs, &file) == 0);
    destroy_heap_filesystem(fs, heap);
}

/** Verify rejected ranges and oversized growth preserve state, then allow usable I/O. */
static void test_bounds_and_capacity_precheck(void) {
    blockdevice_t *heap;
    filesystem_t *fs = create_heap_filesystem(&heap);
    fs_file_t file = {0};
    CHECK(fs->file_open(fs, &file, "/nospc.bin", O_CREAT | O_RDWR) == 0);
    char original[513];
    memset(original, 0x7e, sizeof(original));
    CHECK(fs->file_write(fs, &file, original, sizeof(original)) ==
          (ssize_t)sizeof(original));
    CHECK(fs->file_seek(fs, &file, 200, SEEK_SET) == 200);

    // Exercise signed boundaries without changing size, metadata, or the cursor.
    const off_t off_min = -TEST_OFF_MAX - 1;
    CHECK(fs->file_truncate(fs, &file, -1) == -EINVAL);
    CHECK(fs->file_truncate(fs, &file, off_min) == -EINVAL);
    CHECK(fs->file_tell(fs, &file) == 200);
    CHECK(fs->file_size(fs, &file) == (off_t)sizeof(original));
    check_stat_size(fs, "/nospc.bin", (off_t)sizeof(original));

    CHECK(fs->file_seek(fs, &file, -1, SEEK_SET) == -EINVAL);
    CHECK(fs->file_seek(fs, &file, off_min, SEEK_SET) == -EINVAL);
    CHECK(fs->file_seek(fs, &file, -201, SEEK_CUR) == -EINVAL);
    CHECK(fs->file_seek(fs, &file, off_min, SEEK_CUR) == -EINVAL);
    CHECK(fs->file_seek(fs, &file, -514, SEEK_END) == -EINVAL);
    CHECK(fs->file_seek(fs, &file, 0, 99) == -EINVAL);
    CHECK(fs->file_tell(fs, &file) == 200);
    CHECK(fs->file_seek(fs, &file, TEST_OFF_MAX, SEEK_SET) == TEST_OFF_MAX);
    CHECK(fs->file_seek(fs, &file, 1, SEEK_CUR) == -EOVERFLOW);
    CHECK(fs->file_tell(fs, &file) == TEST_OFF_MAX);

    if (sizeof(off_t) > sizeof(uint32_t)) {
        // On wider hosts, distinguish a valid logical seek from FAT's 32-bit size cap.
        const off_t too_large = (off_t)((uintmax_t)UINT32_MAX + 1);
        CHECK(fs->file_seek(fs, &file, 200, SEEK_SET) == 200);
        CHECK(fs->file_truncate(fs, &file, too_large) == -EFBIG);
        CHECK(fs->file_size(fs, &file) == (off_t)sizeof(original));
        CHECK(fs->file_seek(fs, &file, too_large, SEEK_SET) == too_large);
        CHECK(fs->file_write(fs, &file, "X", 1) == -EFBIG);
        CHECK(fs->file_size(fs, &file) == (off_t)sizeof(original));
    }

    // Growth larger than the entire volume must fail before consuming clusters.
    const DWORD before = free_clusters();
    CHECK(fs->file_seek(fs, &file, 200, SEEK_SET) == 200);
    CHECK(fs->file_truncate(fs, &file, 256u * 1024u) == -ENOSPC);
    CHECK(fs->file_tell(fs, &file) == 200);
    CHECK(fs->file_size(fs, &file) == (off_t)sizeof(original));
    check_stat_size(fs, "/nospc.bin", (off_t)sizeof(original));
    CHECK(free_clusters() == before);

    CHECK(fs->file_seek(fs, &file, 256u * 1024u, SEEK_SET) == 256u * 1024u);
    CHECK(fs->file_write(fs, &file, "Z", 1) == -ENOSPC);
    CHECK(fs->file_tell(fs, &file) == 256u * 1024u);
    CHECK(fs->file_size(fs, &file) == (off_t)sizeof(original));
    check_stat_size(fs, "/nospc.bin", (off_t)sizeof(original));
    CHECK(free_clusters() == before);

    // Reuse the same handle after refusal to show the precheck did not poison it.
    CHECK(fs->file_truncate(fs, &file, 2048) == 0);
    CHECK(fs->file_seek(fs, &file, 2048, SEEK_SET) == 2048);
    CHECK(fs->file_write(fs, &file, "OK", 2) == 2);
    CHECK(fs->file_size(fs, &file) == 2050);
    check_stat_size(fs, "/nospc.bin", 2050);
    CHECK(fs->file_close(fs, &file) == 0);

    // Ordinary writes may fill the remaining volume, but exhaustion must return
    // either a positive short count or ENOSPC, never a misleading zero success.
    CHECK(fs->file_open(fs, &file, "/fill.bin", O_CREAT | O_RDWR) == 0);
    char block[1024];
    memset(block, 0xa5, sizeof(block));
    ssize_t written;
    do {
        written = fs->file_write(fs, &file, block, sizeof(block));
        CHECK(written != 0);
        CHECK(written > 0 || written == -ENOSPC);
    } while (written >= 0);
    CHECK(written == -ENOSPC);
    CHECK(fs->file_close(fs, &file) == 0);
    destroy_heap_filesystem(fs, heap);
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

/** Run real-volume lifecycle regressions and storage-free sector-range tests. */
int main(void) {
    test_failed_open_cleanup();
    test_truncate_metadata_and_cursor();
    test_cursor_and_access_modes();
    test_append_after_truncate();
    test_bounds_and_capacity_precheck();
    // Keep adapter arithmetic coverage independent of the heap volume's geometry.
    const size_t sector_sizes[] = {512, 4096};
    for (size_t i = 0; i < sizeof(sector_sizes) / sizeof(sector_sizes[0]); ++i) {
        test_fat_disk(sector_sizes[i]);
        printf("fat-disk-%zu-byte-sectors: passed\n", sector_sizes[i]);
    }
    return EXIT_SUCCESS;
}
