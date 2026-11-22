#include <stdio.h>
#include <string.h>

#include "blockdevice/heap.h"
#include "filesystem/fat.h"
#include "pico/mutex.h"
#include "ff.h"
#include "diskio.h"

#define FAT12_BPB_RESERVED_SECTORS_OFFSET 14
#define FAT12_BPB_FAT_COUNT_OFFSET       16
#define FAT12_BPB_ROOT_ENTRIES_OFFSET    17
#define FAT12_BPB_FAT_SIZE_OFFSET        22
#define FAT_DIRECTORY_ENTRY_SIZE         32
#define FAT_DIRECTORY_ENTRIES_PER_SECTOR 16
#define FAT_SHORT_NAME_SIZE              11
#define FAT_ATTRIBUTE_OFFSET             11
#define FAT_LONG_NAME_ATTRIBUTE          0x0f
#define FAT12_ROOT_ENTRIES               512

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);       \
            exit(EXIT_FAILURE);                                                 \
        }                                                                       \
    } while (0)

static FRESULT stat_failure, opendir_failure, closedir_failure;
static unsigned stat_probes, opendir_probes, closedir_probes;

FRESULT __real_f_stat(const TCHAR *path, FILINFO *info);
FRESULT __real_f_opendir(FATFS_DIR *dir, const TCHAR *path);
FRESULT __real_f_closedir(FATFS_DIR *dir);

FRESULT __wrap_f_stat(const TCHAR *path, FILINFO *info) {
    ++stat_probes;
    return stat_failure != FR_OK ? stat_failure : __real_f_stat(path, info);
}

FRESULT __wrap_f_opendir(FATFS_DIR *dir, const TCHAR *path) {
    ++opendir_probes;
    return opendir_failure != FR_OK ? opendir_failure : __real_f_opendir(dir, path);
}

FRESULT __wrap_f_closedir(FATFS_DIR *dir) {
    ++closedir_probes;
    return closedir_failure != FR_OK ? closedir_failure : __real_f_closedir(dir);
}

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

static void expect_open_error(filesystem_t *fs, const char *path, int flags, int expected) {
    fs_file_t file = {0};
    const int actual = fs->file_open(fs, &file, path, flags);
    if (actual != expected) {
        fprintf(stderr, "open('%s', 0x%x): expected %d, got %d\n",
                path, flags, expected, actual);
    }
    CHECK(actual == expected);
    CHECK(file.context == NULL);
}

static void verify_file(filesystem_t *fs, const char *path,
                        const void *payload, size_t size) {
    fs_file_t file = {0};
    uint8_t actual[256] = {0};
    CHECK(size <= sizeof(actual));
    CHECK(fs->file_open(fs, &file, path, O_RDONLY) == 0);
    CHECK(fs->file_size(fs, &file) == (off_t)size);
    CHECK(fs->file_read(fs, &file, actual, size) == (ssize_t)size);
    CHECK(memcmp(actual, payload, size) == 0);
    CHECK(fs->file_close(fs, &file) == 0);
}

static void format_and_mount(filesystem_t *fs, blockdevice_t *heap) {
    CHECK(fs->format(fs, heap) == 0);
    CHECK(fs->mount(fs, heap, false) == 0);
}

static void mark_file_read_only(blockdevice_t *heap) {
    static const char short_name[] = "RDOFILE BIN";
    uint8_t sector[512] = {0};
    CHECK(heap->read(heap, sector, 0, sizeof(sector)) == 0);
    const unsigned reserved_sectors = sector[FAT12_BPB_RESERVED_SECTORS_OFFSET]
        | (sector[FAT12_BPB_RESERVED_SECTORS_OFFSET + 1] << 8);
    const unsigned root_entries = sector[FAT12_BPB_ROOT_ENTRIES_OFFSET]
        | (sector[FAT12_BPB_ROOT_ENTRIES_OFFSET + 1] << 8);
    const unsigned fats = sector[FAT12_BPB_FAT_COUNT_OFFSET];
    const unsigned fat_sectors = sector[FAT12_BPB_FAT_SIZE_OFFSET]
        | (sector[FAT12_BPB_FAT_SIZE_OFFSET + 1] << 8);
    const bd_size_t root_sector = reserved_sectors + fats * fat_sectors;
    const unsigned root_sector_count = root_entries / FAT_DIRECTORY_ENTRIES_PER_SECTOR;
    unsigned matches = 0;

    for (unsigned i = 0; i < root_sector_count; ++i) {
        const bd_size_t address = (root_sector + i) * sizeof(sector);
        CHECK(heap->read(heap, sector, address, sizeof(sector)) == 0);
        for (unsigned entry = 0; entry < FAT_DIRECTORY_ENTRIES_PER_SECTOR; ++entry) {
            uint8_t *record = sector + entry * FAT_DIRECTORY_ENTRY_SIZE;
            if (memcmp(record, short_name, FAT_SHORT_NAME_SIZE) == 0
                    && record[FAT_ATTRIBUTE_OFFSET] != FAT_LONG_NAME_ATTRIBUTE) {
                record[FAT_ATTRIBUTE_OFFSET] |= AM_RDO;
                CHECK(heap->program(heap, sector, address, sizeof(sector)) == 0);
                ++matches;
            }
        }
    }
    CHECK(matches == 1);
}

/* Exclusive creation must reject existing files even with truncate or append. */
static void test_exclusive_create(void) {
    static const char path[] = "/exclusive.bin";
    static const char payload[] = "exclusive creation preserves existing data";
    static const int access_modes[] = {O_RDONLY, O_WRONLY, O_RDWR};
    static const int variants[] = {0, O_TRUNC, O_APPEND, O_TRUNC | O_APPEND};
    static const char *const directory_paths[] = {"/dir", "/", "/.", "/dir/.."};
    blockdevice_t *heap = blockdevice_heap_create(128u * 1024u);
    filesystem_t *fs = filesystem_fat_create();
    CHECK(heap != NULL && fs != NULL);
    format_and_mount(fs, heap);

    for (size_t access = 0; access < sizeof(access_modes) / sizeof(access_modes[0]); ++access) {
        for (size_t variant = 0; variant < sizeof(variants) / sizeof(variants[0]); ++variant) {
            const int flags = O_CREAT | O_EXCL | access_modes[access] | variants[variant];
            fs_file_t file = {0};
            CHECK(fs->file_open(fs, &file, path, flags) == 0);
            CHECK(fs->file_tell(fs, &file) == 0);
            CHECK(fs->file_size(fs, &file) == 0);

            if (access_modes[access] == O_WRONLY || access_modes[access] == O_RDWR) {
                CHECK(fs->file_write(fs, &file, payload, sizeof(payload)) == (ssize_t)sizeof(payload));
            }
            expect_open_error(fs, path, flags, -EEXIST);
            CHECK(fs->file_close(fs, &file) == 0);

            if (access_modes[access] == O_RDONLY) {
                CHECK(fs->file_open(fs, &file, path, O_WRONLY) == 0);
                CHECK(fs->file_write(fs, &file, payload, sizeof(payload)) == (ssize_t)sizeof(payload));
                CHECK(fs->file_close(fs, &file) == 0);
            }
            expect_open_error(fs, path, flags, -EEXIST);
            verify_file(fs, path, payload, sizeof(payload));
            CHECK(fs->remove(fs, path) == 0);
        }
    }

    expect_open_error(fs, "/not-created.bin", O_RDWR | O_EXCL, -ENOENT);
    fs_file_t file = {0};
    CHECK(fs->file_open(fs, &file, "/not-created.bin", O_CREAT | O_RDWR) == 0);
    CHECK(fs->file_close(fs, &file) == 0);
    CHECK(fs->file_open(fs, &file, "/not-created.bin", O_RDWR | O_EXCL) == 0);
    CHECK(fs->file_close(fs, &file) == 0);
    CHECK(fs->remove(fs, "/not-created.bin") == 0);

    CHECK(fs->mkdir(fs, "/dir", 0777) == 0);
    for (size_t i = 0; i < sizeof(directory_paths) / sizeof(directory_paths[0]); ++i) {
        expect_open_error(fs, directory_paths[i], O_CREAT | O_EXCL | O_RDONLY, -EEXIST);
        expect_open_error(fs, directory_paths[i], O_CREAT | O_EXCL | O_WRONLY, -EEXIST);
        expect_open_error(fs, directory_paths[i], O_CREAT | O_EXCL | O_RDWR, -EEXIST);
        expect_open_error(fs, directory_paths[i],
                          O_CREAT | O_EXCL | O_RDWR | O_TRUNC, -EEXIST);
    }
    struct stat st = {0};
    CHECK(fs->stat(fs, "/dir", &st) == 0);
    CHECK(S_ISDIR(st.st_mode));
    CHECK(fs->file_open(fs, &file, "/dir/inside.bin", O_CREAT | O_EXCL | O_RDWR) == 0);
    CHECK(fs->file_close(fs, &file) == 0);
    CHECK(fs->remove(fs, "/dir/inside.bin") == 0);
    CHECK(fs->rmdir(fs, "/dir") == 0);

    expect_open_error(fs, "", O_CREAT | O_EXCL | O_RDWR, -EINVAL);
    expect_open_error(fs, "/bad*name", O_CREAT | O_EXCL | O_RDWR, -EINVAL);
    expect_open_error(fs, "/missing/file.bin", O_CREAT | O_EXCL | O_RDWR, -ENOTDIR);

    CHECK(fs->unmount(fs) == 0);
    format_and_mount(fs, heap);
    CHECK(fs->file_open(fs, &file, "/RDOFILE.BIN", O_CREAT | O_RDWR) == 0);
    CHECK(fs->file_write(fs, &file, payload, sizeof(payload)) == (ssize_t)sizeof(payload));
    CHECK(fs->file_close(fs, &file) == 0);
    CHECK(fs->unmount(fs) == 0);
    mark_file_read_only(heap);
    CHECK(fs->mount(fs, heap, false) == 0);
    CHECK(fs->stat(fs, "/RDOFILE.BIN", &st) == 0);
    CHECK((st.st_mode & S_IWUSR) == 0);
    expect_open_error(fs, "/RDOFILE.BIN", O_WRONLY, -EACCES);
    expect_open_error(fs, "/RDOFILE.BIN", O_WRONLY | O_CREAT, -EACCES);
    expect_open_error(fs, "/RDOFILE.BIN", O_CREAT | O_EXCL | O_RDONLY, -EEXIST);
    expect_open_error(fs, "/RDOFILE.BIN", O_CREAT | O_EXCL | O_WRONLY, -EEXIST);
    expect_open_error(fs, "/RDOFILE.BIN", O_CREAT | O_EXCL | O_RDWR, -EEXIST);
    expect_open_error(fs, "/RDOFILE.BIN",
                      O_CREAT | O_EXCL | O_RDWR | O_TRUNC, -EEXIST);
    verify_file(fs, "/RDOFILE.BIN", payload, sizeof(payload));

    CHECK(fs->unmount(fs) == 0);
    format_and_mount(fs, heap);
    // Uppercase 8.3 names occupy exactly one entry in the default fixed root.
    for (unsigned i = 0; i < FAT12_ROOT_ENTRIES; ++i) {
        char short_name[16] = {0};
        CHECK(snprintf(short_name, sizeof(short_name), "/S%05u.BIN", i) > 0);
        CHECK(fs->file_open(fs, &file, short_name, O_CREAT | O_EXCL | O_RDWR) == 0);
        CHECK(fs->file_close(fs, &file) == 0);
    }
    expect_open_error(fs, "/NEWEXCL.BIN", O_CREAT | O_EXCL | O_RDWR, -EACCES);
    expect_open_error(fs, "/S00000.BIN", O_CREAT | O_EXCL | O_RDWR, -EEXIST);

    CHECK(fs->unmount(fs) == 0);
    filesystem_fat_free(fs);
    blockdevice_heap_free(heap);
}

static void verify_recovery(filesystem_t *fs) {
    fs_file_t file = {0};
    const char payload[] = "after probe failure";
    CHECK(fs->file_open(fs, &file, "/recovery.bin", O_CREAT | O_EXCL | O_RDWR) == 0);
    CHECK(fs->file_write(fs, &file, payload, sizeof(payload)) == (ssize_t)sizeof(payload));
    CHECK(fs->file_close(fs, &file) == 0);
    verify_file(fs, "/recovery.bin", payload, sizeof(payload));
    CHECK(fs->remove(fs, "/recovery.bin") == 0);
}

static void test_exclusive_probe_failures(void) {
    blockdevice_t *heap = blockdevice_heap_create(128u * 1024u);
    filesystem_t *fs = filesystem_fat_create();
    CHECK(heap != NULL && fs != NULL);
    format_and_mount(fs, heap);
    CHECK(fs->mkdir(fs, "/dir", 0777) == 0);
    const int flags = O_CREAT | O_EXCL | O_RDWR;

    unsigned probes = stat_probes;
    stat_failure = FR_DISK_ERR;
    expect_open_error(fs, "/dir", flags, -EACCES);
    CHECK(stat_probes == probes + 1);
    stat_failure = FR_OK;
    expect_open_error(fs, "/dir", flags, -EEXIST);
    verify_recovery(fs);

    probes = opendir_probes;
    opendir_failure = FR_NOT_ENOUGH_CORE;
    expect_open_error(fs, "/", flags, -EINVAL);
    CHECK(opendir_probes == probes + 1);
    opendir_failure = FR_OK;
    expect_open_error(fs, "/", flags, -EEXIST);
    verify_recovery(fs);

    probes = closedir_probes;
    closedir_failure = FR_DISK_ERR;
    expect_open_error(fs, "/", flags, -EEXIST);
    CHECK(closedir_probes == probes + 1);
    closedir_failure = FR_OK;
    expect_open_error(fs, "/", flags, -EEXIST);
    verify_recovery(fs);

    CHECK(fs->rmdir(fs, "/dir") == 0);
    CHECK(fs->unmount(fs) == 0);
    filesystem_fat_free(fs);
    blockdevice_heap_free(heap);
}

static void test_fat32_second_volume(void) {
    blockdevice_t *heaps[] = {
        blockdevice_heap_create(128u * 1024u),
        blockdevice_heap_create(64u * 1024u * 1024u),
    };
    filesystem_t *filesystems[] = {filesystem_fat_create(), filesystem_fat_create()};
    CHECK(heaps[0] != NULL && heaps[1] != NULL);
    CHECK(filesystems[0] != NULL && filesystems[1] != NULL);
    format_and_mount(filesystems[0], heaps[0]);
    DWORD free_clusters = 0;
    FATFS *volume = NULL;
    CHECK(f_getfree("0:", &free_clusters, &volume) == FR_OK);
    CHECK(volume->fs_type == FS_FAT12);

    // Force a real FAT32 image on drive 1 while drive 0 remains mounted.
    CHECK(filesystems[1]->mount(filesystems[1], heaps[1], true) == 0);
    const MKFS_PARM options = {.fmt = FM_FAT32 | FM_SFD, .au_size = 512};
    uint8_t work[FF_MAX_SS] = {0};
    CHECK(f_mkfs("1:", &options, work, sizeof(work)) == FR_OK);
    CHECK(filesystems[1]->unmount(filesystems[1]) == 0);
    CHECK(filesystems[1]->mount(filesystems[1], heaps[1], false) == 0);
    CHECK(f_getfree("1:", &free_clusters, &volume) == FR_OK);
    CHECK(volume->fs_type == FS_FAT32);

    const char *const payloads[] = {"drive zero", "drive one"};
    const int flags = O_CREAT | O_EXCL | O_RDWR | O_TRUNC | O_APPEND;
    for (size_t i = 0; i < 2; ++i) {
        filesystem_t *fs = filesystems[i];
        fs_file_t file = {0};
        CHECK(fs->file_open(fs, &file, "/same.bin", flags) == 0);
        CHECK(fs->file_tell(fs, &file) == 0);
        CHECK(fs->file_write(fs, &file, payloads[i], strlen(payloads[i]))
              == (ssize_t)strlen(payloads[i]));
        expect_open_error(fs, "/same.bin", flags, -EEXIST);
        CHECK(fs->file_close(fs, &file) == 0);
        expect_open_error(fs, "/same.bin", flags, -EEXIST);
        CHECK(fs->mkdir(fs, "/dir", 0777) == 0);
        expect_open_error(fs, "/dir", flags, -EEXIST);
        expect_open_error(fs, "/", flags, -EEXIST);
        expect_open_error(fs, "/.", flags, -EEXIST);
        expect_open_error(fs, "/dir/..", flags, -EEXIST);
        expect_open_error(fs, "", flags, -EINVAL);
        verify_recovery(fs);
    }
    for (size_t i = 0; i < 2; ++i) {
        filesystem_t *fs = filesystems[i];
        verify_file(fs, "/same.bin", payloads[i], strlen(payloads[i]));
        CHECK(fs->remove(fs, "/same.bin") == 0);
        CHECK(fs->rmdir(fs, "/dir") == 0);
        CHECK(fs->unmount(fs) == 0);
        filesystem_fat_free(fs);
        blockdevice_heap_free(heaps[i]);
    }
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
    test_exclusive_probe_failures();
    test_fat32_second_volume();
    const size_t sector_sizes[] = {512, 4096};
    for (size_t i = 0; i < sizeof(sector_sizes) / sizeof(sector_sizes[0]); ++i) {
        test_fat_disk(sector_sizes[i]);
        printf("fat-disk-%zu-byte-sectors: passed\n", sector_sizes[i]);
    }
    return EXIT_SUCCESS;
}
