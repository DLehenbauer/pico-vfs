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
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);     \
            exit(EXIT_FAILURE);                                                 \
        }                                                                       \
    } while (0)

#define TEST_OFF_MAX ((off_t)(((uintmax_t)1 <<                    \
                               (sizeof(off_t) * CHAR_BIT - 1)) - \
                              1))

static FRESULT stat_failure, opendir_failure, closedir_failure;
static unsigned stat_probes, opendir_probes, closedir_probes;
static int (*saved_program)(blockdevice_t *, const void *, bd_size_t, bd_size_t);
static unsigned program_calls, fail_program_at;
static bd_size_t fail_program_address;
static bool fail_at_address, program_failed;
static bool fail_followup_metadata, metadata_failed;
static FIL *boundary_file;
static unsigned boundary_writes;
static unsigned boundary_reads;

/** Fail one selected physical write, forwarding every other write to heap storage. */
static int failing_program(blockdevice_t *device, const void *buffer,
                           bd_size_t address, bd_size_t size) {
    ++program_calls;
    if (!program_failed && (fail_at_address ? address == fail_program_address
                                            : program_calls == fail_program_at)) {
        program_failed = true;
        return BD_ERROR_DEVICE_ERROR;
    }
    if (program_failed && fail_followup_metadata && !metadata_failed &&
        address == fail_program_address) {
        metadata_failed = true;
        return BD_ERROR_DEVICE_ERROR;
    }
    return saved_program(device, buffer, address, size);
}

FRESULT __real_f_stat(const TCHAR *path, FILINFO *info);
FRESULT __real_f_opendir(FATFS_DIR *dir, const TCHAR *path);
FRESULT __real_f_closedir(FATFS_DIR *dir);
FRESULT __real_f_write(FIL *file, const void *buffer, UINT size, UINT *written);
FRESULT __real_f_sync(FIL *file);
FRESULT __real_f_read(FIL *file, void *buffer, UINT size, UINT *read_bytes);
FRESULT __real_f_lseek(FIL *file, FSIZE_t offset);

/** Inject stat errors or expose synthetic large-file metadata for boundary tests. */
FRESULT __wrap_f_stat(const TCHAR *path, FILINFO *info) {
    ++stat_probes;
    if (boundary_file != NULL && info != NULL) {
        memset(info, 0, sizeof(*info));
        info->fsize = f_size(boundary_file);
        return FR_OK;
    }
    return stat_failure != FR_OK ? stat_failure : __real_f_stat(path, info);
}

/** Simulate enormous-file transfers only during storage-free adapter boundary tests. */
FRESULT __wrap_f_write(FIL *file, const void *buffer, UINT size, UINT *written) {
    if (boundary_file == NULL) return __real_f_write(file, buffer, size, written);
    CHECK(file == boundary_file);
    ++boundary_writes;
    *written = size;
    file->fptr += size;
    if (file->fptr > file->obj.objsize) file->obj.objsize = file->fptr;
    return FR_OK;
}

/** Bypass physical metadata I/O only for synthetic enormous files. */
FRESULT __wrap_f_sync(FIL *file) {
    if (boundary_file == NULL) return __real_f_sync(file);
    CHECK(file == boundary_file);
    return FR_OK;
}

/** Simulate reads from enormous files without allocating their backing data. */
FRESULT __wrap_f_read(FIL *file, void *buffer, UINT size, UINT *read_bytes) {
    if (boundary_file == NULL) return __real_f_read(file, buffer, size, read_bytes);
    CHECK(file == boundary_file);
    ++boundary_reads;
    const FSIZE_t avail = file->fptr >= file->obj.objsize ? 0 : file->obj.objsize - file->fptr;
    if ((FSIZE_t)size > avail) size = (UINT)avail;
    memset(buffer, 'R', size);
    *read_bytes = size;
    file->fptr += size;
    return FR_OK;
}

/** Position synthetic enormous files without traversing unallocated cluster chains. */
FRESULT __wrap_f_lseek(FIL *file, FSIZE_t offset) {
    if (boundary_file == NULL) return __real_f_lseek(file, offset);
    CHECK(file == boundary_file && offset <= file->obj.objsize);
    file->fptr = offset;
    return FR_OK;
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

/** Mount an explicitly formatted FAT/exFAT volume with the requested sector size. */
static filesystem_t *create_zero_fill_filesystem(blockdevice_t **out_heap,
                                                bool exfat, size_t sector_size) {
    blockdevice_t *heap = blockdevice_heap_create(32u * 1024u * 1024u);
    filesystem_t *fs = filesystem_fat_create();
    CHECK(heap != NULL && fs != NULL);
    heap->read_size = heap->program_size = heap->erase_size = sector_size;
    CHECK(fs->mount(fs, heap, true) == 0);
    const MKFS_PARM options = {
        .fmt = (exfat ? FM_EXFAT : FM_FAT) | FM_SFD, .au_size = sector_size,
    };
    uint8_t work[FF_MAX_SS];
    CHECK(f_mkfs("0:", &options, work, sizeof(work)) == FR_OK);
    CHECK(fs->unmount(fs) == 0);
    CHECK(fs->mount(fs, heap, false) == 0);
    *out_heap = heap;
    return fs;
}

/** Verify zero-filled growth and gaps on real FAT/exFAT volumes at both sector sizes. */
static void test_zero_fill(bool exfat, size_t sector_size) {
    blockdevice_t *heap;
    filesystem_t *fs = create_zero_fill_filesystem(&heap, exfat, sector_size);

    // Reuse dirty file contents so zeros cannot come from pristine heap storage.
    const size_t final_size = 4 * sector_size + 11;
    char *data = malloc(final_size);
    CHECK(data != NULL);
    memset(data, 'Q', final_size);
    fs_file_t file = {0};
    CHECK(fs->file_open(fs, &file, "/zero.bin", O_CREAT | O_RDWR) == 0);
    CHECK(fs->file_write(fs, &file, data, final_size) == (ssize_t)final_size);
    CHECK(fs->file_truncate(fs, &file, 37) == 0);
    const off_t extension = (off_t)(2 * sector_size + 11);
    CHECK(fs->file_truncate(fs, &file, extension) == 0);
    CHECK(fs->file_tell(fs, &file) == (off_t)final_size);
    check_stat_size(fs, "/zero.bin", extension);
    CHECK(fs->file_seek(fs, &file, 0, SEEK_SET) == 0);
    CHECK(fs->file_read(fs, &file, data, (size_t)extension) == extension);
    check_bytes(data, 0, 37, 'Q');
    check_bytes(data, 37, (size_t)extension, 0);

    // Growing an empty file and an unaligned gap must initialize every byte.
    CHECK(fs->file_truncate(fs, &file, 0) == 0);
    CHECK(fs->file_truncate(fs, &file, 31) == 0);
    CHECK(fs->file_tell(fs, &file) == extension);
    const off_t gap_end = (off_t)(final_size - 4);
    CHECK(fs->file_seek(fs, &file, gap_end, SEEK_SET) == gap_end);
    const DWORD before = free_clusters();
    CHECK(fs->file_write(fs, &file, "TAIL", 0) == 0);
    CHECK(fs->file_size(fs, &file) == 31);
    CHECK(free_clusters() == before);
    CHECK(fs->file_write(fs, &file, "TAIL", 4) == 4);
    CHECK(fs->file_tell(fs, &file) == (off_t)final_size);
    check_stat_size(fs, "/zero.bin", (off_t)final_size);
    CHECK(fs->file_close(fs, &file) == 0);

    // Remount to verify initialized contents and size survive metadata publication.
    CHECK(fs->unmount(fs) == 0);
    CHECK(fs->mount(fs, heap, false) == 0);
    CHECK(fs->file_open(fs, &file, "/zero.bin", O_RDONLY) == 0);
    CHECK(fs->file_read(fs, &file, data, final_size) == (ssize_t)final_size);
    check_bytes(data, 0, final_size - 4, 0);
    CHECK(memcmp(data + final_size - 4, "TAIL", 4) == 0);
    CHECK(fs->file_close(fs, &file) == 0);
    free(data);
    destroy_heap_filesystem(fs, heap);
}

/** Check retained zero-filled progress and recovery after physical growth-write failures. */
static void test_zero_fill_errors(bool gap_write, bool exfat, size_t sector_size) {
    // First record a successful operation's writes, then fail each physical write
    // in turn, covering allocation, zero data, payload, and metadata publication.
    unsigned total_programs = 0;
    for (unsigned failure = 0; failure <= total_programs; ++failure) {
        blockdevice_t *heap;
        filesystem_t *fs = create_zero_fill_filesystem(&heap, exfat, sector_size);
        fs_file_t file = {0};
        CHECK(fs->file_open(fs, &file, "/failure.bin", O_CREAT | O_RDWR) == 0);
        CHECK(fs->file_write(fs, &file, "seed", 4) == 4);
        const off_t target = (off_t)(3 * sector_size + 17);
        CHECK(fs->file_seek(fs, &file, target, SEEK_SET) == target);
        saved_program = heap->program;
        heap->program = failing_program;
        program_calls = 0;
        fail_program_at = failure;
        fail_at_address = false;
        program_failed = false;
        const ssize_t result = gap_write ? fs->file_write(fs, &file, "TAIL", 4)
                                        : fs->file_truncate(fs, &file, target);
        heap->program = saved_program;
        if (failure == 0) {
            CHECK(result == (gap_write ? 4 : 0));
            CHECK(!program_failed);
            total_programs = program_calls;
            CHECK(total_programs > 0);
            CHECK(fs->file_close(fs, &file) == 0);
            destroy_heap_filesystem(fs, heap);
            continue;
        }
        CHECK(program_failed);
        CHECK(result == -EIO);
        const FIL *physical = file.context;
        const size_t retained_size = (size_t)f_size(physical);
        CHECK(retained_size >= 4 && retained_size <= (size_t)target + (gap_write ? 4 : 0));

        // Internal zeros never consume the logical cursor or clear an abort.
        // Sync and reopen after removing the fault to inspect retained growth.
        if (physical->err) {
            CHECK(fs->file_write(fs, &file, "X", 1) == -EIO);
        } else {
            CHECK(fs->file_tell(fs, &file) ==
                  (retained_size > (size_t)target ? (off_t)retained_size : target));
        }
        CHECK(fs->file_sync(fs, &file) == 0);
        CHECK(fs->file_close(fs, &file) == 0);
        // A failed directory write can leave the volume window dirty even after
        // FatFs clears the file's modified flag. A new entry flushes that window.
        CHECK(fs->file_open(fs, &file, "/recovery.bin", O_CREAT | O_RDWR) == 0);
        CHECK(fs->file_close(fs, &file) == 0);
        CHECK(fs->unmount(fs) == 0);
        CHECK(fs->mount(fs, heap, false) == 0);
        CHECK(fs->file_open(fs, &file, "/failure.bin", O_RDONLY) == 0);
        char data[3 * FF_MAX_SS + 21];
        CHECK(fs->file_size(fs, &file) == (off_t)retained_size);
        CHECK(fs->file_read(fs, &file, data, sizeof(data)) == (ssize_t)retained_size);
        CHECK(memcmp(data, "seed", 4) == 0);
        const size_t zero_end = retained_size < (size_t)target ? retained_size : (size_t)target;
        check_bytes(data, 4, zero_end, 0);
        if (retained_size > (size_t)target) {
            CHECK(memcmp(data + target, "TAIL", retained_size - (size_t)target) == 0);
        }
        CHECK(fs->file_close(fs, &file) == 0);
        destroy_heap_filesystem(fs, heap);
    }
}

/** Verify stale capacity counts cannot turn disk exhaustion into uninitialized growth. */
static void test_zero_fill_no_space(void) {
    for (unsigned gap_write = 0; gap_write < 2; ++gap_write) {
        blockdevice_t *heap;
        filesystem_t *fs = create_heap_filesystem(&heap);
        fs_file_t file = {0}, filler = {0};
        CHECK(fs->file_open(fs, &file, "/full.bin", O_CREAT | O_RDWR) == 0);
        CHECK(fs->file_write(fs, &file, "Q", 1) == 1);
        CHECK(fs->file_open(fs, &filler, "/filler.bin", O_CREAT | O_RDWR) == 0);
        char data[4096];
        memset(data, 'F', sizeof(data));
        ssize_t written;
        do {
            written = fs->file_write(fs, &filler, data, sizeof(data));
            CHECK(written > 0 || written == -ENOSPC);
        } while (written > 0);
        CHECK(fs->file_close(fs, &filler) == 0);
        CHECK(free_clusters() == 0);

        FIL *physical = file.context;
        FATFS *volume = physical->obj.fs;
        const off_t cluster_bytes = (off_t)volume->csize * volume->ssize;
        CHECK((size_t)cluster_bytes <= sizeof(data));
        CHECK(fs->file_seek(fs, &file, cluster_bytes, SEEK_SET) == cluster_bytes);
        // Model a stale free-space cache: the precheck admits growth, but actual
        // allocation must discover that every cluster belongs to a live file.
        volume->free_clst = 1;
        if (gap_write) {
            CHECK(fs->file_write(fs, &file, "TAIL", 4) == -ENOSPC);
        } else {
            CHECK(fs->file_truncate(fs, &file, 2 * cluster_bytes) == -ENOSPC);
        }
        volume->free_clst = 0;
        CHECK(fs->file_size(fs, &file) == cluster_bytes);
        CHECK(fs->file_tell(fs, &file) == cluster_bytes);
        CHECK(free_clusters() == 0);
        CHECK(fs->file_close(fs, &file) == 0);
        check_stat_size(fs, "/full.bin", cluster_bytes);

        // Reopen after exhaustion: only the original byte and initialized suffix
        // are visible, and the rejected payload must not appear in the file.
        CHECK(fs->file_open(fs, &file, "/full.bin", O_RDONLY) == 0);
        CHECK(fs->file_read(fs, &file, data, sizeof(data)) == cluster_bytes);
        CHECK(data[0] == 'Q');
        check_bytes(data, 1, (size_t)cluster_bytes, 0);
        CHECK(fs->file_close(fs, &file) == 0);
        destroy_heap_filesystem(fs, heap);
    }
}

/** Verify failed writes report completed bytes and preserve the FatFs error latch. */
static void test_partial_write_errors(void) {
    blockdevice_t *heap;
    filesystem_t *fs = create_heap_filesystem(&heap);
    FATFS *volume = NULL;
    DWORD count = 0;
    CHECK(f_getfree("0:", &count, &volume) == FR_OK);
    const size_t cluster_bytes = (size_t)volume->csize * volume->ssize;
    const size_t size = 3 * cluster_bytes;
    char *payload = malloc(size);
    char *actual = malloc(size);
    CHECK(payload != NULL && actual != NULL);

    for (unsigned fail_at = 1; fail_at <= 2; ++fail_at) {
        fs_file_t file = {0};
        CHECK(fs->file_open(fs, &file, "/partial.bin", O_CREAT | O_TRUNC | O_RDWR) == 0);
        memset(payload, '.', size);
        CHECK(fs->file_write(fs, &file, payload, size) == (ssize_t)size);
        CHECK(fs->file_close(fs, &file) == 0);
        CHECK(fs->file_open(fs, &file, "/partial.bin", O_RDWR) == 0);

        // Preallocate and sync first, so each cluster transfer reaches program
        // without unrelated allocation or directory writes affecting the fault.
        saved_program = heap->program;
        heap->program = failing_program;
        program_calls = 0;
        fail_program_at = fail_at;
        fail_at_address = false;
        program_failed = false;
        memset(payload, 'A', size);
        const ssize_t completed = fail_at == 1 ? 0 : (ssize_t)cluster_bytes;
        CHECK(fs->file_write(fs, &file, payload, size) == (completed ? completed : -EIO));
        CHECK(program_failed);

        // A partial success must not clear FatFs's hard-error latch. Closing
        // and reopening permits verification of the completed prefix and untouched
        // suffix. The failed cluster was erased before program, so its bytes are unspecified.
        heap->program = saved_program;
        CHECK(fs->file_write(fs, &file, "X", 1) == -EIO);
        CHECK(fs->file_close(fs, &file) == 0);
        CHECK(fs->file_open(fs, &file, "/partial.bin", O_RDONLY) == 0);
        CHECK(fs->file_read(fs, &file, actual, size) == (ssize_t)size);
        check_bytes(actual, 0, (size_t)completed, 'A');
        check_bytes(actual, (size_t)completed + cluster_bytes, size, '.');
        CHECK(fs->file_close(fs, &file) == 0);
    }
    free(actual);
    free(payload);
    destroy_heap_filesystem(fs, heap);
}

/** Verify completed extensions survive immediate stat, sync retries, close, and remount. */
static void test_partial_extension_errors(void) {
    const BYTE types[] = {FS_FAT12, FS_EXFAT};
    for (size_t format = 0; format < sizeof(types) / sizeof(types[0]); ++format) {
        for (unsigned sync_failure = 0; sync_failure < 2; ++sync_failure) {
            blockdevice_t *heap = blockdevice_heap_create(
                types[format] == FS_EXFAT ? 8u * 1024u * 1024u : 128u * 1024u);
            filesystem_t *fs = filesystem_fat_create();
            CHECK(heap != NULL && fs != NULL);
            CHECK(fs->mount(fs, heap, true) == 0);
            const MKFS_PARM options = {
                .fmt = (types[format] == FS_EXFAT ? FM_EXFAT : FM_FAT) | FM_SFD,
                .au_size = types[format] == FS_EXFAT ? 4096 : 512,
            };
            uint8_t work[FF_MAX_SS] = {0};
            CHECK(f_mkfs("0:", &options, work, sizeof(work)) == FR_OK);
            CHECK(fs->unmount(fs) == 0);
            CHECK(fs->mount(fs, heap, false) == 0);
            FATFS *volume = NULL;
            DWORD count = 0;
            CHECK(f_getfree("0:", &count, &volume) == FR_OK);
            CHECK(volume->fs_type == types[format]);
            const size_t cluster_bytes = (size_t)volume->csize * volume->ssize;
            char *payload = malloc(3 * cluster_bytes);
            char *actual = malloc(2 * cluster_bytes);
            CHECK(payload != NULL && actual != NULL);
            fs_file_t file = {0};
            CHECK(fs->file_open(fs, &file, "/extend.bin", O_CREAT | O_RDWR) == 0);
            memset(payload, 'S', cluster_bytes);
            CHECK(fs->file_write(fs, &file, payload, cluster_bytes) == (ssize_t)cluster_bytes);
            CHECK(fs->file_close(fs, &file) == 0);
            CHECK(fs->file_open(fs, &file, "/extend.bin", O_RDWR) == 0);
            CHECK(fs->file_seek(fs, &file, 0, SEEK_END) == (off_t)cluster_bytes);

            // Reopening clears the previous modified flag. Fail the second
            // extension transfer after the first complete cluster is accepted.
            FATFS_DIR root = {0};
            CHECK(f_opendir(&root, "0:/") == FR_OK);
            fail_program_address = root.sect * volume->ssize;
            CHECK(f_closedir(&root) == FR_OK);
            saved_program = heap->program;
            heap->program = failing_program;
            program_calls = 0;
            fail_program_at = 2;
            fail_at_address = false;
            program_failed = false;
            fail_followup_metadata = sync_failure != 0;
            metadata_failed = false;
            memset(payload, 'A', 3 * cluster_bytes);
            CHECK(fs->file_write(fs, &file, payload, 3 * cluster_bytes) ==
                  (ssize_t)cluster_bytes);
            CHECK(program_failed);
            CHECK(metadata_failed == (sync_failure != 0));

            // Partial success must preserve the abort latch. A metadata failure
            // must also retain the modified flag so sync/close can really retry.
            heap->program = saved_program;
            fail_followup_metadata = false;
            const FIL *physical = file.context;
            CHECK(physical->err == FR_DISK_ERR);
            CHECK(fs->file_write(fs, &file, "X", 1) == -EIO);
            if (sync_failure) CHECK(fs->file_sync(fs, &file) == 0);
            CHECK(physical->err == FR_DISK_ERR);
            check_stat_size(fs, "/extend.bin", (off_t)(2 * cluster_bytes));
            CHECK(fs->file_close(fs, &file) == 0);

            // Discard every FatFs cache before checking that the acknowledged
            // extension, and only its completed prefix, remains readable.
            CHECK(fs->unmount(fs) == 0);
            CHECK(fs->mount(fs, heap, false) == 0);
            check_stat_size(fs, "/extend.bin", (off_t)(2 * cluster_bytes));
            CHECK(fs->file_open(fs, &file, "/extend.bin", O_RDONLY) == 0);
            CHECK(fs->file_read(fs, &file, actual, 2 * cluster_bytes) ==
                  (ssize_t)(2 * cluster_bytes));
            check_bytes(actual, 0, cluster_bytes, 'S');
            check_bytes(actual, cluster_bytes, 2 * cluster_bytes, 'A');
            CHECK(fs->file_read(fs, &file, actual, 1) == 0);
            CHECK(fs->file_close(fs, &file) == 0);
            free(actual);
            free(payload);
            destroy_heap_filesystem(fs, heap);
        }
    }
}

/** Verify signed-offset and format ceilings clip writes and reject further growth. */
static void test_write_size_ceiling(void) {
    blockdevice_t *heap;
    filesystem_t *fs = create_heap_filesystem(&heap);
    const BYTE types[] = {FS_FAT32, FS_EXFAT};
    const int modes[] = {O_RDWR, O_RDWR | O_APPEND};
    for (size_t format = 0; format < sizeof(types) / sizeof(types[0]); ++format) {
        for (size_t mode = 0; mode < sizeof(modes) / sizeof(modes[0]); ++mode) {
            fs_file_t file = {0};
            CHECK(fs->file_open(fs, &file, "/limit.bin", O_CREAT | O_TRUNC | modes[mode]) == 0);
            FIL *physical = file.context;
            FIL saved_file = *physical;
            const BYTE saved_type = physical->obj.fs->fs_type;
            const FSIZE_t format_limit = types[format] == FS_EXFAT ? (FSIZE_t)-1 : UINT32_MAX;
            const FSIZE_t ceiling = format_limit < (FSIZE_t)TEST_OFF_MAX
                ? format_limit : (FSIZE_t)TEST_OFF_MAX;

            // Simulate valid enormous files without allocating their storage.
            // Only FatFs storage operations are stubbed; real adapter positioning,
            // append policy, request clipping, and overflow checks remain active.
            boundary_file = physical;
            boundary_writes = 0;
            physical->obj.fs->fs_type = types[format];
            physical->obj.objsize = physical->fptr = ceiling - 1;
            CHECK(fs->file_seek(fs, &file, mode ? 0 : (off_t)(ceiling - 1), SEEK_SET) ==
                  (mode ? 0 : (off_t)(ceiling - 1)));
            CHECK(fs->file_write(fs, &file, "XY", 2) == 1);
            CHECK(boundary_writes == 1);
            CHECK(fs->file_tell(fs, &file) == (off_t)ceiling);
            CHECK(fs->file_size(fs, &file) == (off_t)ceiling);
            check_stat_size(fs, "/limit.bin", (off_t)ceiling);

            // At the ceiling, including append after seeking backward, refuse
            // growth without transferring bytes or changing size/cursor.
            if (mode) CHECK(fs->file_seek(fs, &file, 0, SEEK_SET) == 0);
            CHECK(fs->file_write(fs, &file, "", 0) == 0);
            CHECK(fs->file_write(fs, &file, "Z", 1) == -EFBIG);
            CHECK(boundary_writes == 1);
            CHECK(fs->file_tell(fs, &file) == (mode ? 0 : (off_t)ceiling));
            CHECK(fs->file_size(fs, &file) == (off_t)ceiling);
            check_stat_size(fs, "/limit.bin", (off_t)ceiling);

            // Restore the real file and volume before native close/unmount.
            physical->obj.fs->fs_type = saved_type;
            *physical = saved_file;
            boundary_file = NULL;
            CHECK(fs->file_close(fs, &file) == 0);
        }
    }
    destroy_heap_filesystem(fs, heap);
}

/** Verify oversized imported files preserve signed read cursors, EOF, and recovery. */
static void test_read_offset_ceiling(void) {
    blockdevice_t *heap;
    filesystem_t *fs = create_heap_filesystem(&heap);
    fs_file_t file = {0};
    CHECK(fs->file_open(fs, &file, "/import.bin", O_CREAT | O_RDWR) == 0);
    CHECK(fs->file_close(fs, &file) == 0);
    const BYTE types[] = {FS_FAT32, FS_EXFAT};
    const FSIZE_t ceiling = (FSIZE_t)TEST_OFF_MAX;
    for (size_t format = 0; format < sizeof(types) / sizeof(types[0]); ++format) {
        const FSIZE_t format_limit = types[format] == FS_EXFAT ? (FSIZE_t)-1 : UINT32_MAX;
        // FAT32 exceeds off_t only on narrower targets; exFAT can do so on wider hosts.
        if (ceiling + 2 > format_limit) continue;
        CHECK(fs->file_open(fs, &file, "/import.bin", O_RDONLY) == 0);
        FIL *physical = file.context;
        FIL saved_file = *physical;
        const BYTE saved_type = physical->obj.fs->fs_type;
        boundary_file = physical;
        boundary_reads = 0;
        physical->obj.fs->fs_type = types[format];
        physical->obj.objsize = ceiling + 2;
        physical->fptr = ceiling - 1;

        // An imported file can have an unrepresentable size but still permit
        // reads of its representable prefix. Clip the two-byte request to one.
        CHECK(fs->file_size(fs, &file) == -EOVERFLOW);
        CHECK(fs->file_seek(fs, &file, (off_t)(ceiling - 1), SEEK_SET) ==
              (off_t)(ceiling - 1));
        char buffer[2] = {0};
        CHECK(fs->file_read(fs, &file, buffer, sizeof(buffer)) == 1);
        CHECK(buffer[0] == 'R' && buffer[1] == 0);
        CHECK(boundary_reads == 1);
        CHECK(fs->file_tell(fs, &file) == (off_t)ceiling);
        CHECK(fs->file_seek(fs, &file, 0, SEEK_CUR) == (off_t)ceiling);

        // Data remains, but consuming it would overflow the cursor. Neither
        // this error nor a zero-byte read may transfer data or move position.
        CHECK(fs->file_read(fs, &file, buffer, 0) == 0);
        CHECK(fs->file_read(fs, &file, buffer, 1) == -EOVERFLOW);
        CHECK(boundary_reads == 1);
        CHECK(fs->file_tell(fs, &file) == (off_t)ceiling);
        CHECK(physical->fptr == ceiling);

        // A relative seek and subsequent read prove overflow did not abort the handle.
        CHECK(fs->file_seek(fs, &file, -1, SEEK_CUR) == (off_t)(ceiling - 1));
        CHECK(fs->file_read(fs, &file, buffer, 1) == 1);
        CHECK(boundary_reads == 2);
        CHECK(fs->file_tell(fs, &file) == (off_t)ceiling);

        // True EOF and an overhanging cursor must still return zero at the limit.
        physical->obj.objsize = ceiling;
        CHECK(fs->file_read(fs, &file, buffer, 1) == 0);
        physical->obj.objsize = ceiling - 1;
        CHECK(fs->file_read(fs, &file, buffer, 1) == 0);
        CHECK(boundary_reads == 2);
        CHECK(fs->file_tell(fs, &file) == (off_t)ceiling);
        CHECK(fs->file_seek(fs, &file, 0, SEEK_SET) == 0);
        CHECK(fs->file_read(fs, &file, buffer, 1) == 1);
        CHECK(fs->file_tell(fs, &file) == 1);

        // Restore the real empty file and volume before native close/unmount.
        physical->obj.fs->fs_type = saved_type;
        *physical = saved_file;
        boundary_file = NULL;
        CHECK(fs->file_close(fs, &file) == 0);
    }
    destroy_heap_filesystem(fs, heap);
}

/** Verify a directory-sync failure does not let the next write overwrite prior data. */
static void test_write_cursor_after_sync_error(void) {
    blockdevice_t *heap;
    filesystem_t *fs = create_heap_filesystem(&heap);
    fs_file_t file = {0};
    CHECK(fs->file_open(fs, &file, "/sync.bin", O_CREAT | O_RDWR) == 0);
    CHECK(fs->file_write(fs, &file, "seed", 4) == 4);
    CHECK(fs->file_close(fs, &file) == 0);
    CHECK(fs->file_open(fs, &file, "/sync.bin", O_RDWR) == 0);

    // Select the actual directory sector, not CTRL_SYNC (which does not invoke
    // the block device's sync callback). Data-sector writes remain successful.
    const FIL *physical = file.context;
    fail_program_address = physical->dir_sect * physical->obj.fs->ssize;
    saved_program = heap->program;
    heap->program = failing_program;
    program_calls = 0;
    fail_at_address = true;
    program_failed = false;
    CHECK(fs->file_write(fs, &file, "AAAAAAAAAAAAAAAA", 16) == -EIO);
    CHECK(program_failed);
    CHECK(fs->file_tell(fs, &file) == 16);

    // Retry metadata synchronization before the next payload; neither operation
    // may rewind the cursor already consumed by the failed write.
    heap->program = saved_program;
    CHECK(fs->file_sync(fs, &file) == 0);
    CHECK(fs->file_write(fs, &file, "BBBB", 4) == 4);
    CHECK(fs->file_tell(fs, &file) == 20);
    CHECK(fs->file_close(fs, &file) == 0);
    check_stat_size(fs, "/sync.bin", 20);
    CHECK(fs->file_open(fs, &file, "/sync.bin", O_RDONLY) == 0);
    char actual[20];
    CHECK(fs->file_read(fs, &file, actual, sizeof(actual)) == (ssize_t)sizeof(actual));
    CHECK(memcmp(actual, "AAAAAAAAAAAAAAAABBBB", sizeof(actual)) == 0);
    CHECK(fs->file_close(fs, &file) == 0);
    destroy_heap_filesystem(fs, heap);
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
    // Check the preserved prefix, zero-filled extension/gap, and final payload.
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
    check_bytes(grown, 513, 3000, 0);
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

/** Run real-volume lifecycle regressions and storage-free sector-range tests. */
int main(void) {
    test_failed_open_cleanup();
    test_partial_write_errors();
    test_partial_extension_errors();
    test_write_size_ceiling();
    test_read_offset_ceiling();
    test_write_cursor_after_sync_error();
    test_zero_fill_no_space();
    test_truncate_metadata_and_cursor();
    test_cursor_and_access_modes();
    test_append_after_truncate();
    test_bounds_and_capacity_precheck();
    test_exclusive_create();
    test_exclusive_probe_failures();
    test_fat32_second_volume();
    // Keep adapter arithmetic coverage independent of the heap volume's geometry.
    const size_t sector_sizes[] = {512, 4096};
    for (size_t i = 0; i < sizeof(sector_sizes) / sizeof(sector_sizes[0]); ++i) {
        test_zero_fill(false, sector_sizes[i]);
        test_zero_fill(true, sector_sizes[i]);
        for (unsigned exfat = 0; exfat < 2; ++exfat) {
            test_zero_fill_errors(false, exfat, sector_sizes[i]);
            test_zero_fill_errors(true, exfat, sector_sizes[i]);
        }
        test_fat_disk(sector_sizes[i]);
        printf("fat-disk-%zu-byte-sectors: passed\n", sector_sizes[i]);
    }
    return EXIT_SUCCESS;
}
