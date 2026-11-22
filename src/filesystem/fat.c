/*
 * Copyright 2024, Hiroyuki OYAMA
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pico/mutex.h>
#undef PATH_MAX
#include "blockdevice/blockdevice.h"
#include "filesystem/fat.h"
#include "ff.h"
#include "diskio.h"

_Static_assert(sizeof(LBA_t) >= sizeof(bd_size_t),
               "FatFs adapter requires 64-bit LBA_t (FF_LBA64=1)");

#define FAT_OFF_MAX   ((FSIZE_t)(((uintmax_t)1 << (sizeof(off_t) * CHAR_BIT - 1)) - 1))
#define FAT_SSIZE_MAX ((size_t)(((uintmax_t)1 << (sizeof(ssize_t) * CHAR_BIT - 1)) - 1))

_Static_assert((off_t)-1 < 0, "off_t must be signed");
_Static_assert(sizeof(off_t) <= sizeof(FSIZE_t),
               "FatFs FSIZE_t must hold all non-negative off_t values");

typedef struct {
    FIL file;
    /* Keep the public cursor independent of FatFs seeks, which can grow files. */
    FSIZE_t pos;
    bool append;
} fat_file_t;

typedef struct {
    FATFS fatfs;
    int id;
    mutex_t _mutex;
    mutex_t _mutex_format;
} filesystem_fat_context_t;

static const char FILESYSTEM_NAME[] = "FAT";
static blockdevice_t *_ffs[FF_VOLUMES] = {0};

static int fat_error_remap(FRESULT res) {
    switch (res) {
    case FR_OK:                   // (0) Succeeded
        return 0;
    case FR_DISK_ERR:             // (1) A hard error occurred in the low level disk I/O layer
        return -EIO;
    case FR_INT_ERR:              // (2) Assertion failed
        return -1;
    case FR_NOT_READY:            // (3) The physical drive cannot work
        return -EIO;
    case FR_NO_FILE:              // (4) Could not find the file
        return -ENOENT;
    case FR_NO_PATH:              // (5) Could not find the path
        return -ENOTDIR;
    case FR_INVALID_NAME:         // (6) The path name format is invalid
        return -EINVAL;
    case FR_DENIED:               // (7) Access denied due to prohibited access or directory full
        return -EACCES;
    case FR_EXIST:                // (8) Access denied due to prohibited access
        return -EEXIST;
    case FR_INVALID_OBJECT:       // (9) The file/directory object is invalid
        return -EBADF;
    case FR_WRITE_PROTECTED:      // (10) The physical drive is write protected
        return -EACCES;
    case FR_INVALID_DRIVE:        // (11) The logical drive number is invalid
        return -ENODEV;
    case FR_NOT_ENABLED:          // (12) The volume has no work area
        return -ENODEV;
    case FR_NO_FILESYSTEM:        // (13) There is no valid FAT volume
        return -EINVAL;
    case FR_MKFS_ABORTED:         // (14) The f_mkfs() aborted due to any problem
        return -EIO;
    case FR_TIMEOUT:              // (15) Could not get a grant to access the volume within defined period
        return -ETIMEDOUT;
    case FR_LOCKED:               // (16) The operation is rejected according to the file sharing policy
        return -EBUSY;
    case FR_NOT_ENOUGH_CORE:      // (17) LFN working buffer could not be allocated
        return -ENOMEM;
    case FR_TOO_MANY_OPEN_FILES:  // (18) Number of open files > FF_FS_LOCK
        return -ENFILE;
    case FR_INVALID_PARAMETER:    // (19) Given parameter is invalid
        return -EINVAL;
    default:
        return -res;
    }
}

static inline void debug_if(int condition, const char *format, ...) {
    if (condition) {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }
}

DSTATUS disk_initialize(BYTE pdrv) {
    debug_if(FFS_DBG, "disk_initialize on pdrv [%d]\n", pdrv);
    return (DSTATUS)_ffs[pdrv]->init(_ffs[pdrv]);
}

static WORD disk_get_sector_size(BYTE pdrv) {
    size_t sector_size = _ffs[(int)pdrv]->erase_size;

    WORD ssize = sector_size;
    if (ssize < 512) {
        ssize = 512;
    }
    return ssize;
}

static LBA_t disk_get_sector_count(BYTE pdrv) {
    // floor(byte capacity / sector size) leaves any trailing bytes unused
    // rather than treating them as an error.
    //
    // Sector-based drivers such as SD are responsible for reporting correctly
    // aligned geometry.  We don't enforce alignment here.
    return _ffs[pdrv]->size(_ffs[pdrv]) / disk_get_sector_size(pdrv);
}

// Convert a sector range on a registered drive to bytes.
static bool disk_get_byte_range(BYTE pdrv, LBA_t sector, LBA_t count,
                               bd_size_t *addr, bd_size_t *size) {
    const LBA_t sectors = disk_get_sector_count(pdrv);

    // Short-circuiting checks sector <= sectors before subtracting. Comparing
    // count to the remaining sectors avoids an overflowing sector + count.
    // Empty ranges, including at the end of the device, reach the backend so
    // it retains responsibility for accepting or rejecting zero-byte transfers.
    if (sector > sectors || count > sectors - sector) {
        // No output has been written, so failure leaves both outputs unchanged.
        return false;
    }

    const bd_size_t ssize = disk_get_sector_size(pdrv);

    // Both sector and count are bounded by floor(byte capacity / ssize), so
    // each product fits bd_size_t. Multiply in that type, not 32-bit arithmetic.
    *addr = (bd_size_t)sector * ssize;
    *size = (bd_size_t)count * ssize;
    return true;
}

// Convert inclusive sector endpoints using the shared bounds and byte conversion.
static bool disk_get_byte_range_inclusive(BYTE pdrv, LBA_t first, LBA_t last,
                                         bd_size_t *addr, bd_size_t *size) {
    if (first > last) {
        return false;
    }

    const LBA_t count = last - first;

    // Ensure count + 1 does not overflow LBA_t.
    if (count == (LBA_t)-1) {
        return false;
    }

    return disk_get_byte_range(pdrv, first, count + 1, addr, size);
}

DSTATUS disk_status(BYTE pdrv) {
    debug_if(FFS_DBG, "disk_status on pdrv [%d]\n", pdrv);
    return RES_OK;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    bd_size_t addr, size;
    if (!disk_get_byte_range(pdrv, sector, count, &addr, &size)) {
        return RES_PARERR;
    }
    int err = _ffs[(int)pdrv]->read(_ffs[(int)pdrv], (const void *)buff, addr, size);
    return err ? RES_PARERR : RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    debug_if(FFS_DBG, "disk_write [%d]\n", pdrv);
    bd_size_t addr, size;
    if (!disk_get_byte_range(pdrv, sector, count, &addr, &size)) {
        return RES_PARERR;
    }

    int err = _ffs[pdrv]->erase(_ffs[pdrv], addr, size);
    if (err) {
        return RES_PARERR;
    }

    err = _ffs[pdrv]->program(_ffs[pdrv], buff, addr, size);
    if (err) {
        return RES_PARERR;
    }

    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    debug_if(FFS_DBG, "disk_ioctl(%d)\n", cmd);
    switch (cmd) {
    case CTRL_SYNC:
        if (_ffs[pdrv] == NULL) {
            return RES_NOTRDY;
        } else {
            return RES_OK;
        }
    case GET_SECTOR_COUNT:
        if (_ffs[pdrv] == NULL) {
            return RES_NOTRDY;
        } else {
            // FatFs supplies an LBA_t destination, not a DWORD, with FF_LBA64.
            *((LBA_t *)buff) = disk_get_sector_count(pdrv);
            return RES_OK;
        }
    case GET_SECTOR_SIZE:
        if (_ffs[pdrv] == NULL) {
            return RES_NOTRDY;
        } else {
            *((WORD *)buff) = disk_get_sector_size(pdrv);
            return RES_OK;
        }
    case GET_BLOCK_SIZE:
        *((DWORD *)buff) = 1; // default when not known
        return RES_OK;
    case CTRL_TRIM:
        if (_ffs[pdrv] == NULL) {
            return RES_NOTRDY;
        } else {
            // FatFs supplies LBA_t[2] with an inclusive last sector.
            const LBA_t *sectors = (const LBA_t *)buff;
            bd_size_t addr, size;
            if (!disk_get_byte_range_inclusive(pdrv, sectors[0], sectors[1], &addr, &size)) {
                return RES_PARERR;
            }
            int err = _ffs[pdrv]->trim(_ffs[pdrv], addr, size);
            return err ? RES_PARERR : RES_OK;
        }
    }
    return RES_PARERR;
}

DWORD get_fattime(void) {
    time_t rawtime;
    time(&rawtime);
    struct tm *ptm = localtime(&rawtime);
    return (DWORD)(ptm->tm_year - 80) << 25
           | (DWORD)(ptm->tm_mon + 1) << 21
           | (DWORD)(ptm->tm_mday) << 16
           | (DWORD)(ptm->tm_hour) << 11
           | (DWORD)(ptm->tm_min) << 5
           | (DWORD)(ptm->tm_sec / 2);
}


static int mount(filesystem_t *fs, blockdevice_t *device, bool pending) {
    filesystem_fat_context_t *context = fs->context;
    mutex_enter_blocking(&context->_mutex);

    char _fsid[3] = {0};
    for (size_t i = 0; i < FF_VOLUMES; i++) {
        if (_ffs[i] == NULL) {
            context->id = i;
            _ffs[i] = device;
            _fsid[0] = '0' + i;
            _fsid[1] = ':';
            _fsid[2] = '\0';
            FRESULT res = f_mount(&context->fatfs, _fsid, !pending);
            mutex_exit(&context->_mutex);
            return fat_error_remap(res);
        }
    }
    mutex_exit(&context->_mutex);
    return -ENOMEM;
}

static int unmount(filesystem_t *fs) {
    filesystem_fat_context_t *context = fs->context;
    mutex_enter_blocking(&context->_mutex);

    char _fsid[3] = "0:";
    _fsid[0] = '0' + context->id;

    FRESULT res = f_mount(NULL, _fsid, 0);
    _ffs[context->id] = NULL;

    mutex_exit(&context->_mutex);
    return fat_error_remap(res);
}

static int format(filesystem_t *fs, blockdevice_t *device) {
    filesystem_fat_context_t *context = fs->context;
    mutex_enter_blocking(&context->_mutex_format);

    if (!device->is_initialized) {
        int err = device->init(device);
        if (err) {
            mutex_exit(&context->_mutex_format);
            return err;
        }
    }

    // erase first handful of blocks
    bd_size_t header = 2 * device->erase_size;
    int err = device->erase(device, 0, header);
    if (err) {
        mutex_exit(&context->_mutex_format);
        return err;
    }

    size_t program_size = device->program_size;
    void *buffer = malloc(program_size);
    if (!buffer) {
        mutex_exit(&context->_mutex_format);
        return -ENOMEM;
    }
    memset(buffer, 0xFF, program_size);
    for (size_t i = 0; i < header; i += program_size) {
        err = device->program(device, buffer, i, program_size);
        if (err) {
            free(buffer);
            mutex_exit(&context->_mutex_format);
            return err;
        }
    }
    free(buffer);

    // trim entire device to indicate it is unneeded
    err = device->trim(device, 0, device->size(device));
    if (err) {
        mutex_exit(&context->_mutex_format);
        return err;
    }

    err = fs->mount(fs, device, true);
    if (err) {
        mutex_exit(&context->_mutex_format);
        return err;
    }

    MKFS_PARM opt;
    opt.fmt = (FM_ANY | FM_SFD);
    opt.n_fat = 0;
    opt.align = 0U;
    opt.n_root = 0U;
    opt.au_size = 0; // (DWORD)cluster_size;

    char id[3] = "0:";
    id[0] = '0' + context->id;

    FRESULT res = f_mkfs((const TCHAR *)id, &opt, NULL, FF_MAX_SS);

    if (res != FR_OK) {
        fs->unmount(fs);
        mutex_exit(&context->_mutex_format);
        return fat_error_remap(res);
    }

    err = fs->unmount(fs);
    if (err) {
        mutex_exit(&context->_mutex_format);
        return res;
    }

    mutex_exit(&context->_mutex_format);
    return 0;
}

static const char *fat_path_prefix(char *dist, int id, const char *path) {
    if (id == 0) {
        strcpy(dist, path);
        return (const char *)dist;
    }

    dist[0] = '0' + id;
    dist[1] = ':';
    strcpy(dist + strlen("0:"), path);
    return (const char *)dist;
}

static int file_remove(filesystem_t *fs, const char *path) {
    filesystem_fat_context_t *context = fs->context;

    char fpath[PATH_MAX];
    fat_path_prefix(fpath, context->id, path);

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = f_unlink(fpath);
    mutex_exit(&context->_mutex);

    if (res != FR_OK) {
        debug_if(FFS_DBG, "f_unlink() failed: %d\n", res);
        if (res == FR_DENIED)
            return -ENOTEMPTY;
    }
    return fat_error_remap(res);
}

static int file_rename(filesystem_t *fs, const char *oldpath, const char *newpath) {
    filesystem_fat_context_t *context = fs->context;
    char oldfpath[PATH_MAX];
    char newfpath[PATH_MAX];
    fat_path_prefix(oldfpath, context->id, oldpath);
    fat_path_prefix(newfpath, context->id, newpath);

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = f_rename(oldfpath, newfpath);
    mutex_exit(&context->_mutex);

    if (res != FR_OK) {
        debug_if(FFS_DBG, "f_rename() failed: %d\n", res);
    }
    return fat_error_remap(res);
}

static int file_mkdir(filesystem_t *fs, const char *path, mode_t mode) {
    (void)mode;
    filesystem_fat_context_t *context = fs->context;
    char fpath[PATH_MAX];
    fat_path_prefix(fpath, context->id, path);

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = f_mkdir(fpath);
    mutex_exit(&context->_mutex);

    if (res != FR_OK) {
        debug_if(FFS_DBG, "f_mkdir() failed: %d\n", res);
    }
    return fat_error_remap(res);
}

static int file_rmdir(filesystem_t *fs, const char *path) {
    filesystem_fat_context_t *context = fs->context;
    char fpath[PATH_MAX];
    fat_path_prefix(fpath, context->id, path);

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = f_unlink(fpath);
    mutex_exit(&context->_mutex);

    if (res != FR_OK) {
        debug_if(FFS_DBG, "f_unlink() failed: %d\n", res);
    }
    return fat_error_remap(res);
}

/** Read path-based FAT metadata and translate it into the VFS stat structure. */
static int file_stat(filesystem_t *fs, const char *path, struct stat *st) {
    filesystem_fat_context_t *context = fs->context;
    char fpath[PATH_MAX];
    fat_path_prefix(fpath, context->id, path);
    FILINFO f = {0};

    // VFS fstat also uses this directory lookup, not the open FIL's size.
    mutex_enter_blocking(&context->_mutex);
    FRESULT res = f_stat(fpath, &f);
    mutex_exit(&context->_mutex);

    if (res != FR_OK) {
        return fat_error_remap(res);
    }
    // Reject sizes that cannot be represented by the target's signed off_t.
    if (f.fsize > FAT_OFF_MAX) {
        return -EOVERFLOW;
    }
    st->st_size = (off_t)f.fsize;
    st->st_mode = 0;
    st->st_mode |= (f.fattrib & AM_DIR) ? S_IFDIR : S_IFREG;
    st->st_mode |= (f.fattrib & AM_RDO) ?
                   (S_IRUSR | S_IXUSR | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH) :
                   (S_IRWXU | S_IRWXG | S_IRWXO);
    return 0;
}

/** Check mount identity and latched FatFs errors while the caller holds the mutex. */
static FRESULT fat_validate_file(const FIL *fp) {
    // A remount changes the volume ID, invalidating handles from the old mount.
    if (fp == NULL || fp->obj.fs == NULL || fp->obj.fs->fs_type == 0 ||
        fp->obj.fs->id != fp->obj.id) {
        return FR_INVALID_OBJECT;
    }
    // Deferred cursor operations must not hide an earlier FatFs abort.
    return (FRESULT)fp->err;
}

/** Return the file-size ceiling imposed by both the format and signed VFS offsets. */
static FSIZE_t fat_max_file_size(const FIL *fp) {
    // FAT12/16/32 store 32-bit sizes. Even exFAT must leave size and cursor
    // representable by the public off_t API after a successful extension.
    const FSIZE_t format_limit =
        fp->obj.fs != NULL && fp->obj.fs->fs_type != FS_EXFAT ? UINT32_MAX : (FSIZE_t)-1;
    return format_limit < FAT_OFF_MAX ? format_limit : FAT_OFF_MAX;
}

/** Bound one FatFs transfer, whose UINT length may be narrower than size_t. */
static UINT fat_chunk_size(size_t remaining) {
#if SIZE_MAX > UINT_MAX
    if (remaining > (size_t)UINT_MAX) {
        return UINT_MAX;
    }
#endif
    return (UINT)remaining;
}

/**
 * Precheck growth capacity for write/truncate; the caller holds the volume mutex.
 * FatFs may use a cached free-cluster count, so this is not an allocation guarantee.
 */
static int fat_ensure_capacity(const filesystem_fat_context_t *context,
                               const FIL *fp,
                               FSIZE_t old_size,
                               FSIZE_t target_size) {
    const FATFS *fatfs = fp->obj.fs;
    const FSIZE_t cluster_bytes =
        (FSIZE_t)fatfs->csize * disk_get_sector_size(fatfs->pdrv);
    // Round both sizes up: extending within the last cluster needs no new space.
    const FSIZE_t old_clusters =
        old_size == 0 ? 0 : (old_size - 1) / cluster_bytes + 1;
    const FSIZE_t new_clusters =
        target_size == 0 ? 0 : (target_size - 1) / cluster_bytes + 1;
    if (new_clusters <= old_clusters) {
        return 0;
    }

    // Ask FatFs about this mounted volume, rather than the block device's capacity.
    char drive[3] = "0:";
    drive[0] = (char)('0' + context->id);
    DWORD free_clusters = 0;
    FATFS *unused_fs = NULL;
    FRESULT res = f_getfree(drive, &free_clusters, &unused_fs);
    if (res != FR_OK) {
        return fat_error_remap(res);
    }
    if (new_clusters - old_clusters > (FSIZE_t)free_clusters) {
        return -ENOSPC;
    }
    return 0;
}

/**
 * Zero-fill growth for truncate and gap writes while the caller holds the mutex.
 * Completed growth is not rolled back; the logical cursor is never changed.
 */
static int fat_zero_extend(FIL *fp, FSIZE_t target_size) {
    static const uint8_t zeros[512] = {0};

    // Seek only to existing EOF, then write every newly exposed byte. Seeking
    // directly to the target would allocate space without initializing its contents.
    FRESULT res = f_lseek(fp, f_size(fp));
    if (res != FR_OK) {
        return fat_error_remap(res);
    }
    while (f_tell(fp) < target_size) {
        const FSIZE_t remaining = target_size - f_tell(fp);
        UINT chunk = remaining < sizeof(zeros) ? (UINT)remaining : sizeof(zeros);
        // End at sector boundaries so a later flush cannot abort a call after
        // growing the file but before FatFs marks its metadata modified.
        const UINT sector_size = disk_get_sector_size(fp->obj.fs->pdrv);
        const UINT sector_remaining = sector_size - (UINT)(f_tell(fp) % sector_size);
        if (chunk > sector_remaining) {
            chunk = sector_remaining;
        }
        UINT written = 0;
        res = f_write(fp, zeros, chunk, &written);
        if (res != FR_OK) {
            return fat_error_remap(res);
        }
        if (written < chunk) {
            // A capacity precheck can be stale. Keep only the bytes FatFs accepted;
            // never fabricate sizes or clear a hard-error latch to force rollback.
            return -ENOSPC;
        }
    }
    return 0;
}

/** Open a VFS file and initialize its independent cursor and append policy. */
static int file_open(filesystem_t *fs, fs_file_t *file, const char *path, int flags) {
    // Translate access/create flags, but implement append in file_write itself.
    const bool exclusive_create = (flags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL);
    BYTE open_mode;
    if (flags & O_RDWR)
        open_mode = FA_READ | FA_WRITE;
    else if (flags & O_WRONLY)
        open_mode = FA_WRITE;
    else
        open_mode = FA_READ;
    if (flags & O_CREAT) {
        if (exclusive_create)
            open_mode |= FA_CREATE_NEW;
        else if (flags & O_TRUNC)
            open_mode |= FA_CREATE_ALWAYS;
        else
            open_mode |= FA_OPEN_ALWAYS;
    }

    char fpath[PATH_MAX];
    filesystem_fat_context_t *context = fs->context;
    fat_path_prefix(fpath, context->id, path);
    fat_file_t *fat_file = file->context = calloc(1, sizeof(fat_file_t));
    if (fat_file == NULL) {
        fprintf(stderr, "file_open: Out of memory\n");
        return -ENOMEM;
    }

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = f_open(&fat_file->file, fpath, open_mode);
    if (exclusive_create) {
        // FatFs rejects read-only/directory targets before checking FA_CREATE_NEW.
        // Origin directories also fail f_stat, so recognize them with f_opendir.
        if (res == FR_DENIED && f_stat(fpath, NULL) == FR_OK) {
            res = FR_EXIST;
        } else if (res == FR_INVALID_NAME && path[0] != '\0') {
            FATFS_DIR dir = {0};
            if (f_opendir(&dir, fpath) == FR_OK) {
                (void)f_closedir(&dir);
                res = FR_EXIST;
            }
        }
    }
    if (res == FR_OK) {
        // Append handles initially report EOF; later writes choose the current
        // EOF again, even if the logical cursor has moved or the file shrank.
        fat_file->append = (flags & O_APPEND) != 0;
        fat_file->pos = fat_file->append ? f_size(&fat_file->file) : f_tell(&fat_file->file);
    }
    mutex_exit(&context->_mutex);

    if (res != FR_OK) {
        debug_if(FFS_DBG, "f_open('w') failed: %d\n", res);
        // Failed opens have no close callback, so release the private handle here.
        free(file->context);
        file->context = NULL;
        return fat_error_remap(res);
    }
    return 0;
}

static int file_close(filesystem_t *fs, fs_file_t *file) {
    (void)fs;
    filesystem_fat_context_t *context = fs->context;
    fat_file_t *fat_file = file->context;

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = f_close(&fat_file->file);
    mutex_exit(&context->_mutex);

    free(file->context);
    file->context = NULL;
    return fat_error_remap(res);
}

/** Write at the logical cursor or append EOF, retaining partial transfers and syncing metadata. */
static ssize_t file_write(filesystem_t *fs, fs_file_t *file, const void *buffer, size_t size) {
    (void)fs;
    filesystem_fat_context_t *context = fs->context;
    fat_file_t *fat_file = file->context;

    // Hold one mutex across validation, allocation, transfer, and metadata sync.
    mutex_enter_blocking(&context->_mutex);
    FRESULT res = fat_validate_file(&fat_file->file);
    if (res != FR_OK) {
        mutex_exit(&context->_mutex);
        return fat_error_remap(res);
    }
    if (!(fat_file->file.flag & FA_WRITE)) {
        mutex_exit(&context->_mutex);
        return fat_error_remap(FR_DENIED);
    }
    if (size == 0) {
        mutex_exit(&context->_mutex);
        return 0;
    }

    // Append ignores a prior seek. Bound payload by both file-size ceilings
    // and the signed byte-count return type.
    const FSIZE_t old_size = f_size(&fat_file->file);
    const FSIZE_t write_pos = fat_file->append ? old_size : fat_file->pos;
    const FSIZE_t max_size = fat_max_file_size(&fat_file->file);
    if (write_pos >= max_size) {
        mutex_exit(&context->_mutex);
        return -EFBIG;
    }

    size_t to_write = size > FAT_SSIZE_MAX ? FAT_SSIZE_MAX : size;
    const FSIZE_t max_avail = max_size - write_pos;
    if ((FSIZE_t)to_write > max_avail) {
        to_write = (size_t)max_avail;
    }

    if (write_pos > old_size) {
        // Check reported capacity for the gap and at least one byte of payload.
        // This avoids predictable gap-only growth, but is not an atomic guarantee.
        int err = fat_ensure_capacity(context, &fat_file->file, old_size, write_pos + 1);
        if (err != 0) {
            mutex_exit(&context->_mutex);
            return err;
        }
        // Internal zero writes do not count as payload or consume the logical cursor.
        err = fat_zero_extend(&fat_file->file, write_pos);
        if (err != 0) {
            mutex_exit(&context->_mutex);
            return err;
        }
    } else if (f_tell(&fat_file->file) != write_pos) {
        // Align the physical cursor only when an actual transfer needs it.
        res = f_lseek(&fat_file->file, write_pos);
        if (res != FR_OK) {
            mutex_exit(&context->_mutex);
            debug_if(FFS_DBG, "f_write() failed: %d", res);
            return fat_error_remap(res);
        }
    }

    // Split requests at FatFs's UINT limit and retain completed transfers.
    size_t total_written = 0;
    while (total_written < to_write) {
        UINT chunk = fat_chunk_size(to_write - total_written);
        UINT n = 0;
        res = f_write(&fat_file->file, (const uint8_t *)buffer + total_written, chunk, &n);
        total_written += n;
        // Completed bytes consume the cursor even if this transfer or a later sync fails.
        if (n != 0) {
            fat_file->pos = f_tell(&fat_file->file);
        }
        if (res != FR_OK) {
            if (total_written > 0) {
                // Publish the accepted prefix before returning a partial count.
                // f_sync permits an aborted handle without clearing its error latch;
                // failed synchronization remains pending for file_sync or close.
                (void)f_sync(&fat_file->file);
            }
            mutex_exit(&context->_mutex);
            debug_if(FFS_DBG, "f_write() failed: %d", res);
            return total_written > 0 ? (ssize_t)total_written : fat_error_remap(res);
        }
        if (n == 0) {
            if (total_written == 0) {
                // No payload fits. Retain any completed zero-filled gap rather
                // than inferring allocation ownership from size for rollback.
                mutex_exit(&context->_mutex);
                return -ENOSPC;
            }
            break;
        }
        if (n < chunk) {
            break;
        }
    }

    // Publish directory metadata independently of the already-consumed cursor.
    // A sync error is still reported, but it cannot undo the preceding transfer.
    res = f_sync(&fat_file->file);
    mutex_exit(&context->_mutex);

    if (res != FR_OK) {
        debug_if(FFS_DBG, "f_write() failed: %d", res);
        return fat_error_remap(res);
    }
    return (ssize_t)total_written;
}

/** Read from the logical cursor, preserving signed offset bounds, EOF, and partial reads. */
static ssize_t file_read(filesystem_t *fs, fs_file_t *file, void *buffer, size_t size) {
    (void)fs;
    filesystem_fat_context_t *context = fs->context;
    fat_file_t *fat_file = file->context;

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = fat_validate_file(&fat_file->file);
    if (res != FR_OK) {
        mutex_exit(&context->_mutex);
        return fat_error_remap(res);
    }
    if (!(fat_file->file.flag & FA_READ)) {
        mutex_exit(&context->_mutex);
        return fat_error_remap(FR_DENIED);
    }

    // An overhanging cursor is valid after seek or truncate. Return EOF without
    // passing it to FatFs, where a writable seek could extend the file.
    const FSIZE_t cur_size = f_size(&fat_file->file);
    if (size == 0 || fat_file->pos >= cur_size) {
        mutex_exit(&context->_mutex);
        return 0;
    }

    // Imported files can exceed off_t even though local growth cannot. Refuse
    // to consume data when advancing the cursor would make it unrepresentable.
    if (fat_file->pos >= FAT_OFF_MAX) {
        mutex_exit(&context->_mutex);
        return -EOVERFLOW;
    }

    // Bound the request by EOF, the signed byte count, and cursor headroom.
    size_t to_read = size > FAT_SSIZE_MAX ? FAT_SSIZE_MAX : size;
    const FSIZE_t avail = cur_size - fat_file->pos;
    if ((FSIZE_t)to_read > avail) {
        to_read = (size_t)avail;
    }
    const FSIZE_t offset_avail = FAT_OFF_MAX - fat_file->pos;
    if ((FSIZE_t)to_read > offset_avail) {
        to_read = (size_t)offset_avail;
    }
    if (f_tell(&fat_file->file) != fat_file->pos) {
        res = f_lseek(&fat_file->file, fat_file->pos);
        if (res != FR_OK) {
            mutex_exit(&context->_mutex);
            debug_if(FFS_DBG, "f_read() failed: %d\n", res);
            return fat_error_remap(res);
        }
    }

    // Transfer in FatFs-sized chunks. Even an error can follow valid data, so
    // retain the consumed cursor and return those bytes before reporting failure.
    size_t total_read = 0;
    while (total_read < to_read) {
        UINT chunk = fat_chunk_size(to_read - total_read);
        UINT n = 0;
        res = f_read(&fat_file->file, (uint8_t *)buffer + total_read, chunk, &n);
        total_read += n;
        fat_file->pos = f_tell(&fat_file->file);
        if (res != FR_OK) {
            mutex_exit(&context->_mutex);
            debug_if(FFS_DBG, "f_read() failed: %d\n", res);
            return total_read > 0 ? (ssize_t)total_read : fat_error_remap(res);
        }
        if (n < chunk) {
            break;
        }
    }
    mutex_exit(&context->_mutex);
    return (ssize_t)total_read;
}

static int file_sync(filesystem_t *fs, fs_file_t *file) {
    (void)fs;
    filesystem_fat_context_t *context = fs->context;
    fat_file_t *fat_file = file->context;

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = f_sync(&fat_file->file);
    mutex_exit(&context->_mutex);

    if (res != FR_OK) {
        debug_if(FFS_DBG, "f_sync() failed: %d\n", res);
    }
    return fat_error_remap(res);
}

/** Change the VFS cursor without letting a FatFs seek allocate or extend the file. */
static off_t file_seek(filesystem_t *fs, fs_file_t *file, off_t offset, int whence) {
    (void)fs;
    filesystem_fat_context_t *context = fs->context;
    fat_file_t *fat_file = file->context;

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = fat_validate_file(&fat_file->file);
    if (res != FR_OK) {
        mutex_exit(&context->_mutex);
        return fat_error_remap(res);
    }

    // Resolve the origin against logical position or live size, not FIL.fptr.
    FSIZE_t base;
    if (whence == SEEK_SET) {
        base = 0;
    } else if (whence == SEEK_CUR) {
        base = fat_file->pos;
    } else if (whence == SEEK_END) {
        base = f_size(&fat_file->file);
    } else {
        mutex_exit(&context->_mutex);
        return -EINVAL;
    }

    FSIZE_t target;
    if (offset < 0) {
        // Unsigned subtraction handles the most-negative off_t without negating it.
        FSIZE_t delta = (FSIZE_t)0 - (FSIZE_t)offset;
        if (delta > base) {
            mutex_exit(&context->_mutex);
            return -EINVAL;
        }
        target = base - delta;
    } else {
        // Check the sum before adding, then ensure the public result fits off_t.
        FSIZE_t delta = (FSIZE_t)offset;
        if (base > FAT_OFF_MAX || delta > FAT_OFF_MAX - base) {
            mutex_exit(&context->_mutex);
            return -EOVERFLOW;
        }
        target = base + delta;
    }
    if (target > FAT_OFF_MAX) {
        mutex_exit(&context->_mutex);
        return -EOVERFLOW;
    }

    // Defer physical positioning until read/write; seeking alone must not grow a file.
    fat_file->pos = target;
    mutex_exit(&context->_mutex);
    return (off_t)target;
}

/** Return a validated logical cursor, rejecting values outside signed off_t. */
static off_t file_tell(filesystem_t *fs, fs_file_t *file) {
    (void)fs;
    filesystem_fat_context_t *context = fs->context;
    fat_file_t *fat_file = file->context;

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = fat_validate_file(&fat_file->file);
    FSIZE_t value = fat_file->pos;
    mutex_exit(&context->_mutex);

    if (res != FR_OK) {
        return fat_error_remap(res);
    }
    return value > FAT_OFF_MAX ? -EOVERFLOW : (off_t)value;
}

/** Return the live FatFs file size, independently of the logical cursor. */
static off_t file_size(filesystem_t *fs, fs_file_t *file) {
    (void)fs;
    filesystem_fat_context_t *context = fs->context;
    fat_file_t *fat_file = file->context;

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = fat_validate_file(&fat_file->file);
    FSIZE_t value = f_size(&fat_file->file);
    mutex_exit(&context->_mutex);

    if (res != FR_OK) {
        return fat_error_remap(res);
    }
    return value > FAT_OFF_MAX ? -EOVERFLOW : (off_t)value;
}

/** Resize and sync a VFS file without changing its logical cursor. */
static int file_truncate(filesystem_t *fs, fs_file_t *file, off_t length) {
    (void)fs;
    if (length < 0) {
        return -EINVAL;
    }

    filesystem_fat_context_t *context = fs->context;
    fat_file_t *fat_file = file->context;

    // Reject invalid handles, access modes, and format limits before changing storage.
    mutex_enter_blocking(&context->_mutex);
    FRESULT res = fat_validate_file(&fat_file->file);
    if (res != FR_OK) {
        mutex_exit(&context->_mutex);
        return fat_error_remap(res);
    }
    if (!(fat_file->file.flag & FA_WRITE)) {
        mutex_exit(&context->_mutex);
        return -EINVAL;
    }

    const FSIZE_t target_size = (FSIZE_t)length;
    if (target_size > fat_max_file_size(&fat_file->file)) {
        mutex_exit(&context->_mutex);
        return -EFBIG;
    }

    const FSIZE_t old_size = f_size(&fat_file->file);
    if (target_size < old_size) {
        // FatFs truncates at its physical cursor, so position it at the new EOF.
        if (f_tell(&fat_file->file) != target_size) {
            res = f_lseek(&fat_file->file, target_size);
            if (res != FR_OK) {
                mutex_exit(&context->_mutex);
                return fat_error_remap(res);
            }
        }
        res = f_truncate(&fat_file->file);
        if (res != FR_OK) {
            mutex_exit(&context->_mutex);
            return fat_error_remap(res);
        }
    } else if (target_size > old_size) {
        // Precheck before writing zeros; preserve the logical cursor even if
        // initialization fails after extending part of the file.
        int err = fat_ensure_capacity(context, &fat_file->file, old_size, target_size);
        if (err != 0) {
            mutex_exit(&context->_mutex);
            return err;
        }
        err = fat_zero_extend(&fat_file->file, target_size);
        if (err != 0) {
            mutex_exit(&context->_mutex);
            return err;
        }
    }

    // Publish the size for path stat/VFS fstat. Do not restore FIL.fptr to the
    // logical cursor: if it is beyond the new EOF, doing so would re-extend the file.
    res = f_sync(&fat_file->file);
    mutex_exit(&context->_mutex);
    return fat_error_remap(res);
}

static int dir_open(filesystem_t *fs, fs_dir_t *dir, const char *path) {
    filesystem_fat_context_t *context = fs->context;
    char fpath[PATH_MAX];
    fat_path_prefix(fpath, context->id, path);

    FATFS_DIR *dh = calloc(1, sizeof(FATFS_DIR));
    if (dh == NULL) {
        fprintf(stderr, "dir_open: Out of memory\n");
        return -ENOMEM;
    }

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = f_opendir(dh, fpath);
    mutex_exit(&context->_mutex);

    if (res != FR_OK) {
        debug_if(FFS_DBG, "f_opendir() failed: %d\n", res);
        free(dh);
        return fat_error_remap(res);
    }
    dir->context = dh;
    dir->fd = -1;
    return 0;
}

static int dir_close(filesystem_t *fs, fs_dir_t *dir) {
    (void)fs;
    filesystem_fat_context_t *context = fs->context;

    FATFS_DIR *dh = (FATFS_DIR *)dir->context;

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = f_closedir(dh);
    mutex_exit(&context->_mutex);

    free(dh);
    return fat_error_remap(res);
}

static int dir_read(filesystem_t *fs, fs_dir_t *dir, struct dirent *ent) {
    (void)fs;
    filesystem_fat_context_t *context = fs->context;

    FATFS_DIR *dh = (FATFS_DIR *)dir->context;
    FILINFO finfo = {0};

    mutex_enter_blocking(&context->_mutex);
    FRESULT res = f_readdir(dh, &finfo);
    mutex_exit(&context->_mutex);

    if (res != FR_OK) {
        return fat_error_remap(res);
    } else if (finfo.fname[0] == 0) {
        return -ENOENT;
    }
    ent->d_type = (finfo.fattrib & AM_DIR) ? DT_DIR : DT_REG;
    strncpy(ent->d_name, finfo.fname, FF_LFN_BUF);
    ent->d_name[FF_LFN_BUF] = '\0';
    return 0;
}


filesystem_t *filesystem_fat_create() {
    filesystem_t *fs = calloc(1, sizeof(filesystem_t));
    if (fs == NULL) {
        fprintf(stderr, "filesystem_fat_create: Out of memory\n");
        return NULL;
    }
    fs->type = FILESYSTEM_TYPE_FAT;
    fs->name = FILESYSTEM_NAME;
    fs->mount = mount;
    fs->unmount = unmount;
    fs->format = format;
    fs->remove = file_remove;
    fs->rename = file_rename;
    fs->mkdir = file_mkdir;
    fs->rmdir = file_rmdir;
    fs->stat = file_stat;
    fs->file_open = file_open;
    fs->file_close = file_close;
    fs->file_write = file_write;
    fs->file_read = file_read;
    fs->file_sync = file_sync;
    fs->file_seek = file_seek;
    fs->file_tell = file_tell;
    fs->file_size = file_size;
    fs->file_truncate = file_truncate;
    fs->dir_open = dir_open;
    fs->dir_close = dir_close;
    fs->dir_read = dir_read;
    filesystem_fat_context_t *context = calloc(1, sizeof(filesystem_fat_context_t));
    if (context == NULL) {
        fprintf(stderr, "filesystem_fat_create: Out of memory\n");
        free(fs);
        return NULL;
    }
    context->id = -1;
    mutex_init(&context->_mutex);
    mutex_init(&context->_mutex_format);

    fs->context = context;
    return fs;
}

void filesystem_fat_free(filesystem_t *fs) {
    free(fs->context);
    fs->context = NULL;
    free(fs);
    fs = NULL;
}
