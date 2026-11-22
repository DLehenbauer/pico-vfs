#include <stdio.h>
#include <string.h>
#include "blockdevice/heap.h"
#include "blockdevice/loopback.h"
#include "filesystem/fat.h"
#include "filesystem/littlefs.h"
#include "filesystem/vfs.h"
#include "pico/mutex.h"
#include <sys/dirent.h>

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);       \
            exit(EXIT_FAILURE);                                                 \
        }                                                                       \
    } while (0)

static unsigned max_recursive_depth;
static unsigned active_recursive_depth;

int _open(const char *path, int flags, ...);
int _close(int fd);
int _unlink(const char *path);
ssize_t _read(int fd, void *buffer, size_t size);
ssize_t _write(int fd, const void *buffer, size_t size);
off_t _lseek(int fd, off_t offset, int whence);

// Host libc does not call Newlib's syscall hooks. Bridge only file operations
// from this executable and the production loopback driver into the real VFS.
int __wrap_open(const char *path, int flags, ...) { return _open(path, flags); }
int __wrap_close(int fd) { return _close(fd); }
int __wrap_unlink(const char *path) { return _unlink(path); }
ssize_t __wrap_read(int fd, void *buffer, size_t size) {
    return _read(fd, buffer, size);
}
ssize_t __wrap_write(int fd, const void *buffer, size_t size) {
    return _write(fd, buffer, size);
}
off_t __wrap_lseek(int fd, off_t offset, int whence) {
    return _lseek(fd, offset, whence);
}

void mutex_init(mutex_t *mutex) { mutex->locked = false; }
void mutex_enter_blocking(mutex_t *mutex) {
    CHECK(!mutex->locked);
    mutex->locked = true;
}
void mutex_exit(mutex_t *mutex) {
    CHECK(mutex->locked);
    mutex->locked = false;
}
void recursive_mutex_enter_blocking(recursive_mutex_t *mutex) {
    ++mutex->depth;
    ++active_recursive_depth;
    if (mutex->depth > max_recursive_depth) max_recursive_depth = mutex->depth;
}
void recursive_mutex_exit(recursive_mutex_t *mutex) {
    CHECK(mutex->depth != 0);
    CHECK(active_recursive_depth != 0);
    --mutex->depth;
    --active_recursive_depth;
}
void stdio_flush(void) { fflush(stdout); }

static void test_exclusive_create(const char *prefix) {
    static const char payload[] = "preserved";
    static const int variants[] = {0, O_TRUNC, O_APPEND, O_TRUNC | O_APPEND};
    const char *mountpoint = prefix[0] == '\0' ? "/" : prefix;
    char path[64] = {0}, recovery[64] = {0}, directory[64] = {0};
    CHECK(snprintf(path, sizeof(path), "%s/exclusive.bin", prefix) > 0);
    CHECK(snprintf(recovery, sizeof(recovery), "%s/recovery.bin", prefix) > 0);
    CHECK(snprintf(directory, sizeof(directory), "%s/dir", prefix) > 0);
    const int flags = O_CREAT | O_EXCL | O_RDWR;
    int original = open(path, flags, 0644);
    CHECK(original >= 3);
    CHECK(write(original, payload, sizeof(payload)) == (ssize_t)sizeof(payload));

    for (size_t i = 0; i < sizeof(variants) / sizeof(variants[0]); ++i) {
        errno = 0;
        CHECK(open(path, flags | variants[i], 0644) == -1);
        CHECK(errno == EEXIST);
        errno = 0;
        CHECK(open(mountpoint, flags | variants[i], 0644) == -1);
        CHECK(errno == EEXIST);
        int next = open(recovery, flags, 0644);
        CHECK(next == original + 1);
        CHECK(close(next) == 0);
        CHECK(unlink(recovery) == 0);
    }
    CHECK(lseek(original, 0, SEEK_SET) == 0);
    char actual[sizeof(payload) + 1] = {0};
    CHECK(read(original, actual, sizeof(actual)) == (ssize_t)sizeof(payload));
    CHECK(memcmp(actual, payload, sizeof(payload)) == 0);
    CHECK(close(original) == 0);

    for (size_t i = 0; i < sizeof(variants) / sizeof(variants[0]); ++i) {
        errno = 0;
        CHECK(open(path, flags | variants[i], 0644) == -1);
        CHECK(errno == EEXIST);
        int next = open(path, O_RDONLY);
        CHECK(next == original);
        memset(actual, 0, sizeof(actual));
        CHECK(read(next, actual, sizeof(actual)) == (ssize_t)sizeof(payload));
        CHECK(memcmp(actual, payload, sizeof(payload)) == 0);
        CHECK(close(next) == 0);
    }
    CHECK(unlink(path) == 0);

    CHECK(mkdir(directory, 0777) == 0);
    errno = 0;
    CHECK(open(directory, flags, 0644) == -1);
    CHECK(errno == EEXIST);
    DIR *dir = opendir(directory);
    CHECK(dir != NULL);
    CHECK(closedir(dir) == 0);
    int next = open(recovery, flags, 0644);
    CHECK(next == original);
    CHECK(write(next, payload, sizeof(payload)) == (ssize_t)sizeof(payload));
    CHECK(close(next) == 0);
    CHECK(unlink(recovery) == 0);
    CHECK(rmdir(directory) == 0);
    CHECK(active_recursive_depth == 0);
}

static void test_heap_backend(bool fat) {
    blockdevice_t *heap = blockdevice_heap_create(128u * 1024u);
    filesystem_t *fs = fat ? filesystem_fat_create() : filesystem_littlefs_create(500, 16);
    CHECK(heap != NULL && fs != NULL);
    CHECK(fs_format(fs, heap) == 0);
    CHECK(fs_mount("/", fs, heap) == 0);
    fs_file_t empty = {0};
    CHECK(fs->file_open(fs, &empty, "", O_CREAT | O_EXCL | O_RDWR) == -EINVAL);
    test_exclusive_create("");
    CHECK(fs_unmount("/") == 0);
    errno = 0;
    CHECK(open("/unmounted", O_RDONLY) == -1);
    CHECK(errno == ENOENT);
    if (fat) filesystem_fat_free(fs);
    else filesystem_littlefs_free(fs);
    blockdevice_heap_free(heap);
    printf("vfs-%s: passed\n", fat ? "fat" : "littlefs");
}

static void test_loopback_backend(void) {
    blockdevice_t *heap = blockdevice_heap_create(256u * 1024u);
    filesystem_t *outer = filesystem_littlefs_create(500, 16);
    filesystem_t *inner = filesystem_fat_create();
    CHECK(heap != NULL && outer != NULL && inner != NULL);
    CHECK(fs_format(outer, heap) == 0);
    CHECK(fs_mount("/backing", outer, heap) == 0);
    blockdevice_t *loopback = blockdevice_loopback_create("/backing/image.bin", 64u * 1024u, 512);
    CHECK(loopback != NULL);
    CHECK(fs_format(inner, loopback) == 0);
    CHECK(fs_mount("/disk", inner, loopback) == 0);
    max_recursive_depth = 0;
    test_exclusive_create("/disk");
    CHECK(max_recursive_depth > 1);
    CHECK(fs_unmount("/disk") == 0);
    blockdevice_loopback_free(loopback);
    filesystem_fat_free(inner);
    CHECK(unlink("/backing/image.bin") == 0);
    CHECK(fs_unmount("/backing") == 0);
    errno = 0;
    CHECK(open("/backing/image.bin", O_RDONLY) == -1);
    CHECK(errno == ENOENT);
    filesystem_littlefs_free(outer);
    blockdevice_heap_free(heap);
    printf("vfs-loopback-fat-on-littlefs: passed\n");
}

int main(void) {
    errno = 0;
    CHECK(open("/unmounted", O_RDONLY) == -1);
    CHECK(errno == ENOENT);
    test_heap_backend(true);
    test_heap_backend(false);
    test_loopback_backend();
    CHECK(active_recursive_depth == 0);
    return 0;
}
