#include "blockdevice/heap.h"
#include "filesystem/fat.h"

static blockdevice_t *sd_card;

static int trim_sd_card(blockdevice_t *device, bd_size_t addr, bd_size_t size) {
    (void)device;
    return sd_card->trim(sd_card, addr, size);
}

int test_sd_fat_format(blockdevice_t *sd) {
    blockdevice_t *storage = blockdevice_heap_create(sd->size(sd));
    if (storage == NULL) {
        return -ENOMEM;
    }
    filesystem_t *fs = filesystem_fat_create();
    if (fs == NULL) {
        blockdevice_heap_free(storage);
        return -ENOMEM;
    }

    // Keep filesystem reads/writes in RAM, but exercise real SD trim and its
    // SPI expectations with exactly the geometry seen by filesystem callers.
    storage->read_size = sd->read_size;
    storage->program_size = sd->program_size;
    storage->erase_size = sd->erase_size;
    storage->erase = sd->erase;
    storage->trim = trim_sd_card;
    sd_card = sd;
    int status = fs->format(fs, storage);
    sd_card = NULL;

    filesystem_fat_free(fs);
    blockdevice_heap_free(storage);
    return status;
}
