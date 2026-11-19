#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONF_SD_CMD_TIMEOUT 5
#include "../../src/blockdevice/sd.c"

int test_sd_fat_format(blockdevice_t *sd);

static const char *current_case;
static unsigned initialized_baudrate;
static unsigned transfer_baudrate;
static unsigned format_calls;

#define CHECK(condition)                                                                           \
    do {                                                                                           \
        if (!(condition)) {                                                                        \
            fprintf(stderr, "%s: %s:%d: %s (event %zu)\n", current_case, __FILE__, __LINE__,          \
                    #condition, event_index);                                                      \
            exit(EXIT_FAILURE);                                                                    \
        }                                                                                          \
    } while (0)

#define RUN_CASE(name, test)                                                                        \
    do {                                                                                           \
        current_case = name;                                                                       \
        test;                                                                                      \
        printf("%s: passed\n", current_case);                                                       \
    } while (0)

typedef struct {
    bool gpio;
    bool cs;
    uint8_t tx;
    uint8_t rx;
    size_t count;
} event_t;

static event_t events[4096];
static size_t event_count;
static size_t event_index;
static bool cs_high;
static uint64_t now_us;
static uint64_t time_step_us;
static bool startup_pending;
static blockdevice_sd_config_t config;
static blockdevice_t device;
static unsigned mutex_entries;
static unsigned mutex_exits;
static mutex_t *expected_mutex;
static bool observing_lifecycle;
static bool initializing;
static bool entry_config_initialized;
static bool entry_device_initialized;
static bool initialized_on_unlock;
static bool observing_ready_io;
static bool filesystem_io;
spi_inst_t mock_spi_instances[2] = {{0}, {1}};

static void finish(int actual, int expected);
static void expect_init_command(int cmd, uint32_t arg, uint8_t response, uint32_t extra);

static void check_spi_state(void) {
    if ((observing_lifecycle && initializing) || observing_ready_io) {
        CHECK(config._mutex.locked);
        CHECK(config.is_initialized == observing_ready_io);
        CHECK(device.is_initialized == observing_ready_io);
    }
}

static void expect_spi(uint8_t tx, uint8_t rx, bool cs, size_t count) {
    CHECK(event_count < sizeof(events) / sizeof(events[0]));
    events[event_count++] = (event_t){.cs = cs, .tx = tx, .rx = rx, .count = count};
}

static void expect_response_bits(const uint8_t *data, size_t bits, unsigned leading, bool busy) {
    size_t clocks = ((leading + bits + 7) / 8) * 8;
    uint8_t byte = 0;
    for (size_t bit = 0; bit < clocks; ++bit) {
        bool high = bit < leading || !busy;
        if (bit >= leading && bit < leading + bits) {
            size_t index = bit - leading;
            high = data[index / 8] & (0x80u >> (index % 8));
        }
        byte = (byte << 1) | high;
        if (bit % 8 == 7) {
            expect_spi(0xFF, byte, false, 1);
        }
    }
}

static void expect_cs(bool cs) {
    CHECK(event_count < sizeof(events) / sizeof(events[0]));
    events[event_count++] = (event_t){.gpio = true, .cs = cs, .count = 1};
}

static uint8_t exchange(uint8_t tx) {
    check_spi_state();
    if (startup_pending) {
        CHECK(cs_high);
        CHECK(now_us >= 1000);
        startup_pending = false;
    }
    CHECK(event_index < event_count);
    event_t *event = &events[event_index];
    CHECK(!event->gpio);
    CHECK(cs_high == event->cs);
    CHECK(tx == event->tx);
    uint8_t rx = event->rx;
    if (--event->count == 0) {
        ++event_index;
    }
    return rx;
}

unsigned spi_init(spi_inst_t *spi, unsigned baudrate) {
    check_spi_state();
    CHECK(spi == config.spi_inst);
    initialized_baudrate = baudrate;
    return baudrate;
}

unsigned spi_set_baudrate(spi_inst_t *spi, unsigned baudrate) {
    check_spi_state();
    CHECK(spi == config.spi_inst);
    transfer_baudrate = baudrate;
    return baudrate;
}

void spi_set_format(spi_inst_t *spi, unsigned bits, unsigned cpol, unsigned cpha, unsigned order) {
    check_spi_state();
    CHECK(spi == config.spi_inst);
    CHECK(bits == 8 && cpol == SPI_CPOL_0 && cpha == SPI_CPHA_0 && order == SPI_MSB_FIRST);
    ++format_calls;
}

int spi_write_read_blocking(spi_inst_t *spi, const uint8_t *src, uint8_t *dst, size_t len) {
    CHECK(spi == config.spi_inst);
    for (size_t i = 0; i < len; ++i) {
        uint8_t rx = exchange(src[i]);
        if (dst) {
            dst[i] = rx;
        }
    }
    return (int)len;
}

int spi_read_blocking(spi_inst_t *spi, uint8_t repeated_tx_data, uint8_t *dst, size_t len) {
    CHECK(spi == config.spi_inst);
    for (size_t i = 0; i < len; ++i) {
        dst[i] = exchange(repeated_tx_data);
    }
    return (int)len;
}

int spi_write_blocking(spi_inst_t *spi, const uint8_t *src, size_t len) {
    return spi_write_read_blocking(spi, src, NULL, len);
}

void gpio_set_dir(unsigned gpio, bool out) {
    CHECK(gpio == config.cs);
    CHECK(out == GPIO_OUT);
    CHECK(cs_high);
}

void gpio_put(unsigned gpio, bool value) {
    CHECK(gpio == config.cs);
    // CS expectations describe transitions, not repeated writes of the same level.
    if (value == cs_high) {
        return;
    }
    CHECK(event_index < event_count);
    event_t *event = &events[event_index++];
    CHECK(event->gpio);
    CHECK(value == event->cs);
    cs_high = value;
}

bool gpio_get(unsigned gpio) {
    CHECK(gpio == config.cs);
    return cs_high;
}

absolute_time_t get_absolute_time(void) {
    now_us += time_step_us;
    return now_us;
}

void sleep_ms(uint32_t ms) {
    now_us += (uint64_t)ms * 1000;
}

void mutex_init(mutex_t *mutex) {
    mutex->locked = false;
    if (!filesystem_io) {
        expected_mutex = mutex;
    }
}

void mutex_enter_blocking(mutex_t *mutex) {
    // FAT and heap add independent mutexes only in the integration scenario.
    if (!filesystem_io) {
        CHECK(mutex == expected_mutex);
    }
    CHECK(!mutex->locked);
    if (observing_lifecycle) {
        CHECK(config.is_initialized == entry_config_initialized);
        CHECK(device.is_initialized == entry_device_initialized);
    }
    ++mutex_entries;
    mutex->locked = true;
}

void mutex_exit(mutex_t *mutex) {
    if (!filesystem_io) {
        CHECK(mutex == expected_mutex);
    }
    CHECK(mutex->locked);
    if (observing_lifecycle) {
        CHECK(config.is_initialized == initialized_on_unlock);
        CHECK(device.is_initialized == initialized_on_unlock);
    }
    ++mutex_exits;
    mutex->locked = false;
}

static void reset_card(void) {
    event_count = event_index = 0;
    cs_high = true;
    now_us = 0;
    time_step_us = 1000;
    startup_pending = false;
    observing_lifecycle = observing_ready_io = false;
    filesystem_io = false;
    mutex_entries = mutex_exits = 0;
    initialized_baudrate = transfer_baudrate = format_calls = 0;
    config = (blockdevice_sd_config_t){
        .spi_inst = spi0,
        .cs = 17,
        .enable_crc = true,
        .card_type = SDCARD_V1,
        .is_initialized = true,
        .block_size = 512,
        .erase_size = 512,
        .total_sectors = 4096,
        .hz = 12000000,
    };
    device = (blockdevice_t){
        .config = &config,
        .read = read,
        .program = program,
        .trim = trim,
        .size = size,
        .read_size = 512,
        .program_size = 512,
        .erase_size = 512,
        .is_initialized = true,
    };
    mutex_init(&config._mutex);
}

static uint8_t reference_crc7(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        for (unsigned bit = 0; bit < 8; ++bit) {
            bool feedback = ((crc >> 6) ^ (data[i] >> (7 - bit))) & 1;
            crc = ((crc << 1) ^ (feedback ? 0x09 : 0)) & 0x7F;
        }
    }
    return (crc << 1) | 1;
}

static uint16_t reference_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
        }
    }
    return crc;
}

static void set_csd_bits(uint8_t *csd, unsigned msb, unsigned lsb, uint32_t value) {
    for (unsigned bit = lsb; bit <= msb; ++bit) {
        uint8_t mask = 1u << (bit % 8);
        csd[15 - bit / 8] = (csd[15 - bit / 8] & ~mask) |
                           ((value & (1u << (bit - lsb))) ? mask : 0);
    }
}

static void expect_select(void) {
    expect_spi(0xFF, 0xFF, true, 1);
    expect_cs(false);
}

static void expect_release(void) {
    // Eight selected bytes conservatively cover N_CR's 1-8 byte-times, not a
    // mandatory fixed delay; N_EC has a zero minimum (SanDisk v2.2,
    // section 5.23, p.5-17, and Table 5-5, p.5-19).
    // One deselected byte covers the one-clock DataOut release during R1b busy.
    // Eight deselected clocks are not a required minimum (SanDisk v2.2,
    // section 5.23.2, p.5-17).
    expect_spi(0xFF, 0xFF, false, 8);
    expect_cs(true);
    expect_spi(0xFF, 0xFF, true, 1);
}

static void expect_command_with_stuff(int cmd, uint32_t arg, uint8_t stuff) {
    expect_spi(0xFF, 0xFF, false, 1);
    uint8_t packet[6] = {
        (uint8_t)(0x40 | cmd), arg >> 24, arg >> 16, arg >> 8, arg, 0xFF,
    };
    // CMD0 before SPI entry, and CMD8 always, need valid CRC even with CRC off
    // (SD v3.01, section 7.2.2, p.116).
    if (config.enable_crc || cmd == CMD0_GO_IDLE_STATE || cmd == CMD8_SEND_IF_COND) {
        packet[5] = reference_crc7(packet, 5);
    }
    for (size_t i = 0; i < sizeof(packet); ++i) {
        // Tail bytes during command transmission are not response bytes.
        expect_spi(packet[i], 0x5A, false, 1);
    }
    expect_spi(0xFF, stuff, false, 1);
}

static void expect_command(int cmd, uint32_t arg) {
    // An error-looking first slot must never be treated as the actual R1.
    expect_command_with_stuff(cmd, arg, 0x04);
}

static void expect_start(size_t blocks, uint32_t address, uint8_t response) {
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(blocks > 1 ? 18 : 17, address);
    expect_spi(0xFF, response, false, 1);
}

static void expect_block_crc(uint8_t value, bool corrupt_crc) {
    uint8_t data[512];
    memset(data, value, sizeof(data));
    uint16_t crc = reference_crc16(data, sizeof(data)) ^ (corrupt_crc ? 1 : 0);
    expect_spi(0xFF, 0xFE, false, 1);
    expect_spi(0xFF, value, false, sizeof(data));
    expect_spi(0xFF, crc >> 8, false, 1);
    expect_spi(0xFF, crc, false, 1);
}

static void expect_block(uint8_t value) {
    // Deliberately corrupt CRC when checking that disabled CRC is still consumed.
    expect_block_crc(value, !config.enable_crc);
}

static void set_csd_bits(uint8_t *csd, unsigned msb, unsigned lsb, uint32_t value);
static void make_csd(uint8_t *csd);
static void expect_csd(const uint8_t *csd, bool corrupt_crc);

static void test_sectors(void) {
    uint8_t csd[16];
    reset_card();
    make_csd(csd);
    expect_csd(csd, false);
    CHECK(_sd_sectors(&config) == 4096);
    finish(BD_ERROR_OK, BD_ERROR_OK);

    reset_card();
    make_csd(csd);
    set_csd_bits(csd, 127, 126, 1);
    set_csd_bits(csd, 69, 48, 0x3FFFFF);
    expect_csd(csd, false);
    CHECK(_sd_sectors(&config) == (UINT64_C(1) << 32));
    finish(BD_ERROR_OK, BD_ERROR_OK);

    reset_card();
    make_csd(csd);
    set_csd_bits(csd, 127, 126, 2);
    expect_csd(csd, false);
    CHECK(_sd_sectors(&config) == 0);
    finish(BD_ERROR_OK, BD_ERROR_OK);

    reset_card();
    make_csd(csd);
    expect_csd(csd, true);
    CHECK(_sd_sectors(&config) == 0);
    finish(BD_ERROR_OK, BD_ERROR_OK);

    reset_card();
    expect_init_command(CMD9_SEND_CSD, 0, R1_ILLEGAL_COMMAND, 0);
    CHECK(_sd_sectors(&config) == 0);
    finish(BD_ERROR_OK, BD_ERROR_OK);

    reset_card();
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD9_SEND_CSD, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(0xFF, 0xFF, false, 300);
    expect_release();
    CHECK(_sd_sectors(&config) == 0);
    finish(BD_ERROR_OK, BD_ERROR_OK);
}

static void expect_program_data(uint8_t value, uint8_t start) {
    uint8_t buffer[512];
    memset(buffer, value, sizeof(buffer));
    uint16_t crc = reference_crc16(buffer, sizeof(buffer));
    expect_spi(start, 0xFF, false, 1);
    expect_spi(value, 0xFF, false, sizeof(buffer));
    expect_spi(crc >> 8, 0xFF, false, 1);
    expect_spi(crc, 0xFF, false, 1);
}

static void expect_program_block(uint8_t value, uint8_t start, uint8_t response,
                                bool timeout) {
    expect_program_data(value, start);
    expect_spi(0xFF, response, false, 1);
    expect_spi(0xFF, 0, false, timeout ? CONF_SD_CMD_TIMEOUT : 2);
    if (!timeout) {
        expect_spi(0xFF, 0xFF, false, 1);
    }
}

static void expect_multiblock_program(void) {
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD55_APP_CMD, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(ACMD23_SET_WR_BLK_ERASE_COUNT, 2);
    expect_spi(0xFF, 0, false, 1);
    expect_release();
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD25_WRITE_MULTIPLE_BLOCK, 0);
    expect_spi(0xFF, 0, false, 1);
}

static void fill_pattern(uint8_t *buffer, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        buffer[i] = i * 37 + (i / 256) * 17 + 11;
    }
}

static void expect_status_block(const uint8_t *data, size_t length, bool corrupt_crc) {
    uint16_t crc = reference_crc16(data, length) ^ (corrupt_crc ? 1 : 0);
    expect_spi(0xFF, SPI_START_BLOCK, false, 1);
    for (size_t i = 0; i < length; ++i) {
        expect_spi(0xFF, data[i], false, 1);
    }
    expect_spi(0xFF, crc >> 8, false, 1);
    expect_spi(0xFF, crc, false, 1);
}

// CMD12 stays in the selected read stream (SanDisk v2.2, section 5.23.3, p.5-18).
// Discard the stuff byte, following Elm-Chan read guidance: "The received byte
// immediately following CMD12 is a stuff byte" (See: https://elm-chan.org/docs/mmc/mmc_e.html#dataxfer)
static void expect_stop(uint8_t response, size_t busy_bytes, size_t response_delay, bool ready) {
    expect_command(12, 0);
    if (response_delay) {
        expect_spi(0xFF, 0xFF, false, response_delay);
    }
    expect_spi(0xFF, response, false, 1);
    // Idle, CRC, and illegal-command responses return before the R1b wait.
    if (!(response & (0x01 | 0x04 | 0x08))) {
        if (busy_bytes) {
            expect_spi(0xFF, 0, false, busy_bytes);
        }
        if (ready) {
            expect_spi(0xFF, 0xFF, false, 1);
        }
    }
    expect_release();
}

static void finish(int actual, int expected) {
    CHECK(actual == expected);
    CHECK(event_index == event_count);
    CHECK(cs_high);
    CHECK(!config._mutex.locked);
}

static void finish_lifecycle(int (*operation)(blockdevice_t *), int expected) {
    unsigned entries = mutex_entries;
    unsigned exits = mutex_exits;
    observing_lifecycle = true;
    initializing = operation == init;
    entry_config_initialized = config.is_initialized;
    entry_device_initialized = device.is_initialized;
    initialized_on_unlock = initializing && expected == BD_ERROR_OK;
    finish(operation(&device), expected);
    CHECK(mutex_entries == entries + 1);
    CHECK(mutex_exits == exits + 1);
    CHECK(config.is_initialized == initialized_on_unlock);
    CHECK(device.is_initialized == initialized_on_unlock);
    observing_lifecycle = false;
}

static void expect_init_command(int cmd, uint32_t arg, uint8_t response, uint32_t extra) {
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    if (cmd == ACMD41_SD_SEND_OP_COND) {
        expect_command(CMD55_APP_CMD, 0);
        expect_spi(0xFF, 1, false, 1);
        expect_spi(0xFF, 0xFF, false, 1);
    }
    expect_command(cmd, arg);
    expect_spi(0xFF, response, false, 1);
    if ((cmd == CMD8_SEND_IF_COND || cmd == CMD58_READ_OCR) &&
        !(response & (R1_COM_CRC_ERROR | R1_ILLEGAL_COMMAND))) {
        for (int shift = 24; shift >= 0; shift -= 8) {
            expect_spi(0xFF, extra >> shift, false, 1);
        }
    }
    expect_release();
}

// CMD59 controls CRC mode and should enable it before ACMD41 when requested
// (SD v3.01, section 7.2.2, p.116, and Table 7-3, p.126).
// Aborting on rejection is host policy; R1 reports command CRC errors
// (SD v3.01, section 7.3.2.1, p.129).
static void test_init_crc(bool enabled, bool rejected) {
    reset_card();
    config.enable_crc = enabled;
    expect_spi(0xFF, 0xFF, true, 10);
    expect_init_command(CMD0_GO_IDLE_STATE, 0, 1, 0);
    expect_init_command(CMD8_SEND_IF_COND, 0x1AA, 1, 0x1AA);
    if (enabled) {
        expect_init_command(CMD59_CRC_ON_OFF, 1, rejected ? 0x08 : 1, 0);
    }
    if (!rejected) {
        expect_init_command(CMD58_READ_OCR, 0, 1, OCR_3_3V);
        expect_init_command(ACMD41_SD_SEND_OP_COND, OCR_HCS_CCS, 0, 0);
        expect_init_command(CMD58_READ_OCR, 0, 0, OCR_3_3V | OCR_HCS_CCS);
        if (!enabled) {
            expect_init_command(CMD59_CRC_ON_OFF, 0, 0, 0);
        }
    }
    finish(init_card(&config), rejected ? SD_BLOCK_DEVICE_ERROR_CRC : BD_ERROR_OK);
}


static void reset_init_card(bool crc) {
    reset_card();
    config.card_type = SDCARD_NONE;
    config.enable_crc = crc;
    config.is_initialized = false;
    config.hz = 25000000;
    device.is_initialized = false;
}

static void expect_init_prefix(uint8_t card_type) {
    bool v2 = card_type != SDCARD_V1;
    expect_spi(0xFF, 0xFF, true, 10);
    expect_init_command(CMD0_GO_IDLE_STATE, 0, 1, 0);
    expect_init_command(CMD8_SEND_IF_COND, 0x1AA, v2 ? 1 : 0x05, 0x1AA);
    if (config.enable_crc) {
        expect_init_command(CMD59_CRC_ON_OFF, 1, 1, 0);
    }
    expect_init_command(CMD58_READ_OCR, 0, 1, OCR_3_3V);
}

static void expect_init_complete(uint8_t card_type) {
    if (card_type != SDCARD_V1) {
        expect_init_command(CMD58_READ_OCR, 0, 0,
                            OCR_3_3V | (card_type == SDCARD_V2HC ? OCR_HCS_CCS : 0));
    }
    if (!config.enable_crc) {
        expect_init_command(CMD59_CRC_ON_OFF, 0, 0, 0);
    }
}

static void finish_init_failure(int expected) {
    finish(init(&device), expected);
    CHECK(config.card_type == CARD_UNKNOWN);
    CHECK(!config.is_initialized);
    CHECK(!device.is_initialized);
}

// Legacy/V2 detection, HCS, ACMD41 polling, and post-init CCS determine these
// sequences and card types (SD v9.10, section 7.2.1, Figure 7-2, pp.287-289).
static void test_init_types(void) {
    for (uint8_t type = SDCARD_V1; type <= SDCARD_V2HC; ++type) {
        for (unsigned crc = 0; crc < 2; ++crc) {
            reset_init_card(crc);
            expect_init_prefix(type);
            uint32_t arg = type == SDCARD_V1 ? 0 : OCR_HCS_CCS;
            expect_init_command(ACMD41_SD_SEND_OP_COND, arg, 1, 0);
            expect_init_command(ACMD41_SD_SEND_OP_COND, arg, 0, 0);
            expect_init_complete(type);

            expect_select();
            expect_spi(0xFF, 0xFF, false, 1);
            expect_command(CMD9_SEND_CSD, 0);
            expect_spi(0xFF, 0, false, 1);
            // CSD v1 describes 1 GiB; CSD v2 describes 8 GiB.
            uint8_t csd[16] = {0};
            csd[5] = 9;
            csd[10] = 0x40;
            if (type == SDCARD_V2HC) {
                csd[0] = 0x40;
                csd[8] = 0x3F;
                csd[9] = 0xFF;
            } else {
                csd[6] = 3;
                csd[7] = 0xFF;
                csd[8] = 0xC0;
                csd[9] = 3;
                csd[10] |= 0x80;
            }
            uint16_t checksum = reference_crc16(csd, sizeof(csd));
            expect_spi(0xFF, 0xFE, false, 1);
            for (size_t i = 0; i < sizeof(csd); ++i) {
                expect_spi(0xFF, csd[i], false, 1);
            }
            expect_spi(0xFF, checksum >> 8, false, 1);
            expect_spi(0xFF, checksum, false, 1);
            expect_release();
            expect_init_command(CMD16_SET_BLOCKLEN, 512, 0, 0);
            finish(init(&device), BD_ERROR_OK);
            CHECK(config.is_initialized);
            CHECK(device.is_initialized);
            CHECK(config.card_type == type);
            CHECK(config.total_sectors == (type == SDCARD_V2HC ? 16777216 : 2097152));

            expect_start(1, type == SDCARD_V2HC ? 2 : 1024, 0);
            expect_block(0x31);
            expect_release();
            uint8_t buffer[512];
            finish(device.read(&device, buffer, 1024, sizeof(buffer)), BD_ERROR_OK);
            for (size_t i = 0; i < sizeof(buffer); ++i) {
                CHECK(buffer[i] == 0x31);
            }
        }
    }
}

static void test_init_crc_recovery(bool enabled, uint8_t response) {
    reset_init_card(enabled);
    time_step_us = 100;
    expect_init_prefix(SDCARD_V2HC);
    expect_init_command(ACMD41_SD_SEND_OP_COND, OCR_HCS_CCS, 1, 0);
    expect_init_command(ACMD41_SD_SEND_OP_COND, OCR_HCS_CCS, response, 0);
    expect_init_command(ACMD41_SD_SEND_OP_COND, OCR_HCS_CCS, 1, 0);
    expect_init_command(ACMD41_SD_SEND_OP_COND, OCR_HCS_CCS, 0, 0);
    expect_init_complete(SDCARD_V2HC);
    finish(init_card(&config), BD_ERROR_OK);
    CHECK(config.card_type == SDCARD_V2HC);
}

// Both idle polling and CRC recovery share one initialization deadline.
static void test_init_crc_timeout(bool enabled, uint8_t response) {
    reset_init_card(enabled);
    expect_init_prefix(SDCARD_V2HC);
    expect_init_command(ACMD41_SD_SEND_OP_COND, OCR_HCS_CCS, response, 0);
    expect_init_command(ACMD41_SD_SEND_OP_COND, OCR_HCS_CCS, response, 0);
    finish_init_failure(SD_BLOCK_DEVICE_ERROR_CRC);
}

// Initialization deadlines and preserving terminal failures are host policy.
static void test_init_failures(void) {
    static const struct {
        uint8_t acmd41;
        uint8_t final_ocr;
        int status;
    } cases[] = {
        {1, 0, SD_BLOCK_DEVICE_ERROR_NO_RESPONSE},
        {0x05, 0, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED},
        {0x11, 0, SD_BLOCK_DEVICE_ERROR_ERASE},
        {0x41, 0, SD_BLOCK_DEVICE_ERROR_PARAMETER},
        {0, 0x04, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED},
        {0, 0x08, SD_BLOCK_DEVICE_ERROR_CRC},
        {0, 0x40, SD_BLOCK_DEVICE_ERROR_PARAMETER},
    };
    for (unsigned crc = 0; crc < 2; ++crc) {
        for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
            reset_init_card(crc);
            expect_init_prefix(SDCARD_V2HC);
            expect_init_command(ACMD41_SD_SEND_OP_COND, OCR_HCS_CCS, 1, 0);
            expect_init_command(ACMD41_SD_SEND_OP_COND, OCR_HCS_CCS, cases[i].acmd41, 0);
            if (!cases[i].acmd41) {
                expect_init_command(CMD58_READ_OCR, 0, cases[i].final_ocr, 0);
            }
            finish_init_failure(cases[i].status);
        }
    }
}

static void test_init_transport_failure(bool final_ocr, bool busy) {
    for (unsigned crc = 0; crc < 2; ++crc) {
        reset_init_card(crc);
        expect_init_prefix(SDCARD_V2HC);
        expect_init_command(ACMD41_SD_SEND_OP_COND, OCR_HCS_CCS, 1, 0);
        if (final_ocr) {
            expect_init_command(ACMD41_SD_SEND_OP_COND, OCR_HCS_CCS, 0, 0);
        }
        expect_select();
        if (busy) {
            expect_spi(0xFF, 0, false, SD_COMMAND_TIMEOUT);
        } else {
            expect_spi(0xFF, 0xFF, false, 1);
            for (unsigned retry = 0; retry < 3; ++retry) {
                if (!final_ocr) {
                    expect_command(CMD55_APP_CMD, 0);
                    expect_spi(0xFF, 1, false, 1);
                    expect_spi(0xFF, 0xFF, false, 1);
                }
                expect_command(final_ocr ? CMD58_READ_OCR : ACMD41_SD_SEND_OP_COND,
                               final_ocr ? 0 : OCR_HCS_CCS);
                expect_spi(0xFF, 0xFF, false, 16);
            }
        }
        expect_release();
        finish_init_failure(busy ? SD_BLOCK_DEVICE_ERROR_NO_RESPONSE
                                 : SD_BLOCK_DEVICE_ERROR_NO_DEVICE);
    }
}

// Checks CMD17 at SDSC byte address 1024, command CRC7, start token, all 512 data
// bytes, CRC16, returned data, CS cleanup, and mutex release.
// Read blocks include the token and CRC bytes (SD v9.10, section 7.3.3.2, p.305).
static void test_singleblock(void) {
    reset_card();
    expect_start(1, 1024, 0);
    expect_block(0x31);
    expect_release();
    uint8_t buffer[512] = {0};
    uint8_t expected[512];
    memset(expected, 0x31, sizeof(expected));
    finish(device.read(&device, buffer, 1024, sizeof(buffer)), BD_ERROR_OK);
    CHECK(memcmp(buffer, expected, sizeof(buffer)) == 0);
}

static void test_singleblock_program(void) {
    static const uint8_t responses[] = {
        SPI_DATA_ACCEPTED, SPI_DATA_CRC_ERROR, SPI_DATA_WRITE_ERROR,
    };
    for (size_t i = 0; i < sizeof(responses); ++i) {
        for (unsigned timeout = 0; timeout < 2; ++timeout) {
            reset_card();
            expect_select();
            expect_spi(0xFF, 0xFF, false, 1);
            expect_command(CMD24_WRITE_BLOCK, 0);
            expect_spi(0xFF, 0, false, 1);
            expect_program_block(0x31, SPI_START_BLOCK, responses[i], timeout);
            expect_release();
            uint8_t buffer[512];
            memset(buffer, 0x31, sizeof(buffer));
            finish(program(&device, buffer, 0, sizeof(buffer)),
                   timeout ? SD_BLOCK_DEVICE_ERROR_NO_RESPONSE :
                   responses[i] == SPI_DATA_ACCEPTED ? BD_ERROR_OK :
                                                      SD_BLOCK_DEVICE_ERROR_WRITE);
        }
    }
}

static void test_multiblock_program_success(bool stop_timeout) {
    reset_card();
    expect_multiblock_program();
    expect_program_block(0x31, SPI_START_BLK_MUL_WRITE, SPI_DATA_ACCEPTED, false);
    expect_program_block(0xA6, SPI_START_BLK_MUL_WRITE, SPI_DATA_ACCEPTED, false);
    expect_spi(SPI_STOP_TRAN, 0xFF, false, 1);
    // Stop Tran may be followed by busy (SD v9.10, Figure 7-7, p.291).
    // Discard N_BR even when it looks ready; busy can start in the next byte
    // (SanDisk v2.2, sections 5.23.4-5.24, p.5-19).
    expect_spi(0xFF, 0xFF, false, 1);
    expect_spi(0xFF, 0, false, stop_timeout ? CONF_SD_CMD_TIMEOUT : 2);
    if (!stop_timeout) {
        expect_spi(0xFF, 0xFF, false, 1);
    }
    expect_release();
    uint8_t buffer[1024];
    memset(buffer, 0x31, 512);
    memset(buffer + 512, 0xA6, 512);
    finish(program(&device, buffer, 0, sizeof(buffer)),
           stop_timeout ? SD_BLOCK_DEVICE_ERROR_NO_RESPONSE : BD_ERROR_OK);
}

// N_BR spans 0..1 byte before busy (SanDisk v2.2, sections 5.23.4-5.24,
// p.5-19); final busy is optional (SD v9.10, Figure 7-7, p.291).
static void test_multiblock_stop_timing(void) {
    static const struct {
        uint8_t nbr;
        uint8_t busy;
        size_t busy_bytes;
    } cases[] = {
        {0xFF, 0x00, 0}, // No busy.
        {0xFF, 0x00, 1}, // Immediate busy.
        {0xFE, 0x7F, 1}, // Busy starts during the last bit of N_BR.
        {0xFF, 0x7F, 1}, // One short busy bit.
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_card();
        expect_multiblock_program();
        expect_program_block(0x31, SPI_START_BLK_MUL_WRITE, SPI_DATA_ACCEPTED, false);
        expect_program_block(0x31, SPI_START_BLK_MUL_WRITE, SPI_DATA_ACCEPTED, false);
        expect_spi(SPI_STOP_TRAN, 0xFF, false, 1);
        expect_spi(0xFF, cases[i].nbr, false, 1);
        if (cases[i].busy_bytes) {
            expect_spi(0xFF, cases[i].busy, false, cases[i].busy_bytes);
        }
        expect_spi(0xFF, 0xFF, false, 1);
        expect_release();
        uint8_t buffer[1024];
        memset(buffer, 0x31, sizeof(buffer));
        finish(program(&device, buffer, 0, sizeof(buffer)), BD_ERROR_OK);
    }
}

static void test_multiblock_program_failure(void) {
    static const struct {
        uint8_t response;
        bool write_timeout;
        uint8_t abort_response;
        bool abort_timeout;
        bool second_block;
    } cases[] = {
        {SPI_DATA_CRC_ERROR, false, 0, false, false},
        {SPI_DATA_WRITE_ERROR, false, R1_ILLEGAL_COMMAND, false, false},
        {SPI_DATA_ACCEPTED, true, 0, false, false},
        {SPI_DATA_CRC_ERROR, true, 0, false, false},
        {SPI_DATA_WRITE_ERROR, true, 0, false, true},
        {SPI_DATA_CRC_ERROR, false, 0, true, false},
        {SPI_DATA_WRITE_ERROR, false, 0, false, true},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_card();
        expect_multiblock_program();
        if (cases[i].second_block) {
            expect_program_block(0x31, SPI_START_BLK_MUL_WRITE, SPI_DATA_ACCEPTED, false);
        }
        expect_program_block(cases[i].second_block ? 0xA6 : 0x31,
                             SPI_START_BLK_MUL_WRITE, cases[i].response,
                             cases[i].write_timeout);
        if (cases[i].write_timeout) {
            expect_release();
        } else {
            expect_stop(cases[i].abort_response,
                        cases[i].abort_timeout ? CONF_SD_CMD_TIMEOUT : 0, 0,
                        !cases[i].abort_timeout);
        }
        uint8_t buffer[1024];
        memset(buffer, 0x31, 512);
        memset(buffer + 512, 0xA6, 512);
        finish(program(&device, buffer, 0, sizeof(buffer)),
               cases[i].write_timeout ? SD_BLOCK_DEVICE_ERROR_NO_RESPONSE :
                                        SD_BLOCK_DEVICE_ERROR_WRITE);
    }
}

// Checks CMD18 rejection with R1 illegal-command status: returns unsupported
// without reading data or sending CMD12, and completes CS/mutex cleanup.
//
// R1 status reference: SD Physical Layer Simplified Specification v9.10
// (December 1, 2023), section 7.3.2.1, p.302.
//
// Error mapping and CS/mutex cleanup are driver behavior, not SD requirements.
static void test_start_rejected(void) {
    reset_card();
    expect_start(2, 0, 0x04);
    expect_release();
    uint8_t buffer[1024];
    finish(device.read(&device, buffer, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_UNSUPPORTED);
}

// R1's bit 7 must be clear (SD v9.10, section 7.3.2.1, p.302).
// Exhausting polls with 0xFF is no response, not command success.
// The existing 16-byte guard and three retries are host tolerance, not N_CR.
// N_CR is at most eight byte-times (SanDisk v2.2, Table 5-5, p.5-19).
static void test_no_response(bool stop) {
    reset_card();
    if (stop) {
        expect_start(2, 0, 0);
        expect_block(0x31);
        expect_block(0xA6);
    } else {
        expect_select();
        expect_spi(0xFF, 0xFF, false, 1);
    }
    for (unsigned retry = 0; retry < 3; ++retry) {
        expect_command(stop ? 12 : 18, 0);
        expect_spi(0xFF, 0xFF, false, 16);
    }
    expect_release();
    uint8_t buffer[1024];
    finish(device.read(&device, buffer, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_DEVICE);
}

// Check valid R1 success/error responses after discarding the stuff byte,
// following Elm-Chan read guidance: "The received byte immediately following
// CMD12 is a stuff byte" (See: https://elm-chan.org/docs/mmc/mmc_e.html#dataxfer)
// A final-user-block CMD18 warning concerns OUT_OF_RANGE, distinct from R1's
// PARAMETER_ERROR (SD v3.01, section 4.3.3, p.29; section 7.3.2.1, p.129;
// Table 7-6, pp.133-134). It does not justify masking CMD12's 0x40 status
// without evidence of a card-specific quirk. These tests do not rule one out.
static void test_stop_status(uint8_t response, int expected, size_t response_delay) {
    reset_card();
    expect_start(2, 0, 0);
    expect_block(0x31);
    expect_block(0xA6);
    expect_stop(response, 0, response_delay, true);
    uint8_t buffer[1024];
    finish(device.read(&device, buffer, 0, sizeof(buffer)), expected);
}

// Bytes captured from a Samsung 2 GB card (MID 0x1B, 2011) after CMD12: stuff
// byte 3F (two data bits), then E0 1F, an R1 of 0x00 three bits late.
// E0 9F is the same alignment carrying ILLEGAL_COMMAND, which must still fail.
static void test_stop_unaligned(uint8_t second, int expected) {
    reset_card();
    expect_start(2, 0, 0);
    expect_block(0x31);
    expect_block(0xA6);
    expect_command_with_stuff(12, 0, 0x3F);
    expect_spi(0xFF, 0xE0, false, 1);
    expect_spi(0xFF, second, false, 1);
    if (BD_ERROR_OK == expected) {
        expect_spi(0xFF, 0xFF, false, 1);
    }
    expect_release();
    uint8_t buffer[1024];
    finish(device.read(&device, buffer, 0, sizeof(buffer)), expected);
}

// R1-only CRC/illegal-command errors: SD v9.10, section 7.3.2, p.302.
// Normal R2/R3/R7 layouts: sections 7.3.2.3-7.3.2.6, pp.303-304.
// Offset/delay tolerance is driver policy, not permission to violate alignment.
static void test_command_alignment(unsigned shift) {
    static const struct {
        int cmd;
        bool app;
        uint8_t bytes[5];
        size_t length;
        uint32_t response;
        int status;
    } cases[] = {
        {CMD16_SET_BLOCKLEN, false, {0x00}, 1, 0x00, BD_ERROR_OK},
        {CMD16_SET_BLOCKLEN, false, {0x01}, 1, 0x01, SD_BLOCK_DEVICE_ERROR_NO_INIT},
        {CMD16_SET_BLOCKLEN, false, {0x02}, 1, 0x02, SD_BLOCK_DEVICE_ERROR_ERASE},
        {CMD16_SET_BLOCKLEN, false, {0x04}, 1, 0x04, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED},
        {CMD16_SET_BLOCKLEN, false, {0x08}, 1, 0x08, SD_BLOCK_DEVICE_ERROR_CRC},
        {CMD16_SET_BLOCKLEN, false, {0x10}, 1, 0x10, SD_BLOCK_DEVICE_ERROR_ERASE},
        {CMD16_SET_BLOCKLEN, false, {0x20}, 1, 0x20, SD_BLOCK_DEVICE_ERROR_PARAMETER},
        {CMD16_SET_BLOCKLEN, false, {0x40}, 1, 0x40, SD_BLOCK_DEVICE_ERROR_PARAMETER},
        {CMD16_SET_BLOCKLEN, false, {0x41}, 1, 0x41, SD_BLOCK_DEVICE_ERROR_PARAMETER},
        {CMD16_SET_BLOCKLEN, false, {0x7F}, 1, 0x7F, SD_BLOCK_DEVICE_ERROR_CRC},
        {CMD8_SEND_IF_COND, false, {1, 0x81, 0xA5, 0x5A, 0xC3}, 5,
         0x81A55AC3, BD_ERROR_OK},
        {CMD8_SEND_IF_COND, false, {0x08}, 1, 0x08, SD_BLOCK_DEVICE_ERROR_CRC},
        {CMD8_SEND_IF_COND, false, {0x04}, 1, 0x04, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED},
        {CMD58_READ_OCR, false, {0, 0xC0, 0xFF, 0x80, 0}, 5,
         0xC0FF8000, BD_ERROR_OK},
        {CMD58_READ_OCR, false, {0x08}, 1, 0x08, SD_BLOCK_DEVICE_ERROR_CRC},
        {CMD58_READ_OCR, false, {0x04}, 1, 0x04, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED},
        {CMD13_SEND_STATUS, false, {0, 0x5A}, 2, 0x5A, BD_ERROR_OK},
        {CMD13_SEND_STATUS, false, {2, 0x5A}, 2, 0x025A, SD_BLOCK_DEVICE_ERROR_ERASE},
        {CMD13_SEND_STATUS, false, {0x08}, 1, 0x08, SD_BLOCK_DEVICE_ERROR_CRC},
        {ACMD13_SD_STATUS, true, {0, 0xA5}, 2, 0xA5, BD_ERROR_OK},
        {ACMD13_SD_STATUS, true, {0x04}, 1, 0x04, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED},
        {CMD38_ERASE, false, {0}, 1, 0, BD_ERROR_OK},
    };
    const char *outer_case = current_case;
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        for (unsigned delay = 0; delay < 2; ++delay) {
            char name[128];
            snprintf(name, sizeof(name), "%s/case-%zu/delay-%u", outer_case, i, delay);
            current_case = name;
            reset_card();
            expect_select();
            expect_spi(0xFF, 0xFF, false, 1);
            if (cases[i].app) {
                uint8_t r1 = 0;
                expect_command(CMD55_APP_CMD, 0);
                expect_response_bits(&r1, 8, (shift + 3) % 8, false);
                expect_spi(0xFF, 0xFF, false, 1);
            }
            expect_command(cases[i].cmd, 0);
            if (delay) {
                expect_spi(0xFF, 0xFF, false, 15);
            }
            expect_response_bits(cases[i].bytes, cases[i].length * 8, shift, false);
            if (cases[i].cmd == CMD38_ERASE) {
                expect_spi(0xFF, 0xFF, false, 1);
            }
            bool data = cases[i].app && cases[i].cmd == ACMD13_SD_STATUS &&
                        cases[i].status == BD_ERROR_OK;
            uint8_t expected[64];
            fill_pattern(expected, sizeof(expected));
            if (data) {
                expect_status_block(expected, sizeof(expected), false);
            }
            expect_release();
            uint32_t response = R1_NO_RESPONSE;
            int status = _cmd(&config, cases[i].cmd, 0, cases[i].app, &response);
            CHECK(response == cases[i].response);
            if (cases[i].cmd == CMD8_SEND_IF_COND) {
                CHECK(config.card_type ==
                      (cases[i].status == BD_ERROR_OK ? SDCARD_V2 :
                       cases[i].status == SD_BLOCK_DEVICE_ERROR_UNSUPPORTED ?
                       CARD_UNKNOWN : SDCARD_V1));
            }
            if (data) {
                CHECK(status == BD_ERROR_OK);
                CHECK(!cs_high);
                uint8_t buffer[64];
                status = _read_bytes(&config, buffer, sizeof(buffer), false);
                CHECK(memcmp(buffer, expected, sizeof(buffer)) == 0);
            }
            finish(status, cases[i].status);
        }
    }
    current_case = outer_case;
}

static void test_command_stuff(unsigned shift) {
    static const uint8_t stuff[] = {0x00, 0x04, 0x80, 0xFF};
    static const struct {
        uint8_t r1;
        int status;
    } cases[] = {
        {0x00, BD_ERROR_OK},
        {0x04, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED},
        {0x08, SD_BLOCK_DEVICE_ERROR_CRC},
        {0x20, SD_BLOCK_DEVICE_ERROR_PARAMETER},
        {0x40, SD_BLOCK_DEVICE_ERROR_PARAMETER},
    };
    for (size_t i = 0; i < sizeof(stuff); ++i) {
        for (size_t j = 0; j < sizeof(cases) / sizeof(cases[0]); ++j) {
            reset_card();
            expect_select();
            expect_spi(0xFF, 0xFF, false, 1);
            expect_command_with_stuff(CMD16_SET_BLOCKLEN, 512, stuff[i]);
            expect_response_bits(&cases[j].r1, 8, shift, false);
            expect_release();
            uint32_t response = R1_NO_RESPONSE;
            finish(_cmd(&config, CMD16_SET_BLOCKLEN, 512, false, &response), cases[j].status);
            CHECK(response == cases[j].r1);
        }
    }
}

static void test_command_retry(unsigned shift) {
    static const struct {
        int cmd;
        bool app;
        uint8_t bytes[5];
        size_t length;
        uint32_t response;
    } cases[] = {
        {CMD8_SEND_IF_COND, false, {1, 0x81, 0xA5, 0x5A, 0xC3}, 5, 0x81A55AC3},
        {CMD58_READ_OCR, false, {0, 0xC0, 0xFF, 0x80, 0}, 5, 0xC0FF8000},
        {ACMD41_SD_SEND_OP_COND, true, {0}, 1, 0},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_card();
        expect_select();
        expect_spi(0xFF, 0xFF, false, 1);
        for (unsigned attempt = 0; attempt < 2; ++attempt) {
            if (cases[i].app) {
                uint8_t r1 = 0;
                expect_command(CMD55_APP_CMD, 0);
                expect_response_bits(&r1, 8, attempt ? (shift + 3) % 8 : 7, false);
                expect_spi(0xFF, 0xFF, false, 1);
            }
            expect_command(cases[i].cmd, 0);
            if (!attempt) {
                expect_spi(0xFF, 0xFF, false, 16);
            } else {
                expect_response_bits(cases[i].bytes, cases[i].length * 8, shift, false);
            }
        }
        expect_release();
        uint32_t response = R1_NO_RESPONSE;
        finish(_cmd(&config, cases[i].cmd, 0, cases[i].app, &response), BD_ERROR_OK);
        CHECK(response == cases[i].response);
    }
}

// R1b busy is optional (SD v9.10, section 7.3.2.2, p.302); shifted R1
// tolerance and preserving specific errors over timeout are driver policy.
static void test_busy_alignment(unsigned shift) {
    for (unsigned error = 0; error < 2; ++error) {
        for (unsigned busy = 0; busy < 3; ++busy) {
            bool timeout = busy == 2;
            reset_card();
            expect_select();
            expect_spi(0xFF, 0xFF, false, 1);
            expect_command(CMD38_ERASE, 0);
            uint8_t r1 = error ? 0x40 : 0;
            expect_response_bits(&r1, 8, shift, busy != 0);
            if (busy) {
                expect_spi(0xFF, 0, false, timeout ? CONF_SD_CMD_TIMEOUT : 2);
            }
            if (!timeout) {
                expect_spi(0xFF, 0xFF, false, 1);
            }
            expect_release();
            int expected = error ? SD_BLOCK_DEVICE_ERROR_PARAMETER :
                           timeout ? SD_BLOCK_DEVICE_ERROR_NO_RESPONSE : BD_ERROR_OK;
            uint32_t response = R1_NO_RESPONSE;
            finish(_cmd(&config, CMD38_ERASE, 0, false, &response), expected);
            CHECK(response == r1);
        }
    }
}

static void test_read_command_alignment(unsigned shift) {
    for (unsigned blocks = 1; blocks <= 2; ++blocks) {
        reset_card();
        expect_select();
        expect_spi(0xFF, 0xFF, false, 1);
        expect_command(blocks == 1 ? CMD17_READ_SINGLE_BLOCK : CMD18_READ_MULTIPLE_BLOCK, 0);
        uint8_t r1 = 0;
        expect_response_bits(&r1, 8, shift, false);
        expect_block(0x31);
        if (blocks == 2) {
            expect_block(0xA6);
            expect_command(CMD12_STOP_TRANSMISSION, 0);
            expect_response_bits(&r1, 8, (shift + 3) % 8, false);
            expect_spi(0xFF, 0xFF, false, 1);
        }
        expect_release();
        uint8_t buffer[1024];
        size_t length = blocks * 512;
        finish(device.read(&device, buffer, 0, length), BD_ERROR_OK);
        for (size_t i = 0; i < length; ++i) {
            CHECK(buffer[i] == (i < 512 ? 0x31 : 0xA6));
        }
    }
}

// A valid CMD55 prefix is required before every application command. Fail
// closed when the prefix is unanswered or reports an R1 error.
static void test_app_prefix_failure(unsigned shift) {
    static const struct {
        uint8_t response;
        int status;
    } cases[] = {
        {R1_COM_CRC_ERROR, SD_BLOCK_DEVICE_ERROR_CRC},
        {R1_ILLEGAL_COMMAND, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED},
        {R1_ERASE_RESET, SD_BLOCK_DEVICE_ERROR_ERASE},
        {R1_PARAMETER_ERROR, SD_BLOCK_DEVICE_ERROR_PARAMETER},
        {R1_NO_RESPONSE, SD_BLOCK_DEVICE_ERROR_NO_DEVICE},
    };
    static const int commands[] = {
        ACMD13_SD_STATUS,
        ACMD23_SET_WR_BLK_ERASE_COUNT,
        ACMD41_SD_SEND_OP_COND,
    };
    const char *outer_case = current_case;
    for (size_t cmd = 0; cmd < sizeof(commands) / sizeof(commands[0]); ++cmd) {
        for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
            char name[96];
            snprintf(name, sizeof(name), "%s/cmd-%zu/case-%zu", outer_case, cmd, i);
            current_case = name;
            reset_card();
            expect_select();
            expect_spi(0xFF, 0xFF, false, 1);
            for (unsigned attempt = 0;
                 attempt < (cases[i].response == R1_NO_RESPONSE ? 3u : 1u); ++attempt) {
                expect_command(CMD55_APP_CMD, 0);
                if (cases[i].response == R1_NO_RESPONSE) {
                    expect_spi(0xFF, 0xFF, false, 16);
                } else {
                    expect_response_bits(&cases[i].response, 8, shift, false);
                }
            }
            expect_release();
            uint32_t response = 0x12345678;
            finish(_cmd(&config, commands[cmd], 0, true, &response), cases[i].status);
            CHECK(response == cases[i].response);
        }
    }
    current_case = outer_case;
}

static void expect_acmd13_preamble(void) {
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD55_APP_CMD, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(ACMD13_SD_STATUS, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(0xFF, 0xA5, false, 1);
}

static void test_acmd13_data(bool corrupt_crc) {
    reset_card();
    uint8_t expected[64];
    uint8_t actual[64] = {0};
    fill_pattern(expected, sizeof(expected));
    expect_acmd13_preamble();
    expect_status_block(expected, sizeof(expected), corrupt_crc);
    expect_release();

    uint32_t response = R1_NO_RESPONSE;
    int status = _cmd(&config, ACMD13_SD_STATUS, 0, true, &response);
    CHECK(status == BD_ERROR_OK);
    CHECK(response == 0xA5);
    CHECK(!cs_high);
    status = _read_bytes(&config, actual, sizeof(actual), false);
    if (corrupt_crc) {
        CHECK(status == SD_BLOCK_DEVICE_ERROR_CRC);
    } else {
        CHECK(status == BD_ERROR_OK);
        CHECK(memcmp(actual, expected, sizeof(actual)) == 0);
    }
    finish(status, corrupt_crc ? SD_BLOCK_DEVICE_ERROR_CRC : BD_ERROR_OK);
}

static void test_cmd13_release(void) {
    reset_card();
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD13_SEND_STATUS, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(0xFF, 0xA5, false, 1);
    expect_release();

    uint32_t response = R1_NO_RESPONSE;
    finish(_cmd(&config, CMD13_SEND_STATUS, 0, false, &response), BD_ERROR_OK);
    CHECK(response == 0xA5);
}

// CMD13 and ACMD13 share opcode 13 and return R2: R1 followed by a second
// status byte. Preserve nonzero R1 status while returning the complete value.
static void test_r2_status(void) {
    static const struct {
        int cmd;
        bool app;
        uint8_t r1;
        uint8_t second;
        bool reads_second;
        uint32_t response;
        int status;
    } cases[] = {
        {CMD13_SEND_STATUS, false, 0x00, 0x5A, true, 0x005A, BD_ERROR_OK},
        {CMD13_SEND_STATUS, false, 0x02, 0xA5, true, 0x02A5,
         SD_BLOCK_DEVICE_ERROR_ERASE},
        {CMD13_SEND_STATUS, false, R1_COM_CRC_ERROR, 0, false, R1_COM_CRC_ERROR,
         SD_BLOCK_DEVICE_ERROR_CRC},
        {ACMD13_SD_STATUS, true, 0x00, 0xA5, true, 0x00A5, BD_ERROR_OK},
        {ACMD13_SD_STATUS, true, R1_ILLEGAL_COMMAND, 0, false, R1_ILLEGAL_COMMAND,
         SD_BLOCK_DEVICE_ERROR_UNSUPPORTED},
    };
    const char *outer_case = current_case;
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        char name[64];
        snprintf(name, sizeof(name), "%s/case-%zu", outer_case, i);
        current_case = name;
        reset_card();
        bool read_status_block = cases[i].app && cases[i].status == BD_ERROR_OK;
        uint8_t expected[64];
        uint8_t actual[64] = {0};
        expect_select();
        expect_spi(0xFF, 0xFF, false, 1);
        if (cases[i].app) {
            expect_command(CMD55_APP_CMD, 0);
            expect_spi(0xFF, 0, false, 1);
            expect_spi(0xFF, 0xFF, false, 1);
        }
        expect_command(cases[i].cmd, 0);
        expect_spi(0xFF, cases[i].r1, false, 1);
        if (cases[i].reads_second) {
            expect_spi(0xFF, cases[i].second, false, 1);
        }
        if (read_status_block) {
            fill_pattern(expected, sizeof(expected));
            expect_status_block(expected, sizeof(expected), false);
        }
        expect_release();
        uint32_t response = R1_NO_RESPONSE;
        int status = _cmd(&config, cases[i].cmd, 0, cases[i].app, &response);
        CHECK(status == cases[i].status);
        CHECK(response == cases[i].response);
        if (read_status_block) {
            CHECK(!cs_high);
            status = _read_bytes(&config, actual, sizeof(actual), false);
            CHECK(status == BD_ERROR_OK);
            CHECK(memcmp(actual, expected, sizeof(actual)) == 0);
        }
        finish(status, cases[i].status);
    }
    current_case = outer_case;
}

// Reissue CMD55 after an unanswered prefix, then send the application opcode.
static void test_app_prefix_retry(unsigned shift) {
    reset_card();
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD55_APP_CMD, 0);
    expect_spi(0xFF, 0xFF, false, 16);
    expect_command(CMD55_APP_CMD, 0);
    uint8_t response = 0;
    expect_response_bits(&response, 8, shift, false);
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(ACMD41_SD_SEND_OP_COND, 0);
    expect_response_bits(&response, 8, (shift + 3) % 8, false);
    expect_release();

    uint32_t result = R1_NO_RESPONSE;
    finish(_cmd(&config, ACMD41_SD_SEND_OP_COND, 0, true, &result), BD_ERROR_OK);
    CHECK(result == 0);
}

// Exercise a full token wait across the 32-bit millisecond rollover, including
// the pre-rollover window where an overflowing deadline used to fail immediately.
static void test_token_wrap(bool timeout) {
    reset_card();
    now_us = ((uint64_t)UINT32_MAX - 8) * 1000;
    expect_start(1, 0, 0);
    if (timeout) {
        expect_spi(0xFF, 0xFF, false, 300);
    } else {
        expect_spi(0xFF, 0xFF, false, 8);
        expect_block(0x31);
    }
    expect_release();
    uint8_t buffer[512];
    finish(device.read(&device, buffer, 0, sizeof(buffer)),
           timeout ? SD_BLOCK_DEVICE_ERROR_NO_RESPONSE : BD_ERROR_OK);
}

// A timeout wider than the millisecond counter must still expire.
static void test_wide_busy_timeout(void) {
    reset_card();
    time_step_us = (uint64_t)UINT32_MAX * 1000;
    expect_spi(0xFF, 0, true, 2);
    finish(_wait_ready(&config, (uint64_t)UINT32_MAX + 10), false);
}

// Commands are rejected during busy (SD v9.10, section 7.2.8, p.292).
// Check that readiness timeout sends no ordinary/application command and
// leaves the response sentinel, with exactly one release and no extra bytes.
static void test_command_busy(bool app_command) {
    reset_card();
    expect_select();
    if (app_command) {
        expect_spi(0xFF, 0xFF, false, 1);
        expect_command(55, 0);
        expect_spi(0xFF, 0, false, 1);
    }
    expect_spi(0xFF, 0, false, CONF_SD_CMD_TIMEOUT);
    expect_release();
    uint32_t response = 0x12345678;
    int status = _cmd(&config, app_command ? 41 : 16, 0, app_command, &response);
    CHECK(response == R1_NO_RESPONSE);
    finish(status, SD_BLOCK_DEVICE_ERROR_NO_RESPONSE);

    if (!app_command) {
        // Also exercise the public read's propagation and mutex cleanup.
        reset_card();
        expect_select();
        expect_spi(0xFF, 0, false, CONF_SD_CMD_TIMEOUT);
        expect_release();
        uint8_t buffer[1024];
        finish(device.read(&device, buffer, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_RESPONSE);
    }
}

// Keep CS low from the final CRC through CMD12 (SanDisk v2.2, section 5.23.3,
// p.5-18). CMD12 gets the same leading FF as other commands, without a ready
// wait that could consume the selected read stream.
// SDSC byte addressing and SDHC block addressing differ (SD v9.10, Table 7-3,
// footnote 10, p.299). Busy is optional (Table 7-3, footnote 5, p.299).
static void test_multiblock(bool high_capacity, bool crc_enabled) {
    reset_card();
    config.card_type = high_capacity ? SDCARD_V2HC : SDCARD_V1;
    config.enable_crc = crc_enabled;
    expect_start(2, high_capacity ? 2 : 1024, 0);
    expect_block(0x31);
    expect_block(0xA6);
    expect_stop(0, 3, 2, true);
    uint8_t buffer[1024] = {0};
    uint8_t expected[1024];
    memset(expected, 0x31, 512);
    memset(expected + 512, 0xA6, 512);
    finish(device.read(&device, buffer, 1024, sizeof(buffer)), BD_ERROR_OK);
    CHECK(memcmp(buffer, expected, sizeof(buffer)) == 0);
}

// Read blocks carry CRC16 and CMD12 terminates CMD18 (SD v9.10, section 7.2.3,
// pp.289-290). Preferring the data error over cleanup failure is API policy.
// One block succeeds before failure. The unread third block must not be consumed.
// The 300 ms token and 5 s default production command budgets are host policy,
// not comprehensive timing compliance: SDSC timing uses TAAC/NSAC, capped at
// 100 ms; hosts should use at least 100 ms for SDHC/SDXC reads (SD v9.10,
// section 4.6.2.1, p.95).
// Non-FF/non-FE read tokens fail immediately (SD v9.10, section 7.3.3.3,
// p.306 defines the data-error tokens; rejecting other bytes is host policy).
static void test_multiblock_data_failure(bool crc_failure, uint8_t stop_response, bool stop_busy) {
    reset_card();
    expect_start(3, 0, 0);
    expect_block(0x31);
    if (crc_failure) {
        expect_block_crc(0xA6, true);
    } else {
        expect_spi(0xFF, 0xFF, false, 300);
    }
    expect_stop(stop_response, stop_busy ? CONF_SD_CMD_TIMEOUT : 0, 0, !stop_busy);
    uint8_t buffer[1536];
    // The original data error takes precedence over any stop/busy failure.
    finish(device.read(&device, buffer, 0, sizeof(buffer)),
           crc_failure ? SD_BLOCK_DEVICE_ERROR_CRC : SD_BLOCK_DEVICE_ERROR_NO_RESPONSE);
}

static void test_singleblock_crc(void) {
    reset_card();
    expect_start(1, 0, 0);
    expect_block_crc(0x31, true);
    expect_release();
    uint8_t buffer[512];
    finish(device.read(&device, buffer, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_CRC);
}

static void test_read_token_rejection(void) {
    for (unsigned crc = 0; crc < 2; ++crc) {
        for (unsigned token = 0; token < 256; ++token) {
            if (token == 0xFF || token == SPI_START_BLOCK) {
                continue;
            }
            reset_card();
            config.enable_crc = crc;
            expect_start(1, 0, 0);
            expect_spi(0xFF, token, false, 1);
            // A later FE belongs to the rejected stream, not a new block.
            expect_spi(0xFF, 0xFE, false, 8);
            expect_cs(true);
            expect_spi(0xFF, 0xFF, true, 1);
            uint8_t buffer[512];
            memset(buffer, 0xA5, sizeof(buffer));
            finish(device.read(&device, buffer, 0, sizeof(buffer)), BD_ERROR_DEVICE_ERROR);
            for (size_t i = 0; i < sizeof(buffer); ++i) {
                CHECK(buffer[i] == 0xA5);
            }
        }
    }
}

static void test_shifted_read_rejection(void) {
    for (unsigned crc = 0; crc < 2; ++crc) {
        for (unsigned shift = 1; shift < 8; ++shift) {
            reset_card();
            config.enable_crc = crc;
            uint8_t stream[517] = {0, 0xFF, SPI_START_BLOCK};
            memset(stream + 3, 0x55, 512);
            stream[3] = 0x01;
            stream[4] = 0xFC;
            uint16_t checksum = reference_crc16(stream + 3, 512);
            stream[515] = checksum >> 8;
            stream[516] = checksum;
            expect_select();
            expect_spi(0xFF, 0xFF, false, 1);
            expect_command(CMD17_READ_SINGLE_BLOCK, 0);
            // Four wire bytes reach the rejected token; the next eight are
            // cleanup clocks. Never scan onward to the payload-derived FE.
            expect_response_bits(stream, 12 * 8 - shift, shift, false);
            expect_cs(true);
            expect_spi(0xFF, 0xFF, true, 1);
            uint8_t buffer[512];
            memset(buffer, 0xA5, sizeof(buffer));
            finish(device.read(&device, buffer, 0, sizeof(buffer)), BD_ERROR_DEVICE_ERROR);
            for (size_t i = 0; i < sizeof(buffer); ++i) {
                CHECK(buffer[i] == 0xA5);
            }
        }
    }
}

static void test_register_token_rejection(bool app) {
    static const uint8_t tokens[] = {0x00, 0x04, 0x80};
    for (unsigned crc = 0; crc < 2; ++crc) {
        for (size_t i = 0; i < sizeof(tokens); ++i) {
            reset_card();
            config.enable_crc = crc;
            expect_select();
            expect_spi(0xFF, 0xFF, false, 1);
            if (app) {
                expect_command(CMD55_APP_CMD, 0);
                expect_spi(0xFF, 0, false, 1);
                expect_spi(0xFF, 0xFF, false, 1);
            }
            int cmd = app ? ACMD13_SD_STATUS : CMD9_SEND_CSD;
            expect_command(cmd, 0);
            expect_spi(0xFF, 0, false, app ? 2 : 1);
            if (!app && tokens[i] == 0) {
                expect_spi(0xFF, 0xFF, false, 1);
            }
            expect_spi(0xFF, tokens[i], false, 1);
            expect_release();
            CHECK(_cmd(&config, cmd, 0, app, NULL) == BD_ERROR_OK);
            CHECK(!cs_high);
            uint8_t buffer[64];
            memset(buffer, 0xA5, sizeof(buffer));
            finish(_read_bytes(&config, buffer, app ? 64 : 16, !app), BD_ERROR_DEVICE_ERROR);
            for (size_t j = 0; j < sizeof(buffer); ++j) {
                CHECK(buffer[j] == 0xA5);
            }
        }
    }
}

static void test_csd_initial_zero_rejection(void) {
    static const struct {
        uint8_t bytes[3];
        size_t length;
        bool timeout;
    } cases[] = {
        {{0x00, 0x00}, 2, false},
        {{0x00, 0xFF, 0x00}, 3, false},
        {{0x00, 0x04}, 2, false},
        {{0x00, 0x80}, 2, false},
        {{0x00}, 1, true},
    };
    for (unsigned crc = 0; crc < 2; ++crc) {
        for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
            reset_card();
            config.enable_crc = crc;
            expect_select();
            expect_spi(0xFF, 0xFF, false, 1);
            expect_command(CMD9_SEND_CSD, 0);
            expect_spi(0xFF, 0, false, 1);
            for (size_t j = 0; j < cases[i].length; ++j) {
                expect_spi(0xFF, cases[i].bytes[j], false, 1);
            }
            if (cases[i].timeout) {
                // The initial zero consumes one millisecond of the same budget.
                expect_spi(0xFF, 0xFF, false, 299);
            }
            expect_spi(0xFF, 0xFE, false, 8);
            expect_cs(true);
            expect_spi(0xFF, 0xFF, true, 1);
            CHECK(_cmd(&config, CMD9_SEND_CSD, 0, false, NULL) == BD_ERROR_OK);
            uint8_t buffer[16];
            memset(buffer, 0xA5, sizeof(buffer));
            finish(_read_bytes(&config, buffer, sizeof(buffer), true),
                   cases[i].timeout ? SD_BLOCK_DEVICE_ERROR_NO_RESPONSE : BD_ERROR_DEVICE_ERROR);
            for (size_t j = 0; j < sizeof(buffer); ++j) {
                CHECK(buffer[j] == 0xA5);
            }
        }
    }
}

static void test_multiblock_token_rejection(void) {
    for (unsigned crc = 0; crc < 2; ++crc) {
        for (unsigned stop = 0; stop < 3; ++stop) {
            reset_card();
            config.enable_crc = crc;
            expect_start(3, 0, 0);
            expect_block(0x31);
            expect_spi(0xFF, stop ? 0x80 : 0x04, false, 1);
            expect_stop(stop == 1 ? R1_ILLEGAL_COMMAND : 0,
                        stop == 2 ? CONF_SD_CMD_TIMEOUT : 0, 0, stop != 2);
            uint8_t buffer[1536];
            memset(buffer, 0xA5, sizeof(buffer));
            finish(device.read(&device, buffer, 0, sizeof(buffer)), BD_ERROR_DEVICE_ERROR);
            CHECK(mutex_entries == 1 && mutex_exits == 1);
            for (size_t i = 0; i < sizeof(buffer); ++i) {
                CHECK(buffer[i] == (i < 512 ? 0x31 : 0xA5));
            }
            expect_start(1, 1536, 0);
            expect_block(0x42);
            expect_release();
            finish(device.read(&device, buffer, 1536, 512), BD_ERROR_OK);
            CHECK(mutex_entries == 2 && mutex_exits == 2);
            for (size_t i = 0; i < 512; ++i) {
                CHECK(buffer[i] == 0x42);
            }
        }
    }
}

static void test_repeated_read(void) {
    reset_card();
    expect_start(2, 0, 0);
    expect_block(0x31);
    expect_block(0xA6);
    expect_stop(0, 3, 2, true);
    expect_start(1, 1024, 0);
    expect_block(0x42);
    expect_release();
    uint8_t buffer[1024];
    CHECK(device.read(&device, buffer, 0, sizeof(buffer)) == BD_ERROR_OK);
    CHECK(cs_high);
    CHECK(!config._mutex.locked);
    finish(device.read(&device, buffer, 1024, 512), BD_ERROR_OK);
    uint8_t expected[512];
    memset(expected, 0x42, sizeof(expected));
    CHECK(memcmp(buffer, expected, sizeof(expected)) == 0);
}

// Idle indicates initialization (SD v3.01, section 7.3.2.1, p.129).
// Preserve initialization responses, reject operational idle, and distinguish
// actual ACMD41 from bare CMD41. Specific erase/parameter errors remain specific.
// Initialization commands are described separately (SD v3.01, section 7.2.1,
// pp.114-115, and section 7.2.7, p.119). Idle is not operational completion.
static void test_idle_command(int cmd, bool is_acmd, uint8_t r1, int expected) {
    reset_card();
    config.is_initialized = expected != BD_ERROR_OK;
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    if (is_acmd) {
        expect_command(55, 0);
        expect_spi(0xFF, 0x01, false, 1);
        expect_spi(0xFF, 0xFF, false, 1);
    }
    expect_command(cmd, 0);
    expect_spi(0xFF, r1, false, 1);
    if (expected == BD_ERROR_OK && (cmd == 8 || cmd == 58)) {
        expect_spi(0xFF, 0, false, 4);
    }
    expect_release();
    uint32_t response = R1_NO_RESPONSE;
    finish(_cmd(&config, cmd, 0, is_acmd, &response), expected);
    CHECK(response == (expected == BD_ERROR_OK && (cmd == 8 || cmd == 58) ? 0 : r1));
}

// R1b permits optional busy bytes; zero means busy and nonzero means ready
// (SD v9.10, section 7.3.2.2, p.302).
// A timeout leaves completion unconfirmed, but must not replace a more
// specific R1 error. The full-high check and timeout budget remain host policy.
// Waiting selected is synchronous-driver policy: deselection is permitted during
// programming (SD v9.10, section 7.2.4, p.291) and R1b busy (SanDisk v2.2,
// section 5.23.2, p.5-17).
static void test_stop_busy(bool wrap, uint8_t response, int expected) {
    reset_card();
    expect_start(2, 0, 0);
    expect_block(0x31);
    expect_block(0xA6);
    expect_stop(response, CONF_SD_CMD_TIMEOUT, 2, false);
    if (wrap) {
        // Packet/data transfers do not advance the mock clock; the three time
        // reads before the busy wait leave it just before millisecond rollover.
        now_us = ((uint64_t)UINT32_MAX - 6) * 1000;
    }
    uint8_t buffer[1024];
    finish(device.read(&device, buffer, 0, sizeof(buffer)), expected);
}

static void test_erase_busy_timeout(void) {
    reset_card();
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(38, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(0xFF, 0, false, CONF_SD_CMD_TIMEOUT);
    expect_release();
    uint32_t response = R1_NO_RESPONSE;
    finish(_cmd(&config, 38, 0, false, &response), SD_BLOCK_DEVICE_ERROR_NO_RESPONSE);
    CHECK(response == 0);
}

// Without cached SD Status erase parameters, estimate write-block count times
// 250 ms (SD v3.01, section 4.6.2.3, p.57). This is not the AU-based formula
// (T_ERASE / N_ERASE) * X + T_OFFSET, with a one-second minimum and partial-AU
// allowances (SD v3.01, section 4.14.4, p.89).
// Forty logical sectors give a 10 s estimate: a 6 s erase must succeed despite
// exceeding the old 5 s production budget. Check both address formats, budget
// expiration, and rollover. Only trim's CMD38 gets the scaled busy budget;
// CMD12 and pre-command waits retain the command timeout.
// Power-up needs at least 74 clocks with CS/DI high (SD v3.01, section 6.4.1, p.111).
// The 1 ms settling wait follows Elm-Chan SPI initialization guidance
// (See: https://elm-chan.org/docs/mmc/mmc_e.html#spiinit).
static void test_power_up(void) {
    reset_card();
    cs_high = false;
    time_step_us = 0;
    startup_pending = true;
    expect_cs(true);
    expect_spi(0xFF, 0xFF, true, 10);
    _spi_init(&config);
    CHECK(!startup_pending);
    finish(BD_ERROR_OK, BD_ERROR_OK);
}

static void test_trim_busy(bool high_capacity, bool timeout, bool wrap) {
    reset_card();
    config.card_type = high_capacity ? SDCARD_V2HC : SDCARD_V1;
    if (wrap) {
        now_us = ((uint64_t)UINT32_MAX - 3) * 1000;
    }
    for (int cmd = 32; cmd <= 33; ++cmd) {
        expect_select();
        expect_spi(0xFF, 0xFF, false, 1);
        uint32_t sector = cmd == 32 ? 2 : 41;
        expect_command(cmd, high_capacity ? sector : sector * 512);
        expect_spi(0xFF, 0, false, 1);
        expect_release();
    }
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(38, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(0xFF, 0, false, timeout ? 10000 : 6000);
    if (!timeout) {
        expect_spi(0xFF, 0xFF, false, 1);
    }
    expect_release();
    finish(trim(&device, 2 * 512, 40 * 512),
           timeout ? SD_BLOCK_DEVICE_ERROR_NO_RESPONSE : BD_ERROR_OK);
}


static void make_ordinary_csd(uint8_t *csd, bool high_capacity) {
    memset(csd, 0, 16);
    set_csd_bits(csd, 127, 126, high_capacity ? 1 : 0);
    set_csd_bits(csd, 83, 80, 9);
    set_csd_bits(csd, 25, 22, 9);
    if (high_capacity) {
        set_csd_bits(csd, 69, 48, 8191);
        set_csd_bits(csd, 46, 46, 1);
        set_csd_bits(csd, 45, 39, 127);
    } else {
        set_csd_bits(csd, 73, 62, 1023);
        set_csd_bits(csd, 49, 47, 0);
        set_csd_bits(csd, 45, 39, 3);
    }
}

static void make_csd(uint8_t *csd) {
    make_ordinary_csd(csd, false);
}

static void expect_csd(const uint8_t *csd, bool corrupt_crc) {
    uint16_t crc = reference_crc16(csd, 16) ^ (corrupt_crc ? 1 : 0);
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD9_SEND_CSD, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(0xFF, 0xFE, false, 1);
    for (size_t i = 0; i < 16; ++i) {
        expect_spi(0xFF, csd[i], false, 1);
    }
    expect_spi(0xFF, crc >> 8, false, 1);
    expect_spi(0xFF, crc, false, 1);
    expect_release();
}

static void expect_card_init(bool fail, bool high_capacity) {
    expect_spi(0xFF, 0xFF, true, 10);
    if (fail) {
        for (unsigned i = 0; i < SD_CMD0_GO_IDLE_STATE_RETRIES; ++i) {
            expect_init_command(CMD0_GO_IDLE_STATE, 0, 0, 0);
        }
        return;
    }
    expect_init_command(CMD0_GO_IDLE_STATE, 0, 1, 0);
    expect_init_command(CMD8_SEND_IF_COND, 0x1AA,
                        high_capacity ? R1_IDLE_STATE : R1_IDLE_STATE | R1_ILLEGAL_COMMAND, 0x1AA);
    if (config.enable_crc) {
        expect_init_command(CMD59_CRC_ON_OFF, 1, 1, 0);
    }
    expect_init_command(CMD58_READ_OCR, 0, 1, OCR_3_3V);
    expect_init_command(ACMD41_SD_SEND_OP_COND, high_capacity ? OCR_HCS_CCS : 0, 0, 0);
    if (high_capacity) {
        expect_init_command(CMD58_READ_OCR, 0, 0, OCR_3_3V | OCR_HCS_CCS);
    }
    if (!config.enable_crc) {
        expect_init_command(CMD59_CRC_ON_OFF, 0, 0, 0);
    }
}

static void expect_top_init_setup(unsigned failure, bool high_capacity) {
    expect_card_init(failure == 4, high_capacity);
    if (failure == 4) {
        return;
    }
    uint8_t csd[16];
    make_csd(csd);
    if (high_capacity) {
        set_csd_bits(csd, 127, 126, 1);
        set_csd_bits(csd, 69, 48, 3);
    }
    expect_csd(csd, failure == 1);
    if (failure != 1) {
        expect_init_command(CMD16_SET_BLOCKLEN, 512, failure == 2 ? R1_ILLEGAL_COMMAND : 0, 0);
    }
}

static void exercise_initialized_io(size_t trim_size) {
    bool high_capacity = config.card_type == SDCARD_V2HC;
    uint8_t buffer[512] = {0};
    observing_ready_io = true;
    CHECK(config.erase_size == trim_size);
    CHECK(device.read_size == 512 && device.program_size == 512 && device.erase_size == 512);
    finish(erase(&device, 2048, sizeof(buffer)), BD_ERROR_OK);
    expect_start(1, high_capacity ? 4 : 2048, 0);
    expect_block(0x31);
    expect_release();
    finish(device.read(&device, buffer, 2048, sizeof(buffer)), BD_ERROR_OK);
    for (size_t i = 0; i < sizeof(buffer); ++i) {
        CHECK(buffer[i] == 0x31);
    }

    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD24_WRITE_BLOCK, high_capacity ? 4 : 2048);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(SPI_START_BLOCK, 0xFF, false, 1);
    expect_spi(0x31, 0xFF, false, sizeof(buffer));
    uint16_t crc = config.enable_crc ? reference_crc16(buffer, sizeof(buffer)) : 0xFFFF;
    expect_spi(crc >> 8, 0xFF, false, 1);
    expect_spi(crc, 0xFF, false, 1);
    expect_spi(0xFF, SPI_DATA_ACCEPTED, false, 1);
    expect_spi(0xFF, 0, false, 2);
    expect_spi(0xFF, 0xFF, false, 1);
    expect_release();
    finish(device.program(&device, buffer, 2048, sizeof(buffer)), BD_ERROR_OK);

    bd_size_t trim_address = 2 * trim_size;
    for (int cmd = CMD32_ERASE_WR_BLK_START_ADDR; cmd <= CMD33_ERASE_WR_BLK_END_ADDR; ++cmd) {
        expect_select();
        expect_spi(0xFF, 0xFF, false, 1);
        uint32_t address = cmd == CMD32_ERASE_WR_BLK_START_ADDR ? trim_address :
                           trim_address + trim_size - 512;
        expect_command(cmd, high_capacity ? address / 512 : address);
        expect_spi(0xFF, 0, false, 1);
        expect_release();
    }
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD38_ERASE, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(0xFF, 0xFF, false, 1);
    expect_release();
    finish(device.trim(&device, trim_address, trim_size), BD_ERROR_OK);
    observing_ready_io = false;
    CHECK(config.is_initialized && device.is_initialized);
}

static void test_initialized_invalid_io(void) {
    reset_card();
    uint8_t buffer[512] = {0};
    const bd_size_t addresses[] = {1, 0, device.size(&device)};
    const bd_size_t sizes[] = {512, 1, 512};
    for (size_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); ++i) {
        finish(device.read(&device, buffer, addresses[i], sizes[i]),
               SD_BLOCK_DEVICE_ERROR_PARAMETER);
        finish(device.program(&device, buffer, addresses[i], sizes[i]),
               SD_BLOCK_DEVICE_ERROR_PARAMETER);
        finish(device.trim(&device, addresses[i], sizes[i]),
               i == 2 ? SD_BLOCK_DEVICE_ERROR_PARAMETER : BD_ERROR_OK);
        CHECK(config.is_initialized && device.is_initialized);
    }
}

static void test_uninitialized_io(bool after_deinit) {
    reset_card();
    if (after_deinit) {
        finish_lifecycle(deinit, BD_ERROR_OK);
    } else {
        config.is_initialized = device.is_initialized = false;
    }
    config.erase_size = 0;
    CHECK(!config.is_initialized && !device.is_initialized);
    uint8_t buffer[512] = {0};
    finish(read(&device, buffer, 1, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_INIT);
    finish(program(&device, buffer, 1, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_INIT);
    finish(trim(&device, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_INIT);
    finish(trim(&device, 0, 0), SD_BLOCK_DEVICE_ERROR_NO_INIT);
}

static void test_top_init(unsigned failure) {
    reset_card();
    expect_top_init_setup(failure, false);
    if (failure == 3) {
        config.hz = 30000000;
    }
    int status = failure == 4 ? SD_BLOCK_DEVICE_ERROR_NO_DEVICE :
                 failure == 1 || failure == 2 ? BD_ERROR_DEVICE_ERROR :
                 failure == 3 ? -EINVAL : BD_ERROR_OK;
    finish_lifecycle(init, status);
    CHECK(device.is_initialized == (failure == 0));
    CHECK(config.is_initialized == (failure == 0));
    if (!failure) {
        CHECK(transfer_baudrate == 12000000);
        CHECK(device.size(&device) == 4096 * 512);
    } else {
        uint8_t buffer[512] = {0};
        finish(read(&device, buffer, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_INIT);
        finish(program(&device, buffer, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_INIT);
        finish(trim(&device, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_INIT);
        expect_top_init_setup(0, false);
        finish_lifecycle(init, BD_ERROR_OK);
    }
    exercise_initialized_io(2048);
}

static void test_cold_init(bool high_capacity, bool crc_enabled) {
    reset_card();
    config.is_initialized = device.is_initialized = false;
    config.card_type = SDCARD_NONE;
    config.total_sectors = config.erase_size = 0;
    config.enable_crc = crc_enabled;
    expect_top_init_setup(0, high_capacity);
    finish_lifecycle(init, BD_ERROR_OK);
    CHECK(config.card_type == (high_capacity ? SDCARD_V2HC : SDCARD_V1));
    CHECK(device.size(&device) == 4096 * 512);
    exercise_initialized_io(high_capacity ? 512 : 2048);
}

static void test_init_csd_initial_zero(void) {
    for (unsigned high_capacity = 0; high_capacity < 2; ++high_capacity) {
        for (unsigned crc = 0; crc < 2; ++crc) {
            for (unsigned padding = 0; padding < 2; ++padding) {
                for (unsigned corrupt_crc = 0; corrupt_crc < 2; ++corrupt_crc) {
                    reset_init_card(crc);
                    uint8_t csd[16];
                    make_ordinary_csd(csd, high_capacity);
                    csd[15] = reference_crc7(csd, 15);
                    expect_card_init(false, high_capacity);
                    expect_select();
                    expect_spi(0xFF, 0xFF, false, 1);
                    expect_command(CMD9_SEND_CSD, 0);
                    expect_spi(0xFF, 0, false, 1);
                    expect_spi(0xFF, 0, false, 1);
                    if (padding) {
                        expect_spi(0xFF, 0xFF, false, 3);
                    }
                    expect_status_block(csd, sizeof(csd), corrupt_crc);
                    expect_release();
                    bool succeeds = !crc || !corrupt_crc;
                    if (succeeds) {
                        expect_init_command(CMD16_SET_BLOCKLEN, 512, 0, 0);
                    }
                    finish_lifecycle(init, succeeds ? BD_ERROR_OK : BD_ERROR_DEVICE_ERROR);
                    CHECK(config.total_sectors ==
                          (succeeds ? (high_capacity ? 8388608u : 4096u) : 0));
                    CHECK(config.card_type == (high_capacity ? SDCARD_V2HC : SDCARD_V1));
                }
            }
        }
    }
}

static void test_deinit_reinit(void) {
    reset_card();
    finish_lifecycle(deinit, BD_ERROR_OK);
    finish_lifecycle(deinit, BD_ERROR_OK);
    CHECK(!config.is_initialized && !device.is_initialized);

    expect_top_init_setup(0, false);
    finish_lifecycle(init, BD_ERROR_OK);
    CHECK(config.is_initialized && device.is_initialized);
    CHECK(transfer_baudrate == 12000000);
    exercise_initialized_io(2048);
}


// Exercise the actual SPI setup, not just the configured frequency constant.
// The compatibility range follows Elm-Chan SPI initialization guidance
// (See: https://elm-chan.org/docs/mmc/mmc_e.html#spiinit).
static void test_init_clock(void) {
    reset_card();
    initialized_baudrate = 0;
    expect_spi(0xFF, 0xFF, true, 10);
    _spi_init(&config);
    CHECK(initialized_baudrate >= 100000 && initialized_baudrate <= 400000);
    CHECK(format_calls == 1);
    finish(BD_ERROR_OK, BD_ERROR_OK);
}

static void expect_trim_range(bd_size_t addr, bd_size_t length) {
    bool high_capacity = config.card_type == SDCARD_V2HC;
    bd_size_t end = addr + length - 512;
    expect_init_command(CMD32_ERASE_WR_BLK_START_ADDR, high_capacity ? addr / 512 : addr, 0, 0);
    expect_init_command(CMD33_ERASE_WR_BLK_END_ADDR, high_capacity ? end / 512 : end, 0, 0);
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD38_ERASE, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(0xFF, 0xFF, false, 1);
    expect_release();
}

static void test_advisory_trim(bool high_capacity) {
    static const struct {
        size_t unit;
        bd_size_t addr;
        bd_size_t length;
        bd_size_t trimmed_addr;
        bd_size_t trimmed_length;
    } cases[] = {
        {1024, 512, 3072, 1024, 2048},
        {1024, 512, 1536, 1024, 1024},
        {1024, 1024, 1536, 1024, 1024},
        {1536, 512, 5120, 1536, 3072},
        {1024, 1, 4094, 1024, 2048},
        {1024, 512, 512, 0, 0},
        {1024, 512, 256, 0, 0},
        {1024, 512, 1024, 0, 0},
        {1024, 1024, 512, 0, 0},
        {1024, 1024, 0, 0, 0},
        {1024, 4096 * 512, 0, 0, 0},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_card();
        config.card_type = high_capacity ? SDCARD_V2HC : SDCARD_V1;
        config.erase_size = cases[i].unit;
        if (cases[i].trimmed_length != 0) {
            expect_trim_range(cases[i].trimmed_addr, cases[i].trimmed_length);
        }
        finish(trim(&device, cases[i].addr, cases[i].length), BD_ERROR_OK);
    }
    if (high_capacity) {
        reset_card();
        config.card_type = SDCARD_V2HC;
        config.total_sectors = UINT64_C(1) << 32;
        expect_trim_range(UINT64_C(0x100000200), 512);
        finish(trim(&device, UINT64_C(0x100000001), 1534), BD_ERROR_OK);
    }
}

static void test_trim_invalid_bounds(void) {
    static const struct {
        bd_size_t addr;
        bd_size_t length;
    } cases[] = {
        {4096 * 512, 512},
        {4096 * 512 + 1, 0},
        {4096 * 512 - 512, 1024},
        {UINT64_MAX - 511, 512},
        {512, UINT64_MAX - 511},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_card();
        finish(trim(&device, cases[i].addr, cases[i].length), SD_BLOCK_DEVICE_ERROR_PARAMETER);
    }
    reset_card();
    config.erase_size = 0;
    finish(trim(&device, 0, 512), SD_BLOCK_DEVICE_ERROR_PARAMETER);
}

static void test_trim_rounded_timeout(bool high_capacity) {
    reset_card();
    config.card_type = high_capacity ? SDCARD_V2HC : SDCARD_V1;
    config.erase_size = 1024;
    expect_init_command(CMD32_ERASE_WR_BLK_START_ADDR, high_capacity ? 2 : 1024, 0, 0);
    expect_init_command(CMD33_ERASE_WR_BLK_END_ADDR, high_capacity ? 39 : 19968, 0, 0);
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD38_ERASE, 0);
    expect_spi(0xFF, 0, false, 1);
    // Rounding 40 requested logical sectors leaves 38: the busy budget is 9.5 s.
    expect_spi(0xFF, 0, false, 9500);
    expect_release();
    finish(trim(&device, 512, 40 * 512), SD_BLOCK_DEVICE_ERROR_NO_RESPONSE);
}

static void test_trim_rounded_rejection(void) {
    reset_card();
    config.erase_size = 1024;
    expect_init_command(CMD32_ERASE_WR_BLK_START_ADDR, 1024, 0, 0);
    expect_init_command(CMD33_ERASE_WR_BLK_END_ADDR, 2560, 0, 0);
    expect_init_command(CMD38_ERASE, 0, R1_ILLEGAL_COMMAND, 0);
    finish(trim(&device, 512, 3072), SD_BLOCK_DEVICE_ERROR_UNSUPPORTED);
}

static void test_trim_partial_tail_format(void) {
    for (unsigned crc_enabled = 0; crc_enabled < 2; ++crc_enabled) {
        reset_init_card(crc_enabled);
        uint8_t csd[16];
        make_ordinary_csd(csd, false);
        set_csd_bits(csd, 73, 62, 1022);
        set_csd_bits(csd, 25, 22, 10);
        set_csd_bits(csd, 21, 21, 1);
        csd[15] = reference_crc7(csd, 15);
        expect_card_init(false, false);
        expect_csd(csd, false);
        expect_init_command(CMD16_SET_BLOCKLEN, 512, 0, 0);
        finish_lifecycle(init, BD_ERROR_OK);
        CHECK(device.size(&device) == 2095104 && config.erase_size == 4096);
        CHECK(device.read_size == 512 && device.program_size == 512 && device.erase_size == 512);
        device.erase = erase;

        expect_trim_range(0, 2093056);
        observing_ready_io = true;
        filesystem_io = true;
        int status = test_sd_fat_format(&device);
        filesystem_io = false;
        finish(status, BD_ERROR_OK);

        uint8_t buffer[512];
        memset(buffer, 0x31, sizeof(buffer));
        bd_size_t tail = device.size(&device) - 512;
        finish(device.erase(&device, tail, sizeof(buffer)), BD_ERROR_OK);
        expect_select();
        expect_spi(0xFF, 0xFF, false, 1);
        expect_command(CMD24_WRITE_BLOCK, tail);
        expect_spi(0xFF, 0, false, 1);
        expect_spi(SPI_START_BLOCK, 0xFF, false, 1);
        expect_spi(0x31, 0xFF, false, sizeof(buffer));
        uint16_t crc = crc_enabled ? reference_crc16(buffer, sizeof(buffer)) : 0xFFFF;
        expect_spi(crc >> 8, 0xFF, false, 1);
        expect_spi(crc, 0xFF, false, 1);
        expect_spi(0xFF, SPI_DATA_ACCEPTED, false, 1);
        expect_spi(0xFF, 0xFF, false, 1);
        expect_release();
        finish(program(&device, buffer, tail, sizeof(buffer)), BD_ERROR_OK);

        memset(buffer, 0, sizeof(buffer));
        expect_start(1, tail, 0);
        expect_block(0x31);
        expect_release();
        finish(read(&device, buffer, tail, sizeof(buffer)), BD_ERROR_OK);
        for (size_t i = 0; i < sizeof(buffer); ++i) {
            CHECK(buffer[i] == 0x31);
        }
        observing_ready_io = false;
        CHECK(config.is_initialized && device.is_initialized);
    }
}

static void test_trim_geometry(void) {
    const struct {
        bool high_capacity;
        bool single_block_erase;
        uint8_t read_bl_len;
        uint8_t write_bl_len;
        uint8_t sector_size;
        size_t trim_size;
    } cases[] = {
        {false, false, 9, 9, 0, 512},
        {false, false, 9, 10, 0, 1024},
        {false, false, 9, 11, 0, 2048},
        {false, false, 9, 9, 2, 1536},
        {false, false, 9, 9, 3, 2048},
        {false, false, 11, 9, 3, 2048},
        {false, false, 9, 10, 3, 4096},
        {false, false, 9, 11, 3, 8192},
        {false, false, 9, 11, 127, 262144},
        {false, true, 9, 9, 127, 512},
        {false, true, 9, 10, 127, 512},
        {false, true, 9, 11, 127, 512},
        {true, true, 9, 9, 127, 512},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_card();
        config.hz = CONF_SD_TRX_FREQUENCY;
        uint8_t csd[16];
        make_ordinary_csd(csd, cases[i].high_capacity);
        set_csd_bits(csd, 83, 80, cases[i].read_bl_len);
        set_csd_bits(csd, 79, 79, cases[i].read_bl_len > 9);
        set_csd_bits(csd, 46, 46, cases[i].single_block_erase);
        set_csd_bits(csd, 45, 39, cases[i].sector_size);
        set_csd_bits(csd, 25, 22, cases[i].write_bl_len);
        set_csd_bits(csd, 21, 21, cases[i].write_bl_len > 9);
        csd[15] = reference_crc7(csd, 15);
        expect_card_init(false, cases[i].high_capacity);
        expect_csd(csd, false);
        expect_init_command(CMD16_SET_BLOCKLEN, 512, 0, 0);

        finish(init(&device), BD_ERROR_OK);
        size_t expected = cases[i].trim_size;
        CHECK(device.erase_size == 512);
        CHECK(config.erase_size == expected);
        bd_size_t end = device.size(&device);
        end -= end % expected;
        const bd_size_t addresses[] = {0, expected, end - 2 * expected};
        for (size_t range = 0; range < sizeof(addresses) / sizeof(addresses[0]); ++range) {
            for (int cmd = CMD32_ERASE_WR_BLK_START_ADDR; cmd <= CMD33_ERASE_WR_BLK_END_ADDR; ++cmd) {
                uint32_t address = cmd == CMD32_ERASE_WR_BLK_START_ADDR ? addresses[range] :
                                   addresses[range] + 2 * expected - 512;
                if (cases[i].high_capacity) {
                    address /= 512;
                }
                expect_init_command(cmd, address, 0, 0);
            }
            expect_select();
            expect_spi(0xFF, 0xFF, false, 1);
            expect_command(CMD38_ERASE, 0);
            expect_spi(0xFF, 0, false, 1);
            expect_spi(0xFF, 0xFF, false, 1);
            expect_release();
            finish(trim(&device, addresses[range], 2 * expected), BD_ERROR_OK);
        }
        expect_trim_range(0, end);
        finish(trim(&device, 0, device.size(&device)), BD_ERROR_OK);
        if (expected > 512) {
            finish(trim(&device, 512, expected), BD_ERROR_OK);
            finish(trim(&device, expected, 512), BD_ERROR_OK);
            finish(trim(&device, expected + 512, expected - 512), BD_ERROR_OK);
        }
        finish(trim(&device, end, expected), SD_BLOCK_DEVICE_ERROR_PARAMETER);
        exercise_initialized_io(expected);
    }
}

static void test_trim_invalid_write_length(void) {
    for (unsigned crc = 0; crc < 2; ++crc) {
        for (unsigned write_bl_len = 0; write_bl_len < 9; ++write_bl_len) {
            reset_init_card(crc);
            uint8_t csd[16];
            make_ordinary_csd(csd, false);
            set_csd_bits(csd, 25, 22, write_bl_len);
            csd[15] = reference_crc7(csd, 15);
            expect_card_init(false, false);
            expect_csd(csd, false);
            finish_lifecycle(init, BD_ERROR_DEVICE_ERROR);
            CHECK(config.total_sectors == 0);
            finish(trim(&device, 0, 512), SD_BLOCK_DEVICE_ERROR_NO_INIT);
        }
    }
}

// R3/R7 continuations are assembled as unsigned 32-bit values. In particular,
// CMD58 OCR bit 31 must not be promoted and shifted as a signed int.
static void test_response_arithmetic(void) {
    static const struct {
        int cmd;
        uint8_t r1;
        uint8_t bytes[4];
        uint32_t expected;
    } cases[] = {
        {CMD8_SEND_IF_COND, R1_IDLE_STATE, {0x81, 0xA5, 0x5A, 0xC3}, 0x81A55AC3},
        {CMD58_READ_OCR, 0, {0xC0, 0xFF, 0x80, 0x00}, 0xC0FF8000},
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_card();
        expect_select();
        expect_spi(0xFF, 0xFF, false, 1);
        expect_command(cases[i].cmd, 0);
        expect_spi(0xFF, cases[i].r1, false, 1);
        for (size_t byte = 0; byte < sizeof(cases[i].bytes); ++byte) {
            expect_spi(0xFF, cases[i].bytes[byte], false, 1);
        }
        expect_release();

        uint32_t response = R1_NO_RESPONSE;
        finish(_cmd(&config, cases[i].cmd, 0, false, &response), BD_ERROR_OK);
        CHECK(response == cases[i].expected);
    }
}


// SPI1 RX includes GPIO24 and GPIO28 (RP2040 datasheet, section 1.4.3,
// Table 2, pp.12-13); SD does not specify the host's GPIO routing. The
// ordinary CSD fixture is only enough to initialize a small SDSC card.
static void test_create_spi1_miso(unsigned miso) {
    reset_card();
    config.spi_inst = spi1;
    expect_spi(0xFF, 0xFF, true, 10);
    expect_init_command(CMD0_GO_IDLE_STATE, 0, 1, 0);
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD8_SEND_IF_COND, 0x1AA);
    expect_spi(0xFF, R1_ILLEGAL_COMMAND | R1_IDLE_STATE, false, 1);
    expect_release();
    expect_init_command(CMD59_CRC_ON_OFF, 1, 1, 0);
    expect_init_command(CMD58_READ_OCR, 0, 1, OCR_3_3V);
    expect_init_command(ACMD41_SD_SEND_OP_COND, 0, 0, 0);

    uint8_t csd[16];
    make_ordinary_csd(csd, false);
    uint16_t crc = reference_crc16(csd, sizeof(csd));
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD9_SEND_CSD, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(0xFF, 0xFE, false, 1);
    for (size_t i = 0; i < sizeof(csd); ++i) {
        expect_spi(0xFF, csd[i], false, 1);
    }
    expect_spi(0xFF, crc >> 8, false, 1);
    expect_spi(0xFF, crc, false, 1);
    expect_release();
    expect_init_command(CMD16_SET_BLOCKLEN, 512, 0, 0);

    blockdevice_t *created =
        blockdevice_sd_create(spi1, 27, miso, 26, 17, 12000000, true);
    CHECK(created != NULL);
    blockdevice_sd_config_t *created_config = created->config;
    CHECK(created->is_initialized && created_config->is_initialized);
    CHECK(created_config->spi_inst == spi1 && created_config->miso == miso);
    CHECK(event_index == event_count && !created_config->_mutex.locked);
    blockdevice_sd_free(created);
}

int main(void) {
    RUN_CASE("init-csd-initial-zero", test_init_csd_initial_zero());
    RUN_CASE("csd-initial-zero-rejection", test_csd_initial_zero_rejection());
    RUN_CASE("read-token-rejection", test_read_token_rejection());
    RUN_CASE("shifted-read-rejection", test_shifted_read_rejection());
    RUN_CASE("csd-token-rejection", test_register_token_rejection(false));
    RUN_CASE("sd-status-token-rejection", test_register_token_rejection(true));
    RUN_CASE("multiblock-token-rejection", test_multiblock_token_rejection());
    RUN_CASE("response-arithmetic", test_response_arithmetic());
    RUN_CASE("create-spi1-miso24", test_create_spi1_miso(24));
    RUN_CASE("create-spi1-miso28", test_create_spi1_miso(28));
    RUN_CASE("initialized-invalid-io", test_initialized_invalid_io());
    RUN_CASE("cold-sdsc-crc", test_cold_init(false, true));
    RUN_CASE("cold-sdsc-no-crc", test_cold_init(false, false));
    RUN_CASE("cold-sdhc-crc", test_cold_init(true, true));
    RUN_CASE("cold-sdhc-no-crc", test_cold_init(true, false));
    RUN_CASE("uninitialized-io", test_uninitialized_io(false));
    RUN_CASE("deinitialized-io", test_uninitialized_io(true));
    RUN_CASE("top-init", test_top_init(0));
    RUN_CASE("top-init-csd-error", test_top_init(1));
    RUN_CASE("top-init-cmd16-error", test_top_init(2));
    RUN_CASE("top-init-frequency-error", test_top_init(3));
    RUN_CASE("top-init-card-error", test_top_init(4));
    RUN_CASE("deinit-reinit", test_deinit_reinit());
    RUN_CASE("acmd13-status-data", test_acmd13_data(false));
    RUN_CASE("acmd13-status-crc", test_acmd13_data(true));
    RUN_CASE("cmd13-release", test_cmd13_release());
    RUN_CASE("r2-status", test_r2_status());
    RUN_CASE("init-crc-enabled", test_init_crc(true, false));
    RUN_CASE("init-crc-disabled", test_init_crc(false, false));
    RUN_CASE("init-crc-rejected", test_init_crc(true, true));
    RUN_CASE("init-types", test_init_types());
    RUN_CASE("init-crc-idle-recovery-enabled", test_init_crc_recovery(true, 0x09));
    RUN_CASE("init-crc-idle-recovery-disabled", test_init_crc_recovery(false, 0x09));
    RUN_CASE("init-crc-recovery-enabled", test_init_crc_recovery(true, 0x08));
    RUN_CASE("init-crc-recovery-disabled", test_init_crc_recovery(false, 0x08));
    RUN_CASE("init-crc-idle-timeout-enabled", test_init_crc_timeout(true, 0x09));
    RUN_CASE("init-crc-idle-timeout-disabled", test_init_crc_timeout(false, 0x09));
    RUN_CASE("init-crc-timeout-enabled", test_init_crc_timeout(true, 0x08));
    RUN_CASE("init-crc-timeout-disabled", test_init_crc_timeout(false, 0x08));
    RUN_CASE("init-failures", test_init_failures());
    RUN_CASE("init-acmd41-no-response", test_init_transport_failure(false, false));
    RUN_CASE("init-acmd41-busy", test_init_transport_failure(false, true));
    RUN_CASE("init-final-ocr-no-response", test_init_transport_failure(true, false));
    RUN_CASE("init-final-ocr-busy", test_init_transport_failure(true, true));
    RUN_CASE("singleblock", test_singleblock());
    RUN_CASE("sectors", test_sectors());
    RUN_CASE("singleblock-program-status", test_singleblock_program());
    RUN_CASE("multiblock-program-success", test_multiblock_program_success(false));
    RUN_CASE("multiblock-program-stop-timeout", test_multiblock_program_success(true));
    RUN_CASE("multiblock-stop-timing", test_multiblock_stop_timing());
    RUN_CASE("multiblock-program-failure", test_multiblock_program_failure());
    RUN_CASE("start-rejected", test_start_rejected());
    RUN_CASE("token-busy-wrap", test_token_wrap(false));
    RUN_CASE("token-timeout-wrap", test_token_wrap(true));
    RUN_CASE("wide-busy-timeout", test_wide_busy_timeout());
    RUN_CASE("power-up", test_power_up());
    RUN_CASE("start-no-response", test_no_response(false));
    RUN_CASE("stop-unaligned", test_stop_unaligned(0x1F, BD_ERROR_OK));
    RUN_CASE("stop-unaligned-illegal", test_stop_unaligned(0x9F, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED));
    RUN_CASE("stop-no-response", test_no_response(true));
    for (unsigned shift = 0; shift < 8; ++shift) {
        char name[64];
        snprintf(name, sizeof(name), "command-alignment-%u", shift);
        RUN_CASE(name, test_command_alignment(shift));
        snprintf(name, sizeof(name), "command-stuff-%u", shift);
        RUN_CASE(name, test_command_stuff(shift));
        snprintf(name, sizeof(name), "command-retry-%u", shift);
        RUN_CASE(name, test_command_retry(shift));
        snprintf(name, sizeof(name), "app-prefix-failure-%u", shift);
        RUN_CASE(name, test_app_prefix_failure(shift));
        snprintf(name, sizeof(name), "app-prefix-retry-%u", shift);
        RUN_CASE(name, test_app_prefix_retry(shift));
        snprintf(name, sizeof(name), "busy-alignment-%u", shift);
        RUN_CASE(name, test_busy_alignment(shift));
        snprintf(name, sizeof(name), "read-command-alignment-%u", shift);
        RUN_CASE(name, test_read_command_alignment(shift));
    }
    RUN_CASE("stop-first-response", test_stop_status(0, BD_ERROR_OK, 0));
    RUN_CASE("stop-illegal", test_stop_status(0x04, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED, 2));
    RUN_CASE("command-busy-timeout", test_command_busy(false));
    RUN_CASE("app-command-busy-timeout", test_command_busy(true));
    RUN_CASE("multiblock-sdsc", test_multiblock(false, true));
    RUN_CASE("multiblock-sdhc", test_multiblock(true, true));
    RUN_CASE("multiblock-token-timeout", test_multiblock_data_failure(false, 0, false));
    RUN_CASE("multiblock-token-timeout-stop-error",
             test_multiblock_data_failure(false, 0x04, false));
    RUN_CASE("multiblock-data-crc", test_multiblock_data_failure(true, 0, false));
    RUN_CASE("multiblock-data-crc-stop-error",
             test_multiblock_data_failure(true, 0x04, false));
    RUN_CASE("singleblock-crc", test_singleblock_crc());
    RUN_CASE("read-crc-stop-busy", test_multiblock_data_failure(true, 0, true));
    RUN_CASE("crc-disabled", test_multiblock(false, false));
    RUN_CASE("repeated-read", test_repeated_read());
    RUN_CASE("stop-crc", test_stop_status(0x08, SD_BLOCK_DEVICE_ERROR_CRC, 2));
    RUN_CASE("stop-parameter", test_stop_status(0x40, SD_BLOCK_DEVICE_ERROR_PARAMETER, 2));
    RUN_CASE("stop-erase", test_stop_status(0x10, SD_BLOCK_DEVICE_ERROR_ERASE, 2));
    RUN_CASE("stop-no-busy", test_stop_status(0, BD_ERROR_OK, 2));
    // Seven delayed bytes plus the stuff byte exercise eight filler bytes.
    RUN_CASE("stop-max-response-delay", test_stop_status(0, BD_ERROR_OK, 7));
    RUN_CASE("stop-idle", test_stop_status(0x01, SD_BLOCK_DEVICE_ERROR_NO_INIT, 2));
    RUN_CASE("idle-cmd16", test_idle_command(16, false, 1, SD_BLOCK_DEVICE_ERROR_NO_INIT));
    RUN_CASE("idle-cmd32", test_idle_command(32, false, 1, SD_BLOCK_DEVICE_ERROR_NO_INIT));
    RUN_CASE("idle-cmd33", test_idle_command(33, false, 1, SD_BLOCK_DEVICE_ERROR_NO_INIT));
    RUN_CASE("idle-cmd38", test_idle_command(38, false, 1, SD_BLOCK_DEVICE_ERROR_NO_INIT));
    RUN_CASE("idle-cmd0", test_idle_command(0, false, 1, BD_ERROR_OK));
    RUN_CASE("idle-cmd1", test_idle_command(1, false, 1, BD_ERROR_OK));
    RUN_CASE("idle-cmd8", test_idle_command(8, false, 1, BD_ERROR_OK));
    RUN_CASE("idle-cmd58", test_idle_command(58, false, 1, BD_ERROR_OK));
    RUN_CASE("idle-cmd59", test_idle_command(59, false, 1, BD_ERROR_OK));
    RUN_CASE("idle-acmd41", test_idle_command(41, true, 1, BD_ERROR_OK));
    RUN_CASE("idle-bare-cmd41", test_idle_command(41, false, 1, SD_BLOCK_DEVICE_ERROR_NO_INIT));
    RUN_CASE("idle-parameter-error", test_idle_command(16, false, 0x41, SD_BLOCK_DEVICE_ERROR_PARAMETER));
    RUN_CASE("idle-erase-error", test_idle_command(16, false, 0x11, SD_BLOCK_DEVICE_ERROR_ERASE));
    RUN_CASE("stop-busy-timeout", test_stop_busy(false, 0, SD_BLOCK_DEVICE_ERROR_NO_RESPONSE));
    RUN_CASE("stop-busy-wrap", test_stop_busy(true, 0, SD_BLOCK_DEVICE_ERROR_NO_RESPONSE));
    RUN_CASE("stop-busy-parameter", test_stop_busy(false, 0x40, SD_BLOCK_DEVICE_ERROR_PARAMETER));
    RUN_CASE("stop-busy-erase", test_stop_busy(false, 0x10, SD_BLOCK_DEVICE_ERROR_ERASE));
    RUN_CASE("erase-busy-timeout", test_erase_busy_timeout());
    RUN_CASE("trim-sdsc-long-busy", test_trim_busy(false, false, false));
    RUN_CASE("trim-sdhc-long-busy", test_trim_busy(true, false, false));
    RUN_CASE("trim-busy-timeout", test_trim_busy(true, true, false));
    RUN_CASE("trim-busy-wrap", test_trim_busy(true, false, true));
    RUN_CASE("init-clock", test_init_clock());
    RUN_CASE("trim-geometry", test_trim_geometry());
    RUN_CASE("trim-invalid-write-length", test_trim_invalid_write_length());
    RUN_CASE("advisory-trim-sdsc", test_advisory_trim(false));
    RUN_CASE("advisory-trim-sdhc", test_advisory_trim(true));
    RUN_CASE("trim-invalid-bounds", test_trim_invalid_bounds());
    RUN_CASE("trim-sdsc-rounded-timeout", test_trim_rounded_timeout(false));
    RUN_CASE("trim-sdhc-rounded-timeout", test_trim_rounded_timeout(true));
    RUN_CASE("trim-rounded-rejection", test_trim_rounded_rejection());
    RUN_CASE("trim-partial-tail-format", test_trim_partial_tail_format());
    return EXIT_SUCCESS;
}
