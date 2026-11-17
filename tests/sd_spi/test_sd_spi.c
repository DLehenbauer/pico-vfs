#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../src/blockdevice/sd.c"

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
spi_inst_t mock_spi_instances[2] = {{0}, {1}};

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
    CHECK(spi == config.spi_inst);
    initialized_baudrate = baudrate;
    return baudrate;
}

unsigned spi_set_baudrate(spi_inst_t *spi, unsigned baudrate) {
    CHECK(spi == config.spi_inst);
    transfer_baudrate = baudrate;
    return baudrate;
}

void spi_set_format(spi_inst_t *spi, unsigned bits, unsigned cpol, unsigned cpha, unsigned order) {
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
}

void mutex_enter_blocking(mutex_t *mutex) {
    CHECK(!mutex->locked);
    mutex->locked = true;
}

void mutex_exit(mutex_t *mutex) {
    CHECK(mutex->locked);
    mutex->locked = false;
}

static void reset_card(void) {
    event_count = event_index = 0;
    cs_high = true;
    now_us = 0;
    time_step_us = 1000;
    startup_pending = false;
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

// Independent bitwise CRC7, x^7+x^3+1 (SD v9.10, section 4.5, p.93).
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

// Independent bitwise CRC16, x^16+x^12+x^5+1 (SD v9.10, section 4.5,
// p.94; SPI data uses this polynomial too, section 7.2.3, p.289).
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

static void fill_pattern(uint8_t *buffer, size_t length) {
    // Test design, not an SD data pattern: expose repeated-byte, ordering, and
    // block-stride errors that constant payloads cannot distinguish.
    for (size_t i = 0; i < length; ++i) {
        buffer[i] = i * 37 + (i / 256) * 17 + 11;
    }
}

// SPI blocks carry a start token, payload, and two CRC bytes
// (SD v9.10, section 7.3.3.2, p.305), including when CRC checking is disabled.
static void expect_read_payload(const uint8_t *data, size_t length, bool corrupt_crc) {
    uint16_t crc = reference_crc16(data, length) ^ (corrupt_crc ? 1 : 0);
    expect_spi(0xFF, SPI_START_BLOCK, false, 1);
    for (size_t i = 0; i < length; ++i) {
        expect_spi(0xFF, data[i], false, 1);
    }
    expect_spi(0xFF, crc >> 8, false, 1);
    expect_spi(0xFF, crc, false, 1);
}

static void expect_write_payload(const uint8_t *data, size_t length, uint8_t start) {
    uint16_t crc = config.enable_crc ? reference_crc16(data, length) : 0xFFFF;
    expect_spi(start, 0xFF, false, 1);
    for (size_t i = 0; i < length; ++i) {
        expect_spi(data[i], 0xFF, false, 1);
    }
    expect_spi(crc >> 8, 0xFF, false, 1);
    expect_spi(crc, 0xFF, false, 1);
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

static void expect_command(int cmd, uint32_t arg) {
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

static void expect_program_data(uint8_t value, uint8_t start) {
    uint8_t buffer[512];
    memset(buffer, value, sizeof(buffer));
    uint16_t crc = config.enable_crc ? reference_crc16(buffer, sizeof(buffer)) : 0xFFFF;
    expect_spi(start, 0xFF, false, 1);
    expect_spi(value, 0xFF, false, sizeof(buffer));
    expect_spi(crc >> 8, 0xFF, false, 1);
    expect_spi(crc, 0xFF, false, 1);
}

static void expect_program_block(uint8_t value, uint8_t start, uint8_t response, unsigned shift,
                                  bool timeout) {
    expect_program_data(value, start);
    uint8_t token = response << 3;
    expect_response_bits(&token, 5, 3 + shift, true);
    expect_spi(0xFF, 0, false, timeout ? CONF_SD_CMD_TIMEOUT : 2);
    if (!timeout) {
        expect_spi(0xFF, 0xFF, false, 1);
    }
}

static void expect_multiblock_program(unsigned shift) {
    uint8_t r1 = 0;
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD55_APP_CMD, 0);
    expect_response_bits(&r1, 8, (shift + 1) % 8, false);
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(ACMD23_SET_WR_BLK_ERASE_COUNT, 2);
    r1 = 0;
    expect_response_bits(&r1, 8, shift, false);
    expect_release();
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD25_WRITE_MULTIPLE_BLOCK, 0);
    expect_response_bits(&r1, 8, (shift + 3) % 8, false);
}

// CMD12 stays in the selected read stream (SanDisk v2.2, section 5.23.3, p.5-18).
// Discard the stuff byte, following Elm-Chan read guidance: "The received byte
// immediately following CMD12 is a stuff byte" (See: https://elm-chan.org/docs/mmc/mmc_e.html#dataxfer)
static void expect_stop(uint8_t response, size_t busy_bytes, size_t response_delay, bool ready) {
    expect_command(12, 0);
    // An error-looking stuff byte must not be interpreted as the actual R1.
    expect_spi(0xFF, 0x04, false, 1);
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
    if (!rejected) {
        CHECK(config.card_type == SDCARD_V2HC);
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
        if (stop) {
            // An error-looking stuff byte must be discarded before each response.
            expect_spi(0xFF, 0x04, false, 1);
        }
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
    expect_command(12, 0);
    expect_spi(0xFF, 0x3F, false, 1);
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
        {CMD8_SEND_IF_COND, false, {1, 0x81, 0xA5, 0x5A, 0xC3}, 5, 0x81A55AC3, BD_ERROR_OK},
        {CMD8_SEND_IF_COND, false, {0x08}, 1, 0x08, SD_BLOCK_DEVICE_ERROR_CRC},
        {CMD8_SEND_IF_COND, false, {0x04}, 1, 0x04, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED},
        {CMD58_READ_OCR, false, {0, 0xC0, 0xFF, 0x80, 0}, 5, 0xC0FF8000, BD_ERROR_OK},
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
                // Last response-poll slot in the driver's existing 16-byte guard.
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
                expect_read_payload(expected, sizeof(expected), false);
            }
            expect_release();
            uint32_t response = R1_NO_RESPONSE;
            int status = _cmd(&config, cases[i].cmd, 0, cases[i].app, &response);
            CHECK(response == cases[i].response);
            if (cases[i].cmd == CMD8_SEND_IF_COND) {
                CHECK(config.card_type == (cases[i].status == BD_ERROR_OK ? SDCARD_V2 :
                                          cases[i].status == SD_BLOCK_DEVICE_ERROR_UNSUPPORTED ?
                                          CARD_UNKNOWN : SDCARD_V1));
            }
            if (data) {
                CHECK(status == BD_ERROR_OK);
                CHECK(!cs_high);
                uint8_t buffer[64];
                status = _read_bytes(&config, buffer, sizeof(buffer));
                CHECK(memcmp(buffer, expected, sizeof(buffer)) == 0);
            }
            finish(status, cases[i].status);
        }
    }
    current_case = outer_case;
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
            expect_spi(0xFF, 0x04, false, 1);
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

static void test_write_alignment(unsigned shift, bool enable_crc) {
    // Accepted, CRC-rejected, and write-rejected tokens are defined by
    // SD v9.10, section 7.3.3.1, p.305; the remaining values must not succeed.
    static const uint8_t tokens[] = {0x05, 0x0B, 0x0D, 0x01, 0x0F, 0x00};
    for (size_t i = 0; i < sizeof(tokens); ++i) {
        reset_card();
        config.enable_crc = enable_crc;
        device.program_size = 512;
        expect_select();
        expect_spi(0xFF, 0xFF, false, 1);
        expect_command(CMD24_WRITE_BLOCK, 0);
        expect_spi(0xFF, 0, false, 1);
        uint8_t buffer[512];
        memset(buffer, 0x31, sizeof(buffer));
        expect_program_data(0x31, SPI_START_BLOCK);
        uint8_t token = tokens[i] << 3;
        expect_response_bits(&token, 5, 3 + shift, false);
        expect_spi(0xFF, 0xFF, false, 1);
        expect_release();
        finish(program(&device, buffer, 0, sizeof(buffer)),
               tokens[i] == SPI_DATA_ACCEPTED ? BD_ERROR_OK : SD_BLOCK_DEVICE_ERROR_WRITE);
    }
}

static void test_write_busy_timeout(unsigned shift) {
    static const uint8_t tokens[] = {SPI_DATA_ACCEPTED, SPI_DATA_CRC_ERROR, SPI_DATA_WRITE_ERROR};
    for (size_t i = 0; i < sizeof(tokens); ++i) {
        reset_card();
        device.program_size = 512;
        expect_select();
        expect_spi(0xFF, 0xFF, false, 1);
        expect_command(CMD24_WRITE_BLOCK, 0);
        expect_spi(0xFF, 0, false, 1);
        expect_program_block(0x31, SPI_START_BLOCK, tokens[i], shift, true);
        expect_release();
        uint8_t buffer[512];
        memset(buffer, 0x31, sizeof(buffer));
        finish(program(&device, buffer, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_RESPONSE);
    }
}

static void test_multiblock_program(unsigned shift, bool stop_timeout, bool enable_crc) {
    reset_card();
    config.enable_crc = enable_crc;
    device.program_size = 512;
    expect_multiblock_program(shift);
    expect_program_block(0x31, SPI_START_BLK_MUL_WRITE, SPI_DATA_ACCEPTED, shift, false);
    expect_program_block(0xA6, SPI_START_BLK_MUL_WRITE, SPI_DATA_ACCEPTED, 7 - shift, false);
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
        unsigned delay;
        unsigned busy_bits;
    } cases[] = {{0, 0}, {0, 1}, {0, 16}, {7, 2}, {8, 1}, {8, 16}};
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_card();
        device.program_size = 512;
        expect_multiblock_program(0);
        expect_program_block(0x31, SPI_START_BLK_MUL_WRITE, SPI_DATA_ACCEPTED, 0, false);
        expect_program_block(0x31, SPI_START_BLK_MUL_WRITE, SPI_DATA_ACCEPTED, 0, false);
        expect_spi(SPI_STOP_TRAN, 0xFF, false, 1);
        if (cases[i].busy_bits) {
            const uint8_t busy[2] = {0};
            expect_response_bits(busy, cases[i].busy_bits, cases[i].delay, false);
        } else {
            expect_spi(0xFF, 0xFF, false, 1);
        }
        expect_spi(0xFF, 0xFF, false, 1);
        expect_release();
        uint8_t buffer[1024];
        memset(buffer, 0x31, sizeof(buffer));
        finish(program(&device, buffer, 0, sizeof(buffer)), BD_ERROR_OK);
    }
}

static void test_program_response_failure(bool multiple) {
    static const struct {
        uint8_t last_byte;
        bool busy_timeout;
    } cases[] = {{0xFF, false}, {0xFF, true}, {0xF2, false}, {0xF2, true}};
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_card();
        device.program_size = 512;
        if (multiple) {
            expect_multiblock_program(0);
        } else {
            expect_select();
            expect_spi(0xFF, 0xFF, false, 1);
            expect_command(CMD24_WRITE_BLOCK, 0);
            expect_spi(0xFF, 0, false, 1);
        }
        expect_program_data(0x31, multiple ? SPI_START_BLK_MUL_WRITE : SPI_START_BLOCK);
        expect_spi(0xFF, 0xFF, false, 3);
        // F2 ends in a truncated 0010 token. Reading a fifth byte of FF would
        // incorrectly complete it as 00101 (accepted).
        expect_spi(0xFF, cases[i].last_byte, false, 1);
        expect_spi(0xFF, cases[i].busy_timeout ? 0 : 0xFF, false,
                   cases[i].busy_timeout ? CONF_SD_CMD_TIMEOUT : 1);
        if (multiple && !cases[i].busy_timeout) {
            expect_stop(0, 0, 0, true);
        } else {
            expect_release();
        }
        uint8_t buffer[1024];
        memset(buffer, 0x31, sizeof(buffer));
        finish(program(&device, buffer, 0, multiple ? sizeof(buffer) : 512),
               cases[i].busy_timeout ? SD_BLOCK_DEVICE_ERROR_NO_RESPONSE :
                                       SD_BLOCK_DEVICE_ERROR_WRITE);
    }
}

// CRC/write rejection requires CMD12 (SD v9.10, section 7.3.3.1, p.305), but
// commands are rejected while busy (section 7.2.8, p.292). Only abort after ready.
// On busy timeout, release the bus without another command. Error precedence
// and the timeout budget are driver policy.
static void test_multiblock_program_failure(unsigned shift) {
    static const struct {
        uint8_t token;
        bool write_timeout;
        uint8_t abort_response;
        bool abort_timeout;
        bool second_block;
    } cases[] = {
        {0x0B, false, 0, false, false},
        {0x0D, false, 0x04, false, false},
        {0x05, true, 0, false, false},
        {0x0B, true, 0, false, false},
        {0x0D, true, 0, false, true},
        {0x0B, false, 0, true, false},
        {0x0B, false, 0, false, true},
        {0x05, true, 0, false, true},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_card();
        device.program_size = 512;
        expect_multiblock_program(shift);
        if (cases[i].second_block) {
            expect_program_block(0x31, SPI_START_BLK_MUL_WRITE, SPI_DATA_ACCEPTED, 7 - shift,
                                 false);
        }
        expect_program_block(cases[i].second_block ? 0xA6 : 0x31, SPI_START_BLK_MUL_WRITE,
                             cases[i].token, shift, cases[i].write_timeout);
        if (cases[i].write_timeout) {
            // A busy timeout releases the bus without sending CMD12 or waiting again.
            expect_release();
        } else {
            expect_stop(cases[i].abort_response, cases[i].abort_timeout ? CONF_SD_CMD_TIMEOUT : 0,
                        0, !cases[i].abort_timeout);
        }
        uint8_t buffer[1024];
        memset(buffer, 0x31, 512);
        memset(buffer + 512, 0xA6, 512);
        finish(program(&device, buffer, 0, sizeof(buffer)),
               cases[i].write_timeout ? SD_BLOCK_DEVICE_ERROR_NO_RESPONSE :
                                       SD_BLOCK_DEVICE_ERROR_WRITE);
    }
}

// xxx0sss1 leaves the first three bits undefined (SD v9.10, section 7.3.3.1,
// p.305). Prefix invariance and the four-byte host-policy cap are separate checks.
static void test_write_response_bounds(void) {
    for (unsigned prefix = 0; prefix < 8; ++prefix) {
        for (unsigned token = 0; token < 16; ++token) {
            reset_card();
            cs_high = false;
            expect_spi(0xFF, (prefix << 5) | token, false, 1);
            CHECK(_read_data_response(&config) == token);
            CHECK(event_index == event_count);
        }
    }

    reset_card();
    cs_high = false;
    expect_spi(0xFF, 0xFF, false, 3);
    expect_spi(0xFF, 0xE5, false, 1);
    CHECK(_read_data_response(&config) == SPI_DATA_ACCEPTED);
    CHECK(event_index == event_count);

    reset_card();
    cs_high = false;
    expect_spi(0xFF, 0xFF, false, 3);
    expect_spi(0xFF, 0xF2, false, 1);
    CHECK(_read_data_response(&config) == 0xFF);
    CHECK(event_index == event_count);

    reset_card();
    cs_high = false;
    expect_spi(0xFF, 0xFF, false, 4);
    CHECK(_read_data_response(&config) == 0xFF);
    CHECK(event_index == event_count);
}

static void test_write_response_windows(void) {
    for (unsigned leading = 3; leading < 32; ++leading) {
        for (unsigned response = 0; response < 16; ++response) {
            reset_card();
            cs_high = false;
            uint8_t token = response << 3;
            bool complete = leading <= 27;
            expect_response_bits(&token, complete ? 5 : 32 - leading, leading, true);
            CHECK(_read_data_response(&config) == (complete ? response : 0xFF));
            CHECK(event_index == event_count);
        }
    }
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
// p.5-18). The exact sequence without extra clocks or ready polling is driver
// policy, not a requirement to issue CMD12 immediately.
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
// Waiting for 0xFE does not immediately decode read-error tokens (SD v9.10,
// section 7.3.3.3, p.306).
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

// Forty sectors give a 10 s erase estimate (250 ms/block; SD v9.10,
// section 4.6.2.3, p.96). Exercise both address formats, expiration, and rollover.
static void test_trim_busy(bool high_capacity, bool timeout, bool wrap) {
    reset_card();
    config.card_type = high_capacity ? SDCARD_V2HC : SDCARD_V1;
    if (wrap) {
        // Four mock time reads precede busy polling. Start early enough that
        // the wait itself spans rollover; this is a host-clock regression.
        now_us = ((uint64_t)UINT32_MAX - 10) * 1000;
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

static void expect_preerase(size_t blocks, uint8_t response) {
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD55_APP_CMD, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(ACMD23_SET_WR_BLK_ERASE_COUNT, blocks);
    expect_spi(0xFF, response, false, 1);
    expect_release();
}

// Byte/block address units: SD v9.10, Table 7-3, footnote 10, p.299.
// Nonuniform three-block transfers check payload order as well as command choice.
// The final case uses a synthetic capacity at the 32-bit LBA encoding limit,
// not the smaller specified SDXC user-area maximum (section 5.3.3, p.235).
static void test_patterned_io(bool writing, bool crc_enabled) {
    static const struct {
        uint8_t type;
        uint64_t address;
        size_t blocks;
    } cases[] = {
        {SDCARD_V1, 1024, 1}, {SDCARD_V2, 1536, 3},
        {SDCARD_V2HC, UINT64_C(0x100000200), 1},
        {SDCARD_V2HC, UINT64_C(0x100000200), 3},
        {SDCARD_V2HC, (UINT64_C(1) << 41) - 512, 1},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_card();
        config.card_type = cases[i].type;
        config.enable_crc = crc_enabled;
        if (config.card_type == SDCARD_V2HC) {
            config.total_sectors = UINT64_C(1) << 32;
        }
        uint32_t argument = config.card_type == SDCARD_V2HC ?
                            cases[i].address / 512 : cases[i].address;
        uint8_t expected[1536], buffer[1536] = {0};
        size_t length = cases[i].blocks * 512;
        fill_pattern(expected, length);
        if (writing) {
            memcpy(buffer, expected, length);
            if (cases[i].blocks > 1) {
                expect_preerase(cases[i].blocks, 0);
            }
            expect_select();
            expect_spi(0xFF, 0xFF, false, 1);
            expect_command(cases[i].blocks > 1 ? CMD25_WRITE_MULTIPLE_BLOCK : CMD24_WRITE_BLOCK,
                           argument);
            expect_spi(0xFF, 0, false, 1);
        } else {
            expect_start(cases[i].blocks, argument, 0);
        }
        for (size_t block = 0; block < cases[i].blocks; ++block) {
            if (writing) {
                expect_write_payload(expected + block * 512, 512,
                                     cases[i].blocks > 1 ? SPI_START_BLK_MUL_WRITE : SPI_START_BLOCK);
                expect_spi(0xFF, 0xE5, false, 1);
                expect_spi(0xFF, 0xFF, false, 1);
            } else {
                expect_read_payload(expected + block * 512, 512, !crc_enabled);
            }
        }
        if (cases[i].blocks > 1 && !writing) {
            expect_stop(0, 0, 0, true);
        } else {
            if (cases[i].blocks > 1) {
                expect_spi(SPI_STOP_TRAN, 0xFF, false, 1);
                expect_spi(0xFF, 0xFF, false, 2);
            }
            expect_release();
        }
        int status = writing ? program(&device, buffer, cases[i].address, length) :
                               read(&device, buffer, cases[i].address, length);
        finish(status, BD_ERROR_OK);
        CHECK(memcmp(buffer, expected, length) == 0);
    }
}

// These parameter errors and pre-transfer rejection are API policy; SD does not
// define C null pointers, uint64_t overflow, or zero-length blockdevice requests.
static void test_invalid_io(void) {
    static const struct {
        uint64_t address;
        uint64_t length;
    } cases[] = {
        {0, 0}, {1, 512}, {0, 511}, {4096 * 512, 512},
        {4096 * 512 - 512, 1024}, {UINT64_MAX - 511, 512},
        {512, UINT64_MAX - 511},
    };
    uint8_t buffer[512];
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_card();
        finish(read(&device, buffer, cases[i].address, cases[i].length),
               SD_BLOCK_DEVICE_ERROR_PARAMETER);
        finish(program(&device, buffer, cases[i].address, cases[i].length),
               SD_BLOCK_DEVICE_ERROR_PARAMETER);
        finish(trim(&device, cases[i].address, cases[i].length),
               SD_BLOCK_DEVICE_ERROR_PARAMETER);
    }
    reset_card();
    finish(read(&device, NULL, 0, 512), SD_BLOCK_DEVICE_ERROR_PARAMETER);
    finish(program(&device, NULL, 0, 512), SD_BLOCK_DEVICE_ERROR_PARAMETER);
    config.erase_size = 1024;
    finish(trim(&device, 512, 1024), SD_BLOCK_DEVICE_ERROR_PARAMETER);
    finish(trim(&device, 0, 512), SD_BLOCK_DEVICE_ERROR_PARAMETER);
}

// Software initialization/deinitialization and NO_INIT are API policy, distinct
// from card idle/reset state (SD v9.10, section 7.2.7, p.292).
static void test_uninitialized_io(bool after_deinit) {
    reset_card();
    if (after_deinit) {
        finish(deinit(&device), BD_ERROR_OK);
    } else {
        config.is_initialized = device.is_initialized = false;
        config.erase_size = 0;
    }
    CHECK(!config.is_initialized && !device.is_initialized);
    uint8_t buffer[512];
    finish(read(&device, buffer, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_INIT);
    finish(program(&device, buffer, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_INIT);
    finish(trim(&device, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_INIT);
}

// CMD55 is prerequisite to every ACMD (SD v9.10, Table 7-4, p.300).
// R1 error meanings: section 7.3.2.1, p.302. Never issuing a bare application
// opcode after an unconfirmed prefix is the driver's fail-closed policy.
static void test_app_prefix_failure(unsigned shift) {
    static const struct {
        uint8_t response;
        int status;
    } cases[] = {
        {0x08, SD_BLOCK_DEVICE_ERROR_CRC},
        {0x04, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED},
        {0x40, SD_BLOCK_DEVICE_ERROR_PARAMETER},
        {0xFF, SD_BLOCK_DEVICE_ERROR_NO_DEVICE},
    };
    static const int commands[] = {ACMD13_SD_STATUS, ACMD23_SET_WR_BLK_ERASE_COUNT,
                                   ACMD41_SD_SEND_OP_COND};
    for (size_t cmd = 0; cmd < sizeof(commands) / sizeof(commands[0]); ++cmd) {
        for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
            reset_card();
            expect_select();
            expect_spi(0xFF, 0xFF, false, 1);
            for (unsigned attempt = 0; attempt < (cases[i].response == 0xFF ? 3u : 1u); ++attempt) {
                expect_command(CMD55_APP_CMD, 0);
                if (cases[i].response == 0xFF) {
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
}

// Re-establish the CMD55 prerequisite after no response; the retry count is
// host policy (SD v9.10, Table 7-4, p.300).
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
    finish(_cmd(&config, ACMD41_SD_SEND_OP_COND, 0, true, NULL), BD_ERROR_OK);
}

// Commands can be rejected (SD v9.10, section 7.2.8, p.292); data phases follow
// valid read/write commands (sections 7.2.3-7.2.4, pp.289-290). One release
// and no follow-up after rejection are driver cleanup policy.
static void test_transfer_rejected(void) {
    static const int commands[] = {CMD17_READ_SINGLE_BLOCK, CMD18_READ_MULTIPLE_BLOCK,
                                   CMD24_WRITE_BLOCK, CMD25_WRITE_MULTIPLE_BLOCK,
                                   CMD32_ERASE_WR_BLK_START_ADDR, CMD33_ERASE_WR_BLK_END_ADDR};
    uint8_t buffer[1024] = {0};
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        reset_card();
        if (commands[i] == CMD25_WRITE_MULTIPLE_BLOCK) {
            expect_preerase(2, 0);
        }
        if (commands[i] == CMD33_ERASE_WR_BLK_END_ADDR) {
            expect_init_command(CMD32_ERASE_WR_BLK_START_ADDR, 1024, 0, 0);
        }
        uint32_t address = commands[i] == CMD33_ERASE_WR_BLK_END_ADDR ? 1536 : 1024;
        expect_init_command(commands[i], address, R1_ILLEGAL_COMMAND, 0);
        int status;
        if (commands[i] == CMD17_READ_SINGLE_BLOCK || commands[i] == CMD18_READ_MULTIPLE_BLOCK) {
            status = read(&device, buffer, 1024,
                          commands[i] == CMD17_READ_SINGLE_BLOCK ? 512 : 1024);
        } else if (commands[i] == CMD24_WRITE_BLOCK || commands[i] == CMD25_WRITE_MULTIPLE_BLOCK) {
            status = program(&device, buffer, 1024, commands[i] == CMD24_WRITE_BLOCK ? 512 : 1024);
        } else {
            status = trim(&device, 1024, 1024);
        }
        finish(status, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED);
    }
}

// Using pre-erase is optional for the host, but successful CMD25 still needs
// Stop Tran with or without ACMD23 (SD v9.10, Table 7-4, footnote 2, p.300).
// Continuing after rejected pre-erase is driver policy, not required by SD.
static void test_preerase_rejected(void) {
    reset_card();
    expect_preerase(2, R1_ILLEGAL_COMMAND);
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD25_WRITE_MULTIPLE_BLOCK, 1024);
    expect_spi(0xFF, 0, false, 1);
    expect_program_block(0x31, SPI_START_BLK_MUL_WRITE, SPI_DATA_ACCEPTED, 0, false);
    expect_program_block(0x31, SPI_START_BLK_MUL_WRITE, SPI_DATA_ACCEPTED, 0, false);
    expect_spi(SPI_STOP_TRAN, 0xFF, false, 1);
    expect_spi(0xFF, 0xFF, false, 2);
    expect_release();
    uint8_t buffer[1024];
    memset(buffer, 0x31, sizeof(buffer));
    finish(program(&device, buffer, 1024, sizeof(buffer)), BD_ERROR_OK);
}

// CMD32/CMD33 endpoints are inclusive (SD v9.10, Table 7-3, p.298);
// one-block trim has equal endpoints. The 250 ms/block estimate comes from
// section 4.6.2.3, p.96; retaining the command-timeout floor is host policy.
static void test_trim_boundaries(bool high_capacity) {
    for (unsigned wide = 0; wide < (high_capacity ? 2u : 1u); ++wide) {
        reset_card();
        config.card_type = high_capacity ? SDCARD_V2HC : SDCARD_V1;
        config.total_sectors = high_capacity ? UINT64_C(1) << 32 : 4096;
        uint64_t address = wide ? UINT64_C(0x100000200) : 1024;
        uint32_t argument = high_capacity ? address / 512 : address;
        expect_init_command(CMD32_ERASE_WR_BLK_START_ADDR, argument, 0, 0);
        expect_init_command(CMD33_ERASE_WR_BLK_END_ADDR, argument, 0, 0);
        expect_select();
        expect_spi(0xFF, 0xFF, false, 1);
        expect_command(CMD38_ERASE, 0);
        expect_spi(0xFF, 0, false, 1);
        uint64_t timeout = SD_COMMAND_TIMEOUT > 250 ? SD_COMMAND_TIMEOUT : 250;
        expect_spi(0xFF, 0, false, timeout);
        expect_release();
        finish(trim(&device, address, 512), SD_BLOCK_DEVICE_ERROR_NO_RESPONSE);
    }
}

// Legacy/V2 detection, HCS, ACMD41 polling, and post-init CCS determine these
// sequences and card types (SD v9.10, section 7.2.1, Figure 7-2, pp.287-289).
static void expect_card_init(uint8_t card_type, bool polling) {
    bool v2 = card_type != SDCARD_V1;
    expect_spi(0xFF, 0xFF, true, 10);
    expect_init_command(CMD0_GO_IDLE_STATE, 0, 1, 0);
    expect_init_command(CMD8_SEND_IF_COND, 0x1AA, v2 ? 1 : 0x05, 0x1AA);
    if (config.enable_crc) {
        expect_init_command(CMD59_CRC_ON_OFF, 1, 1, 0);
    }
    expect_init_command(CMD58_READ_OCR, 0, 1, OCR_3_3V);
    if (polling) {
        expect_init_command(ACMD41_SD_SEND_OP_COND, v2 ? OCR_HCS_CCS : 0, 1, 0);
    }
    expect_init_command(ACMD41_SD_SEND_OP_COND, v2 ? OCR_HCS_CCS : 0, 0, 0);
    if (v2) {
        expect_init_command(CMD58_READ_OCR, 0, 0,
                            OCR_3_3V | (card_type == SDCARD_V2HC ? OCR_HCS_CCS : 0));
    }
    if (!config.enable_crc) {
        expect_init_command(CMD59_CRC_ON_OFF, 0, 0, 0);
    }
}

static void test_init_types(void) {
    for (uint8_t type = SDCARD_V1; type <= SDCARD_V2HC; ++type) {
        for (unsigned crc = 0; crc < 2; ++crc) {
            reset_card();
            config.card_type = SDCARD_NONE;
            config.enable_crc = crc;
            expect_card_init(type, true);
            finish(init_card(&config), BD_ERROR_OK);
            CHECK(config.card_type == type);
        }
    }
}

// Reject voltage/pattern mismatches and do not infer capacity without final
// CMD58 (SD v9.10, section 7.2.1, pp.287-289). Deadline/error mapping and
// preserving the first failure instead of overwriting it are host policy.
static void test_init_failures(void) {
    static const struct {
        uint32_t r7;
        uint32_t ocr;
        uint8_t acmd41;
        uint8_t final_ocr;
        int status;
    } cases[] = {
        {0x1AB, OCR_3_3V, 0, 0, SD_BLOCK_DEVICE_ERROR_UNUSABLE},
        {0x2AA, OCR_3_3V, 0, 0, SD_BLOCK_DEVICE_ERROR_UNUSABLE},
        {0x1AA, 0, 0, 0, SD_BLOCK_DEVICE_ERROR_UNUSABLE},
        {0x1AA, OCR_3_3V, 1, 0, SD_BLOCK_DEVICE_ERROR_NO_RESPONSE},
        {0x1AA, OCR_3_3V, 0x09, 0, SD_BLOCK_DEVICE_ERROR_CRC},
        {0x1AA, OCR_3_3V, 0, 0x04, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_card();
        config.enable_crc = false;
        if (cases[i].acmd41 == 1) {
            time_step_us = (uint64_t)SD_COMMAND_TIMEOUT * 1000;
        }
        expect_spi(0xFF, 0xFF, true, 10);
        expect_init_command(CMD0_GO_IDLE_STATE, 0, 1, 0);
        expect_init_command(CMD8_SEND_IF_COND, 0x1AA, 1, cases[i].r7);
        if (cases[i].r7 == 0x1AA) {
            expect_init_command(CMD58_READ_OCR, 0, 1, cases[i].ocr);
            if (cases[i].ocr & OCR_3_3V) {
                expect_init_command(ACMD41_SD_SEND_OP_COND, OCR_HCS_CCS, cases[i].acmd41, 0);
                if (!cases[i].acmd41) {
                    expect_init_command(CMD58_READ_OCR, 0, cases[i].final_ocr, 0);
                }
            }
        }
        finish(init_card(&config), cases[i].status);
        CHECK(config.card_type == CARD_UNKNOWN);
    }
}

// CMD0 enters SPI idle (SD v9.10, section 7.2.1, p.287).
// Retry count and sleeps are driver policy, not a prescribed reset sequence.
static void test_idle_retries(bool exhausted) {
    reset_card();
    for (unsigned i = 0; i < (exhausted ? SD_CMD0_GO_IDLE_STATE_RETRIES : 2u); ++i) {
        expect_init_command(CMD0_GO_IDLE_STATE, 0, !exhausted && i == 1 ? 1 : 0, 0);
    }
    finish(_go_idle_state(&config), exhausted ? 0 : R1_IDLE_STATE);
    CHECK(now_us >= (exhausted ? SD_CMD0_GO_IDLE_STATE_RETRIES : 1u) * 1000);
}

static void set_csd_bits(uint8_t *csd, unsigned msb, unsigned lsb, uint32_t value) {
    for (unsigned bit = lsb; bit <= msb; ++bit) {
        uint8_t mask = 1u << (bit % 8);
        csd[15 - bit / 8] = (csd[15 - bit / 8] & ~mask) |
                           ((value & (1u << (bit - lsb))) ? mask : 0);
    }
}

// Synthetic fixtures isolate CSD fields, not complete valid card registers.
// Layout/formulas: SD v9.10, sections 5.3.2-5.3.3, pp.226-235.
static void make_csd(uint8_t *csd, bool high_capacity) {
    memset(csd, 0, 16);
    set_csd_bits(csd, 127, 126, high_capacity ? 1 : 0);
    if (high_capacity) {
        // Stress the full 22-bit encoding, beyond the specified SDXC maximum
        // (SD v9.10, section 5.3.3, pp.234-235), to catch arithmetic wrap.
        set_csd_bits(csd, 69, 48, 0x3FFFFF);
    } else {
        set_csd_bits(csd, 83, 80, 9);
        set_csd_bits(csd, 73, 62, 1023);
        set_csd_bits(csd, 49, 47, 0);
        set_csd_bits(csd, 45, 39, 3);
    }
}

static void expect_csd(const uint8_t *csd, bool corrupt_crc) {
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD9_SEND_CSD, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_read_payload(csd, 16, corrupt_crc);
    expect_release();
}

// CSD v1 capacity and erase units: SD v9.10, section 5.3.2, pp.229-231.
// CSD v2 capacity and fixed ERASE_BLK_EN: section 5.3.3, pp.234-235.
// CMD9 transfers a 16-byte block with CRC16 (section 7.2.6, p.292).
static void test_sectors(void) {
    for (unsigned kind = 0; kind < 6; ++kind) {
        reset_card();
        uint8_t csd[16];
        make_csd(csd, kind == 1);
        if (kind == 2) {
            set_csd_bits(csd, 46, 46, 1);
        }
        if (kind == 3) {
            set_csd_bits(csd, 127, 126, 2);
        }
        if (kind == 5) {
            expect_init_command(CMD9_SEND_CSD, 0, R1_ILLEGAL_COMMAND, 0);
        } else {
            expect_csd(csd, kind == 4);
        }
        uint64_t sectors = _sd_sectors(&config);
        uint64_t expected = kind == 1 ? UINT64_C(1) << 32 : kind >= 3 ? 0 : 4096;
        CHECK(sectors == expected);
        if (expected) {
            CHECK(config.erase_size == (kind == 0 ? 2048u : 512u));
        }
        finish(BD_ERROR_OK, BD_ERROR_OK);
    }
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

// Publishing derived geometry and setting both initialization flags only after
// all setup succeeds are API policy. Erase granularity comes from CSD
// ERASE_BLK_EN/SECTOR_SIZE (SD v9.10, section 5.3.2, pp.230-231).
static void test_top_init(unsigned failure) {
    reset_card();
    if (failure == 4) {
        expect_spi(0xFF, 0xFF, true, 10);
        for (unsigned i = 0; i < SD_CMD0_GO_IDLE_STATE_RETRIES; ++i) {
            expect_init_command(CMD0_GO_IDLE_STATE, 0, 0, 0);
        }
    } else {
        expect_card_init(SDCARD_V1, false);
        uint8_t csd[16];
        make_csd(csd, false);
        expect_csd(csd, failure == 1);
        if (failure != 1) {
            expect_init_command(CMD16_SET_BLOCKLEN, 512, failure == 2 ? R1_ILLEGAL_COMMAND : 0, 0);
        }
    }
    if (failure == 3) {
        config.hz = 30000000;
    }
    int status = failure == 4 ? SD_BLOCK_DEVICE_ERROR_NO_DEVICE :
                 failure == 1 || failure == 2 ? BD_ERROR_DEVICE_ERROR :
                 failure == 3 ? -EINVAL : BD_ERROR_OK;
    finish(init(&device), status);
    CHECK(device.is_initialized == (failure == 0));
    CHECK(config.is_initialized == (failure == 0));
    if (!failure) {
        CHECK(device.erase_size == 2048 && config.erase_size == 2048);
        CHECK(device.size(&device) == 4096 * 512);
        CHECK(transfer_baudrate == 12000000);
        finish(trim(&device, 512, 512), SD_BLOCK_DEVICE_ERROR_PARAMETER);
    } else {
        uint8_t buffer[512];
        finish(read(&device, buffer, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_INIT);
        finish(program(&device, buffer, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_INIT);
        finish(trim(&device, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_NO_INIT);
    }
}

// Default-speed TRAN_SPEED is 25 MHz; 50 MHz needs High-Speed mode
// (SD v9.10, section 5.3.2, p.227). Clamp/error behavior is driver policy.
static void test_frequency(void) {
    static const uint32_t frequencies[] = {12000000, 25000000, 30000000};
    for (size_t i = 0; i < sizeof(frequencies) / sizeof(frequencies[0]); ++i) {
        reset_card();
        config.hz = frequencies[i];
        finish(_freq(&config), i == 2 ? -EINVAL : BD_ERROR_OK);
        CHECK(transfer_baudrate == (i == 2 ? 25000000 : frequencies[i]));
        CHECK(config.hz == transfer_baudrate);
    }
}

// SPI1 RX includes GPIO24 and GPIO28 (RP2040 datasheet, section 1.4.3,
// Table 2, pp.12-13); SD does not specify the host's GPIO routing.
static void test_create(unsigned miso) {
    reset_card();
    config.spi_inst = spi1;
    expect_card_init(SDCARD_V2HC, false);
    uint8_t csd[16];
    make_csd(csd, true);
    expect_csd(csd, false);
    expect_init_command(CMD16_SET_BLOCKLEN, 512, 0, 0);
    blockdevice_t *created = blockdevice_sd_create(spi1, 27, miso, 26, 17, 12000000, true);
    CHECK(created != NULL);
    blockdevice_sd_config_t *created_config = created->config;
    CHECK(created->is_initialized && created_config->is_initialized);
    CHECK(created_config->spi_inst == spi1 && created_config->miso == miso);
    CHECK(created->size(created) == (UINT64_C(1) << 41));
    CHECK(created->erase_size == created_config->erase_size);
    CHECK(transfer_baudrate == 12000000 && format_calls == 1);
    CHECK(!created_config->_mutex.locked);
    finish(created->deinit(created), BD_ERROR_OK);
    CHECK(!created_config->is_initialized && !created->is_initialized);
    blockdevice_sd_free(created);
}

int main(void) {
    RUN_CASE("invalid-io", test_invalid_io());
    RUN_CASE("uninitialized-io", test_uninitialized_io(false));
    RUN_CASE("deinitialized-io", test_uninitialized_io(true));
    RUN_CASE("init-types", test_init_types());
    RUN_CASE("init-failures", test_init_failures());
    RUN_CASE("idle-retries", test_idle_retries(false));
    RUN_CASE("idle-exhausted", test_idle_retries(true));
    RUN_CASE("sectors", test_sectors());
    RUN_CASE("top-init", test_top_init(0));
    RUN_CASE("top-init-csd-error", test_top_init(1));
    RUN_CASE("top-init-cmd16-error", test_top_init(2));
    RUN_CASE("top-init-frequency-error", test_top_init(3));
    RUN_CASE("top-init-card-error", test_top_init(4));
    RUN_CASE("frequency", test_frequency());
    RUN_CASE("create-spi1-miso24", test_create(24));
    RUN_CASE("create-spi1-miso28", test_create(28));
    RUN_CASE("patterned-read", test_patterned_io(false, true));
    RUN_CASE("patterned-read-crc-disabled", test_patterned_io(false, false));
    RUN_CASE("patterned-program", test_patterned_io(true, true));
    RUN_CASE("patterned-program-crc-disabled", test_patterned_io(true, false));
    RUN_CASE("transfer-rejected", test_transfer_rejected());
    RUN_CASE("preerase-rejected", test_preerase_rejected());
    RUN_CASE("trim-sdsc-boundaries", test_trim_boundaries(false));
    RUN_CASE("trim-sdhc-boundaries", test_trim_boundaries(true));
    RUN_CASE("init-crc-enabled", test_init_crc(true, false));
    RUN_CASE("init-crc-disabled", test_init_crc(false, false));
    RUN_CASE("init-crc-rejected", test_init_crc(true, true));
    RUN_CASE("singleblock", test_singleblock());
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
        snprintf(name, sizeof(name), "write-alignment-%u", shift);
        RUN_CASE(name, test_write_alignment(shift, true));
        snprintf(name, sizeof(name), "write-alignment-crc-disabled-%u", shift);
        RUN_CASE(name, test_write_alignment(shift, false));
        snprintf(name, sizeof(name), "write-busy-timeout-%u", shift);
        RUN_CASE(name, test_write_busy_timeout(shift));
        snprintf(name, sizeof(name), "multiblock-program-%u", shift);
        RUN_CASE(name, test_multiblock_program(shift, false, true));
        snprintf(name, sizeof(name), "multiblock-program-crc-disabled-%u", shift);
        RUN_CASE(name, test_multiblock_program(shift, false, false));
        snprintf(name, sizeof(name), "multiblock-program-stop-timeout-%u", shift);
        RUN_CASE(name, test_multiblock_program(shift, true, true));
        snprintf(name, sizeof(name), "multiblock-program-failure-%u", shift);
        RUN_CASE(name, test_multiblock_program_failure(shift));
    }
    RUN_CASE("write-response-bounds", test_write_response_bounds());
    RUN_CASE("write-response-windows", test_write_response_windows());
    RUN_CASE("multiblock-stop-timing", test_multiblock_stop_timing());
    RUN_CASE("singleblock-response-failure", test_program_response_failure(false));
    RUN_CASE("multiblock-response-failure", test_program_response_failure(true));
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
    RUN_CASE("idle-cmd9", test_idle_command(9, false, 1, SD_BLOCK_DEVICE_ERROR_NO_INIT));
    RUN_CASE("idle-cmd17", test_idle_command(17, false, 1, SD_BLOCK_DEVICE_ERROR_NO_INIT));
    RUN_CASE("idle-cmd18", test_idle_command(18, false, 1, SD_BLOCK_DEVICE_ERROR_NO_INIT));
    RUN_CASE("idle-cmd24", test_idle_command(24, false, 1, SD_BLOCK_DEVICE_ERROR_NO_INIT));
    RUN_CASE("idle-cmd25", test_idle_command(25, false, 1, SD_BLOCK_DEVICE_ERROR_NO_INIT));
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
    return EXIT_SUCCESS;
}
