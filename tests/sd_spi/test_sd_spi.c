#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONF_SD_CMD_TIMEOUT 5
#include "../../src/blockdevice/sd.c"

static const char *current_case;
static unsigned initialized_baudrate;

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
    CHECK(spi == spi0);
    initialized_baudrate = baudrate;
    return baudrate;
}

int spi_write_read_blocking(spi_inst_t *spi, const uint8_t *src, uint8_t *dst, size_t len) {
    CHECK(spi == spi0);
    for (size_t i = 0; i < len; ++i) {
        uint8_t rx = exchange(src[i]);
        if (dst) {
            dst[i] = rx;
        }
    }
    return (int)len;
}

int spi_read_blocking(spi_inst_t *spi, uint8_t repeated_tx_data, uint8_t *dst, size_t len) {
    CHECK(spi == spi0);
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
    config = (blockdevice_sd_config_t){
        .spi_inst = spi0,
        .cs = 17,
        .enable_crc = true,
        .card_type = SDCARD_V1,
        .is_initialized = true,
        .block_size = 512,
        .erase_size = 512,
        .total_sectors = 4096,
    };
    device = (blockdevice_t){
        .config = &config,
        .read = read,
        .size = size,
        .read_size = 512,
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

static void expect_select(void) {
    expect_spi(0xFF, 0xFF, true, 1);
    expect_cs(false);
}

static void expect_release(void) {
    // Eight selected bytes conservatively cover N_CR's 1-8 byte-times, not a
    // mandatory fixed delay; N_EC has a zero minimum (SD v2.00, section 7.5,
    // p.149, and Table 7-7, p.153).
    // One deselected byte covers the one-clock DataOut release during R1b busy.
    // Eight deselected clocks are not a required minimum (SD v2.00,
    // section 7.5.1.2, p.150).
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

// CMD12 stays in the selected read stream (SD v2.00, Figure 7-18, p.151).
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
    if (cmd == CMD8_SEND_IF_COND || cmd == CMD58_READ_OCR) {
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

// Checks CMD17 at SDSC byte address 1024, command CRC7, start token, all 512 data
// bytes, CRC16, returned data, CS cleanup, and mutex release.
// Read blocks include the token and CRC bytes (SD v2.00, section 7.3.3.2, p.146).
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
// R1 status reference: SD Physical Layer Specification v2.00 (May 9, 2006),
// section 7.3.2.1.
//
// Error mapping and CS/mutex cleanup are driver behavior, not SD requirements.
static void test_start_rejected(void) {
    reset_card();
    expect_start(2, 0, 0x04);
    expect_release();
    uint8_t buffer[1024];
    finish(device.read(&device, buffer, 0, sizeof(buffer)), SD_BLOCK_DEVICE_ERROR_UNSUPPORTED);
}

// R1's bit 7 must be clear (SD v2.00, section 7.3.2.1, p.143).
// Exhausting polls with 0x80 or 0xFF is no response, not command success.
// The existing 16-byte guard and three retries are host tolerance, not N_CR.
// N_CR is at most eight byte-times (SD v2.00, Table 7-7, p.153).
static void test_no_response(bool stop, uint8_t invalid_byte) {
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
        expect_spi(0xFF, invalid_byte, false, 16);
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

// A valid CMD55 prefix is required before every application command. Fail
// closed when the prefix is unanswered or reports an R1 error.
static void test_app_prefix_failure(void) {
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
                    expect_spi(0xFF, cases[i].response, false, 1);
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

// Reissue CMD55 after an unanswered prefix, then send the application opcode.
static void test_app_prefix_retry(void) {
    reset_card();
    expect_select();
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(CMD55_APP_CMD, 0);
    expect_spi(0xFF, 0xFF, false, 16);
    expect_command(CMD55_APP_CMD, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_spi(0xFF, 0xFF, false, 1);
    expect_command(ACMD41_SD_SEND_OP_COND, 0);
    expect_spi(0xFF, 0, false, 1);
    expect_release();

    uint32_t response = R1_NO_RESPONSE;
    finish(_cmd(&config, ACMD41_SD_SEND_OP_COND, 0, true, &response), BD_ERROR_OK);
    CHECK(response == 0);
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

// Commands are rejected during busy (SD v2.00, section 7.2.8, p.133).
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

// No extra clocks, ready polling, or CS edges are allowed from the final CRC
// through CMD12 (SD v2.00, section 7.5.2.2, Figure 7-18, p.151).
// SDSC byte addressing and SDHC block addressing differ (SD v2.00, Table 7-3,
// footnote 10, p.140). Busy is optional (Table 7-3, footnote 5, p.140).
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

// Read blocks carry CRC16 and CMD12 terminates CMD18 (SD v2.00, section 7.2.3,
// pp.130-131). Preferring the data error over cleanup failure is API policy.
// One block succeeds before failure. The unread third block must not be consumed.
// The 300 ms token and 5 s default production command budgets are host policy,
// not comprehensive timing compliance: SDSC timing is card-dependent and SDHC
// has a 100 ms read bound (SD v2.00, section 4.6.2, p.54, and Table 7-7, p.153).
// Waiting for 0xFE does not immediately decode read-error tokens (SD v2.00,
// section 7.3.3.3, p.147).
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
// (SD v2.00, section 7.3.2.2, p.143).
// A timeout leaves completion unconfirmed, but must not replace a more
// specific R1 error. The full-high check and timeout budget remain host policy.
// Waiting selected is synchronous-driver policy: deselection is permitted during
// programming and R1b busy (SD v2.00, section 7.2.4, p.132, and section 7.5.1.2,
// p.150).
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

// Exercise the actual SPI setup, not just the configured frequency constant.
// The compatibility range follows Elm-Chan SPI initialization guidance
// (See: https://elm-chan.org/docs/mmc/mmc_e.html#spiinit).
static void test_init_clock(void) {
    reset_card();
    initialized_baudrate = 0;
    expect_spi(0xFF, 0xFF, true, 10);
    _spi_init(&config);
    CHECK(initialized_baudrate >= 100000 && initialized_baudrate <= 400000);
    finish(BD_ERROR_OK, BD_ERROR_OK);
}

int main(void) {
    RUN_CASE("app-prefix-failure", test_app_prefix_failure());
    RUN_CASE("app-prefix-retry", test_app_prefix_retry());
    RUN_CASE("init-crc-enabled", test_init_crc(true, false));
    RUN_CASE("init-crc-disabled", test_init_crc(false, false));
    RUN_CASE("init-crc-rejected", test_init_crc(true, true));
    RUN_CASE("singleblock", test_singleblock());
    RUN_CASE("start-rejected", test_start_rejected());
    RUN_CASE("token-busy-wrap", test_token_wrap(false));
    RUN_CASE("token-timeout-wrap", test_token_wrap(true));
    RUN_CASE("wide-busy-timeout", test_wide_busy_timeout());
    RUN_CASE("power-up", test_power_up());
    RUN_CASE("start-invalid-response", test_no_response(false, 0x80));
    RUN_CASE("stop-unaligned", test_stop_unaligned(0x1F, BD_ERROR_OK));
    RUN_CASE("stop-unaligned-illegal", test_stop_unaligned(0x9F, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED));
    RUN_CASE("stop-no-response", test_no_response(true, 0xFF));
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
    return EXIT_SUCCESS;
}
