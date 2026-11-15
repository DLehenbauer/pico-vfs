#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONF_SD_CMD_TIMEOUT 5
#include "../../src/blockdevice/sd.c"

static const char *current_case;

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

void gpio_put(unsigned gpio, bool value) {
    CHECK(gpio == config.cs);
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
    expect_spi(0xFF, 0xFF, false, 8);
    expect_cs(true);
    expect_spi(0xFF, 0xFF, true, 1);
}

static void expect_command(int cmd, uint32_t arg) {
    uint8_t packet[6] = {
        (uint8_t)(0x40 | cmd), arg >> 24, arg >> 16, arg >> 8, arg, 0xFF,
    };
    if (config.enable_crc) {
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
static void expect_stop(uint8_t response, size_t busy_bytes, size_t response_delay) {
    expect_command(12, 0);
    expect_spi(0xFF, 0x04, false, 1);
    if (response_delay) {
        expect_spi(0xFF, 0xFF, false, response_delay);
    }
    expect_spi(0xFF, response, false, 1);
    // CRC/illegal-command errors return before the existing R1b wait.
    if (!(response & (0x04 | 0x08))) {
        if (busy_bytes) {
            expect_spi(0xFF, 0, false, busy_bytes);
        }
        expect_spi(0xFF, 0xFF, false, 1);
    }
    expect_release();
}

static void finish(int actual, int expected) {
    CHECK(actual == expected);
    CHECK(event_index == event_count);
    CHECK(cs_high);
    CHECK(!config._mutex.locked);
}

// Checks CMD17 at SDSC byte address 1024, command CRC7, start token, all 512 data
// bytes, CRC16, returned data, CS cleanup, and mutex release.
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
static void test_stop_status(uint8_t response, int expected, size_t response_delay) {
    reset_card();
    expect_start(2, 0, 0);
    expect_block(0x31);
    expect_block(0xA6);
    expect_stop(response, 0, response_delay);
    uint8_t buffer[1024];
    finish(device.read(&device, buffer, 0, sizeof(buffer)), expected);
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
    expect_stop(0, 3, 2);
    uint8_t buffer[1024] = {0};
    uint8_t expected[1024];
    memset(expected, 0x31, 512);
    memset(expected + 512, 0xA6, 512);
    finish(device.read(&device, buffer, 1024, sizeof(buffer)), BD_ERROR_OK);
    CHECK(memcmp(buffer, expected, sizeof(buffer)) == 0);
}

static void test_multiblock_data_failure(bool crc_failure, uint8_t stop_response, int expected) {
    reset_card();
    expect_start(3, 0, 0);
    expect_block(0x31);
    if (crc_failure) {
        expect_block_crc(0xA6, true);
    } else {
        expect_spi(0xFF, 0xFF, false, 300);
    }
    expect_stop(stop_response, 0, 0);
    uint8_t buffer[1536];
    // Preserve the deferred behavior: the stop result replaces the read error.
    finish(device.read(&device, buffer, 0, sizeof(buffer)), expected);
}

static void test_repeated_read(void) {
    reset_card();
    expect_start(2, 0, 0);
    expect_block(0x31);
    expect_block(0xA6);
    expect_stop(0, 3, 2);
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

int main(void) {
    RUN_CASE("singleblock", test_singleblock());
    RUN_CASE("start-rejected", test_start_rejected());
    RUN_CASE("token-busy-wrap", test_token_wrap(false));
    RUN_CASE("token-timeout-wrap", test_token_wrap(true));
    RUN_CASE("wide-busy-timeout", test_wide_busy_timeout());
    RUN_CASE("start-invalid-response", test_no_response(false, 0x80));
    RUN_CASE("stop-invalid-response", test_no_response(true, 0x80));
    RUN_CASE("stop-no-response", test_no_response(true, 0xFF));
    RUN_CASE("stop-first-response", test_stop_status(0, BD_ERROR_OK, 0));
    RUN_CASE("stop-illegal", test_stop_status(0x04, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED, 2));
    RUN_CASE("command-busy-timeout", test_command_busy(false));
    RUN_CASE("app-command-busy-timeout", test_command_busy(true));
    RUN_CASE("multiblock-sdsc", test_multiblock(false, true));
    RUN_CASE("multiblock-sdhc", test_multiblock(true, true));
    RUN_CASE("multiblock-token-timeout", test_multiblock_data_failure(false, 0, BD_ERROR_OK));
    RUN_CASE("multiblock-token-timeout-stop-error",
             test_multiblock_data_failure(false, 0x04, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED));
    RUN_CASE("multiblock-data-crc", test_multiblock_data_failure(true, 0, BD_ERROR_OK));
    RUN_CASE("multiblock-data-crc-stop-error",
             test_multiblock_data_failure(true, 0x04, SD_BLOCK_DEVICE_ERROR_UNSUPPORTED));
    RUN_CASE("crc-disabled", test_multiblock(false, false));
    RUN_CASE("repeated-read", test_repeated_read());
    RUN_CASE("stop-crc", test_stop_status(0x08, SD_BLOCK_DEVICE_ERROR_CRC, 2));
    RUN_CASE("stop-parameter", test_stop_status(0x40, SD_BLOCK_DEVICE_ERROR_PARAMETER, 2));
    RUN_CASE("stop-erase", test_stop_status(0x10, SD_BLOCK_DEVICE_ERROR_ERASE, 2));
    RUN_CASE("stop-no-busy", test_stop_status(0, BD_ERROR_OK, 2));
    // Seven delayed bytes plus the stuff byte exercise eight filler bytes.
    RUN_CASE("stop-max-response-delay", test_stop_status(0, BD_ERROR_OK, 7));
    return EXIT_SUCCESS;
}
