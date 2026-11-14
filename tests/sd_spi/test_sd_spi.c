#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONF_SD_CMD_TIMEOUT 5
#include "../../src/blockdevice/sd.c"

#define CHECK(condition)                                                                           \
    do {                                                                                           \
        if (!(condition)) {                                                                        \
            fprintf(stderr, "%s:%d: %s (event %zu)\n", __FILE__, __LINE__, #condition,             \
                    event_index);                                                                  \
            exit(EXIT_FAILURE);                                                                    \
        }                                                                                          \
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

static void expect_block(uint8_t value) {
    uint8_t data[512];
    memset(data, value, sizeof(data));
    uint16_t crc = reference_crc16(data, sizeof(data));
    expect_spi(0xFF, 0xFE, false, 1);
    expect_spi(0xFF, value, false, sizeof(data));
    expect_spi(0xFF, crc >> 8, false, 1);
    expect_spi(0xFF, crc, false, 1);
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

int main(int argc, char **argv) {
    CHECK(argc == 2);
    const char *name = argv[1];
    if (!strcmp(name, "singleblock")) {
        test_singleblock();
    } else if (!strcmp(name, "start-rejected")) {
        test_start_rejected();
    } else if (!strcmp(name, "token-busy-wrap")) {
        test_token_wrap(false);
    } else if (!strcmp(name, "token-timeout-wrap")) {
        test_token_wrap(true);
    } else if (!strcmp(name, "wide-busy-timeout")) {
        test_wide_busy_timeout();
    } else {
        CHECK(false);
    }
    printf("%s: passed\n", name);
    return EXIT_SUCCESS;
}
