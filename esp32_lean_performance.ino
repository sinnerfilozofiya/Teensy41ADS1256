#include <Arduino.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include "common_frame.h"

// ============================================================================
// LEAN MAXIMUM PERFORMANCE CONFIGURATION
// ============================================================================

#define PIN_MOSI 39
#define PIN_MISO 40
#define PIN_SCLK 38
#define PIN_CS   41

#define FRAME_BYTES 144
#define QUEUED_XFERS 8
#define SAMPLES_PER_FRAME 10

#define ESPNOW_CHANNEL 1
static const uint8_t ESPNOW_PEER_MAC[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x01};
#define USE_CUSTOM_MAC 1
static const uint8_t CUSTOM_STA_MAC_DUAL[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x03};

// ============================================================================
// OUTPUT MODES
// ============================================================================

enum OutputMode {
    MODE_COMPACT_TEXT = 0,    // L:frame:sample:lc1:lc2:lc3:lc4
    MODE_BINARY = 1,          // Binary struct
    MODE_COMBINED_TEXT = 2,   // C:frame:sample:local_lc1:...:remote_lc4
    MODE_COMBINED_BINARY = 3  // Combined binary for wireless
};

static OutputMode current_output_mode = MODE_COMPACT_TEXT;
static bool output_local_data = true;
static bool output_remote_data = true;

// ============================================================================
// BINARY STRUCTURES
// ============================================================================

struct __attribute__((packed)) BinarySample {
    uint8_t type;           // 'L' or 'R'
    uint16_t frame_idx;
    uint8_t sample_idx;
    int32_t lc[4];
};

struct __attribute__((packed)) CombinedSample {
    uint16_t frame_idx;
    uint8_t sample_idx;
    int32_t local_lc[4];
    int32_t remote_lc[4];
    uint8_t flags;          // bit0=has_local, bit1=has_remote
};

// ============================================================================
// LATEST DATA SYNCHRONIZATION
// ============================================================================

struct LatestData {
    int32_t lc[4];
    uint16_t frame_idx;
    uint8_t sample_idx;
    bool valid;
};

static LatestData latest_local = {0};
static LatestData latest_remote = {0};

// SPI buffers
static uint8_t spi_rxbuf[QUEUED_XFERS][FRAME_BYTES] __attribute__((aligned(4)));
static spi_slave_transaction_t spi_trx[QUEUED_XFERS];

// ============================================================================
// ULTRA-FAST DATA PROCESSING
// ============================================================================

static inline void update_latest_local(uint16_t frame_idx, uint8_t sample_idx, 
                                      int32_t lc1, int32_t lc2, int32_t lc3, int32_t lc4) {
    latest_local.lc[0] = lc1;
    latest_local.lc[1] = lc2;
    latest_local.lc[2] = lc3;
    latest_local.lc[3] = lc4;
    latest_local.frame_idx = frame_idx;
    latest_local.sample_idx = sample_idx;
    latest_local.valid = true;
}

static inline void update_latest_remote(uint16_t frame_idx, uint8_t sample_idx,
                                       int32_t lc1, int32_t lc2, int32_t lc3, int32_t lc4) {
    latest_remote.lc[0] = lc1;
    latest_remote.lc[1] = lc2;
    latest_remote.lc[2] = lc3;
    latest_remote.lc[3] = lc4;
    latest_remote.frame_idx = frame_idx;
    latest_remote.sample_idx = sample_idx;
    latest_remote.valid = true;
}

// Statistics tracking
static unsigned long local_samples_count = 0;
static unsigned long remote_samples_count = 0;
static unsigned long combined_samples_count = 0;
static unsigned long local_frames_count = 0;
static unsigned long remote_frames_count = 0;

static inline void count_sample(char type, uint16_t frame_idx, uint8_t sample_idx,
                               int32_t lc1, int32_t lc2, int32_t lc3, int32_t lc4) {
    // Just count samples for statistics - no serial output
    if (type == 'L') {
        local_samples_count++;
        if (sample_idx == 0) local_frames_count++;  // Count frames on first sample
        update_latest_local(frame_idx, sample_idx, lc1, lc2, lc3, lc4);
    } else if (type == 'R') {
        remote_samples_count++;
        if (sample_idx == 0) remote_frames_count++;  // Count frames on first sample
        update_latest_remote(frame_idx, sample_idx, lc1, lc2, lc3, lc4);
    }
    
    // Count combined samples when both are available
    if (latest_local.valid && latest_remote.valid) {
        combined_samples_count++;
    }
}

static inline void process_frame_ultra_fast(const InnerFrame* frame, char type) {
    const uint8_t* samples = frame->samples;
    uint16_t frame_idx = frame->frame_idx;
    
    for (int sample = 0; sample < SAMPLES_PER_FRAME; sample++) {
        const uint8_t* s = samples + (sample * 12);
        
        // Fastest possible unpack
        int32_t lc1 = s[0] | (s[1] << 8) | (s[2] << 16);
        if (lc1 & 0x800000) lc1 |= 0xFF000000;
        
        int32_t lc2 = s[3] | (s[4] << 8) | (s[5] << 16);
        if (lc2 & 0x800000) lc2 |= 0xFF000000;
        
        int32_t lc3 = s[6] | (s[7] << 8) | (s[8] << 16);
        if (lc3 & 0x800000) lc3 |= 0xFF000000;
        
        int32_t lc4 = s[9] | (s[10] << 8) | (s[11] << 16);
        if (lc4 & 0x800000) lc4 |= 0xFF000000;
        
        count_sample(type, frame_idx, sample, lc1, lc2, lc3, lc4);
    }
}

// ============================================================================
// SPI PROCESSING
// ============================================================================

static void spi_requeue_all() {
    for (int i = 0; i < QUEUED_XFERS; i++) {
        memset(&spi_trx[i], 0, sizeof(spi_trx[i]));
        spi_trx[i].length = FRAME_BYTES * 8;
        spi_trx[i].rx_buffer = spi_rxbuf[i];
        spi_trx[i].user = (void*)(intptr_t)i;
        ESP_ERROR_CHECK(spi_slave_queue_trans(SPI2_HOST, &spi_trx[i], portMAX_DELAY));
    }
}

static void spi_rx_task(void* param) {
    while (true) {
        spi_slave_transaction_t* ret;
        esp_err_t e = spi_slave_get_trans_result(SPI2_HOST, &ret, portMAX_DELAY);
        if (e != ESP_OK) continue;
        
        int idx = (int)(intptr_t)ret->user;
        InnerFrame* f = (InnerFrame*)spi_rxbuf[idx];

        if (output_local_data && f->sync[0] == 0xA5 && f->sync[1] == 0x5A) {
            uint16_t want = f->crc16;
            uint16_t got = crc16_ccitt_false((uint8_t*)f, offsetof(InnerFrame, crc16));
            
            if (want == got) {
                process_frame_ultra_fast(f, 'L');
            }
        }

        ESP_ERROR_CHECK(spi_slave_queue_trans(SPI2_HOST, ret, portMAX_DELAY));
    }
}

// ============================================================================
// ESP-NOW PROCESSING
// ============================================================================

static void espnow_rx_callback(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
    if (!output_remote_data || len != INNER_FRAME_SIZE) return;
    
    const InnerFrame* frame = (const InnerFrame*)data;
    if (!validate_inner_frame(frame)) return;
    
    process_frame_ultra_fast(frame, 'R');
}

// ============================================================================
// INITIALIZATION
// ============================================================================

static void espnow_init() {
#if USE_CUSTOM_MAC
    WiFi.mode(WIFI_STA);
    esp_wifi_set_mac(WIFI_IF_STA, (uint8_t*)CUSTOM_STA_MAC_DUAL);
#endif
    
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    
    if (esp_now_init() != ESP_OK) return;
    
    esp_now_register_recv_cb(espnow_rx_callback);
    
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, ESPNOW_PEER_MAC, 6);
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.encrypt = false;
    
    esp_now_add_peer(&peer_info);
}

static void handle_serial_commands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "LOCAL_ON") {
            output_local_data = true;
        } else if (command == "LOCAL_OFF") {
            output_local_data = false;
        } else if (command == "REMOTE_ON") {
            output_remote_data = true;
        } else if (command == "REMOTE_OFF") {
            output_remote_data = false;
        } else if (command == "MODE_COMPACT") {
            current_output_mode = MODE_COMPACT_TEXT;
        } else if (command == "MODE_BINARY") {
            current_output_mode = MODE_BINARY;
        } else if (command == "MODE_COMBINED") {
            current_output_mode = MODE_COMBINED_TEXT;
        } else if (command == "MODE_COMBINED_BIN") {
            current_output_mode = MODE_COMBINED_BINARY;
        }
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(921600);   // Safer baud rate to prevent USB driver issues
    delay(50);
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize ESP-NOW
    espnow_init();
    
    // Initialize SPI
    spi_bus_config_t bus{};
    bus.mosi_io_num = PIN_MOSI;
    bus.miso_io_num = PIN_MISO;
    bus.sclk_io_num = PIN_SCLK;
    bus.max_transfer_sz = FRAME_BYTES;

    spi_slave_interface_config_t slv{};
    slv.mode = 0;
    slv.spics_io_num = PIN_CS;
    slv.queue_size = QUEUED_XFERS;

    ESP_ERROR_CHECK(spi_slave_initialize(SPI2_HOST, &bus, &slv, SPI_DMA_CH_AUTO));
    gpio_set_pull_mode((gpio_num_t)PIN_CS, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)PIN_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)PIN_SCLK, GPIO_PULLUP_ONLY);

    spi_requeue_all();
    
    // Start high priority SPI task on Core 1 (max priority is 24)
    xTaskCreatePinnedToCore(spi_rx_task, "spi_rx", 4096, NULL, 24, NULL, 1);
}

void loop() {
    static unsigned long last_stats_time = 0;
    static unsigned long last_local_count = 0;
    static unsigned long last_remote_count = 0;
    static unsigned long last_combined_count = 0;
    
    unsigned long now = millis();
    
    // Print stats every 5 seconds
    if (now - last_stats_time >= 5000) {
        unsigned long local_rate = (local_samples_count - last_local_count) / 5;
        unsigned long remote_rate = (remote_samples_count - last_remote_count) / 5;
        unsigned long combined_rate = (combined_samples_count - last_combined_count) / 5;
        
        Serial.printf("STATS: T=%lu.%lus | LOCAL: %lu sps (%lu total) | REMOTE: %lu sps (%lu total) | COMBINED: %lu sps (%lu total) | FRAMES: L=%lu R=%lu\n",
                     now / 1000, (now % 1000) / 100,
                     local_rate, local_samples_count,
                     remote_rate, remote_samples_count,
                     combined_rate, combined_samples_count,
                     local_frames_count, remote_frames_count);
        
        last_stats_time = now;
        last_local_count = local_samples_count;
        last_remote_count = remote_samples_count;
        last_combined_count = combined_samples_count;
    }
    
    handle_serial_commands();
    vTaskDelay(pdMS_TO_TICKS(100));
}
