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

// BLE includes
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"

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
// BLE CONFIGURATION
// ============================================================================

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789ABC"
#define DATA_CHAR_UUID      "87654321-4321-4321-4321-CBA987654321"
#define STATS_CHAR_UUID     "11111111-2222-3333-4444-555555555555"
#define CONTROL_CHAR_UUID   "AAAAAAAA-BBBB-CCCC-DDDD-EEEEEEEEEEEE"

// BLE packet size optimization for maximum throughput
#define BLE_PACKET_SIZE 240  // Maximum BLE packet size minus headers

BLEServer* pServer = NULL;
BLECharacteristic* pDataCharacteristic = NULL;
BLECharacteristic* pStatsCharacteristic = NULL;
BLECharacteristic* pControlCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ============================================================================
// OUTPUT MODES
// ============================================================================

enum OutputMode {
    MODE_COMPACT_TEXT = 0,    // L:frame:sample:lc1:lc2:lc3:lc4
    MODE_BINARY = 1,          // Binary struct
    MODE_COMBINED_TEXT = 2,   // C:frame:sample:local_lc1:...:remote_lc4
    MODE_COMBINED_BINARY = 3, // Combined binary for wireless
    MODE_BLE_OPTIMIZED = 4    // Optimized for BLE transmission
};

static OutputMode current_output_mode = MODE_BLE_OPTIMIZED;
static bool output_local_data = true;
static bool output_remote_data = true;
static bool ble_output_enabled = true;

// ============================================================================
// OPTIMIZED BLE DATA STRUCTURES
// ============================================================================

struct __attribute__((packed)) BLEDataPacket {
    uint8_t packet_type;      // 0=combined_data, 1=stats, 2=heartbeat
    uint32_t timestamp;       // milliseconds
    uint16_t sequence;        // packet sequence number
    uint8_t data_count;       // number of data entries in this packet
    uint8_t reserved;         // padding for alignment
};

struct __attribute__((packed)) BLECombinedData {
    uint16_t frame_idx;
    uint8_t sample_idx;
    uint8_t flags;            // bit0=has_local, bit1=has_remote
    int32_t local_lc[4];      // 16 bytes
    int32_t remote_lc[4];     // 16 bytes
    // Total: 38 bytes per sample
};

struct __attribute__((packed)) BLEStatsPacket {
    uint8_t packet_type;      // 1
    uint32_t timestamp;
    uint32_t local_samples_total;
    uint32_t remote_samples_total;
    uint32_t combined_samples_total;
    uint32_t local_samples_rate;
    uint32_t remote_samples_rate;
    uint32_t combined_samples_rate;
    uint32_t local_frames_total;
    uint32_t remote_frames_total;
    uint16_t buffer_usage_percent;
    uint8_t connection_quality;
    uint8_t reserved;
};

// ============================================================================
// BLE BUFFERING SYSTEM
// ============================================================================

#define BLE_BUFFER_SIZE 10
static BLECombinedData ble_buffer[BLE_BUFFER_SIZE];
static volatile int ble_buffer_write_idx = 0;
static volatile int ble_buffer_read_idx = 0;
static volatile int ble_buffer_count = 0;
static uint16_t ble_sequence = 0;
static SemaphoreHandle_t ble_buffer_mutex;

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
    uint32_t timestamp;
};

static LatestData latest_local = {0};
static LatestData latest_remote = {0};

// SPI buffers
static uint8_t spi_rxbuf[QUEUED_XFERS][FRAME_BYTES] __attribute__((aligned(4)));
static spi_slave_transaction_t spi_trx[QUEUED_XFERS];

// ============================================================================
// BLE SERVER CALLBACKS
// ============================================================================

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("BLE Client connected");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("BLE Client disconnected");
    }
};

class MyControlCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
      // getValue() returns Arduino String in your setup
      String value = pCharacteristic->getValue();   // <-- fix: use String, not std::string
      if (value.length() > 0) {
        value.trim();
  
        if      (value == "LOCAL_ON")  { output_local_data = true;  }
        else if (value == "LOCAL_OFF") { output_local_data = false; }
        else if (value == "REMOTE_ON") { output_remote_data = true; }
        else if (value == "REMOTE_OFF"){ output_remote_data = false; }
        else if (value == "BLE_ON")    { ble_output_enabled = true; }
        else if (value == "BLE_OFF")   { ble_output_enabled = false; }
        else if (value == "MODE_BLE")  { current_output_mode = MODE_BLE_OPTIMIZED; }
        // add more commands as needed
  
        Serial.printf("BLE Command received: %s\n", value.c_str());
      }
    }
  };
  

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
    latest_local.timestamp = millis();
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
    latest_remote.timestamp = millis();
}

// Statistics tracking
static unsigned long local_samples_count = 0;
static unsigned long remote_samples_count = 0;
static unsigned long combined_samples_count = 0;
static unsigned long local_frames_count = 0;
static unsigned long remote_frames_count = 0;

static inline void add_to_ble_buffer(uint16_t frame_idx, uint8_t sample_idx, uint8_t flags) {
    if (!ble_output_enabled || !deviceConnected) return;
    
    if (xSemaphoreTake(ble_buffer_mutex, 0) == pdTRUE) {  // Non-blocking
        if (ble_buffer_count < BLE_BUFFER_SIZE) {
            BLECombinedData* entry = &ble_buffer[ble_buffer_write_idx];
            entry->frame_idx = frame_idx;
            entry->sample_idx = sample_idx;
            entry->flags = flags;
            
            if (flags & 0x01) {  // has_local
                memcpy(entry->local_lc, latest_local.lc, sizeof(latest_local.lc));
            }
            if (flags & 0x02) {  // has_remote
                memcpy(entry->remote_lc, latest_remote.lc, sizeof(latest_remote.lc));
            }
            
            ble_buffer_write_idx = (ble_buffer_write_idx + 1) % BLE_BUFFER_SIZE;
            ble_buffer_count++;
        }
        xSemaphoreGive(ble_buffer_mutex);
    }
}

static inline void count_sample(char type, uint16_t frame_idx, uint8_t sample_idx,
                               int32_t lc1, int32_t lc2, int32_t lc3, int32_t lc4) {
    uint8_t flags = 0;
    
    if (type == 'L') {
        local_samples_count++;
        if (sample_idx == 0) local_frames_count++;
        update_latest_local(frame_idx, sample_idx, lc1, lc2, lc3, lc4);
        flags |= 0x01;  // has_local
    } else if (type == 'R') {
        remote_samples_count++;
        if (sample_idx == 0) remote_frames_count++;
        update_latest_remote(frame_idx, sample_idx, lc1, lc2, lc3, lc4);
        flags |= 0x02;  // has_remote
    }
    
    // Add combined data when both are available
    if (latest_local.valid && latest_remote.valid) {
        combined_samples_count++;
        flags |= 0x03;  // has both
        add_to_ble_buffer(frame_idx, sample_idx, flags);
    } else if (flags != 0) {
        // Add individual data
        add_to_ble_buffer(frame_idx, sample_idx, flags);
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
// BLE TRANSMISSION TASK
// ============================================================================

void ble_transmission_task(void* param) {
    uint8_t packet_buffer[BLE_PACKET_SIZE];
    
    while (true) {
        if (!deviceConnected || !ble_output_enabled) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        int data_to_send = 0;
        BLECombinedData temp_data[6];  // Maximum that fits in one BLE packet
        
        // Collect data from buffer
        if (xSemaphoreTake(ble_buffer_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            while (ble_buffer_count > 0 && data_to_send < 6) {
                temp_data[data_to_send] = ble_buffer[ble_buffer_read_idx];
                ble_buffer_read_idx = (ble_buffer_read_idx + 1) % BLE_BUFFER_SIZE;
                ble_buffer_count--;
                data_to_send++;
            }
            xSemaphoreGive(ble_buffer_mutex);
        }
        
        if (data_to_send > 0) {
            // Create BLE packet
            BLEDataPacket* header = (BLEDataPacket*)packet_buffer;
            header->packet_type = 0;  // combined_data
            header->timestamp = millis();
            header->sequence = ble_sequence++;
            header->data_count = data_to_send;
            header->reserved = 0;
            
            // Copy data
            memcpy(packet_buffer + sizeof(BLEDataPacket), temp_data, data_to_send * sizeof(BLECombinedData));
            
            size_t total_size = sizeof(BLEDataPacket) + (data_to_send * sizeof(BLECombinedData));
            
            // Send via BLE
            pDataCharacteristic->setValue(packet_buffer, total_size);
            pDataCharacteristic->notify();
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz transmission rate
    }
}

// ============================================================================
// BLE STATS TRANSMISSION
// ============================================================================

void ble_stats_task(void* param) {
    static unsigned long last_local_count = 0;
    static unsigned long last_remote_count = 0;
    static unsigned long last_combined_count = 0;
    static unsigned long last_time = 0;
    
    while (true) {
        if (deviceConnected && ble_output_enabled) {
            unsigned long now = millis();
            unsigned long dt = now - last_time;
            
            if (dt >= 1000) {  // Every second
                BLEStatsPacket stats = {0};
                stats.packet_type = 1;
                stats.timestamp = now;
                stats.local_samples_total = local_samples_count;
                stats.remote_samples_total = remote_samples_count;
                stats.combined_samples_total = combined_samples_count;
                
                if (dt > 0) {
                    stats.local_samples_rate = ((local_samples_count - last_local_count) * 1000) / dt;
                    stats.remote_samples_rate = ((remote_samples_count - last_remote_count) * 1000) / dt;
                    stats.combined_samples_rate = ((combined_samples_count - last_combined_count) * 1000) / dt;
                }
                
                stats.local_frames_total = local_frames_count;
                stats.remote_frames_total = remote_frames_count;
                stats.buffer_usage_percent = (ble_buffer_count * 100) / BLE_BUFFER_SIZE;
                stats.connection_quality = 100;  // Simplified
                
                pStatsCharacteristic->setValue((uint8_t*)&stats, sizeof(stats));
                pStatsCharacteristic->notify();
                
                last_local_count = local_samples_count;
                last_remote_count = remote_samples_count;
                last_combined_count = combined_samples_count;
                last_time = now;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
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

static void ble_init() {
    BLEDevice::init("ESP32-DataLogger");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Data characteristic
    pDataCharacteristic = pService->createCharacteristic(
                            DATA_CHAR_UUID,
                            BLECharacteristic::PROPERTY_READ |
                            BLECharacteristic::PROPERTY_NOTIFY
                          );
    pDataCharacteristic->addDescriptor(new BLE2902());

    // Stats characteristic
    pStatsCharacteristic = pService->createCharacteristic(
                             STATS_CHAR_UUID,
                             BLECharacteristic::PROPERTY_READ |
                             BLECharacteristic::PROPERTY_NOTIFY
                           );
    pStatsCharacteristic->addDescriptor(new BLE2902());

    // Control characteristic
    pControlCharacteristic = pService->createCharacteristic(
                               CONTROL_CHAR_UUID,
                               BLECharacteristic::PROPERTY_WRITE
                             );
    pControlCharacteristic->setCallbacks(new MyControlCallbacks());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();

    Serial.println("BLE Server started, waiting for connections...");
}

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
        } else if (command == "BLE_ON") {
            ble_output_enabled = true;
        } else if (command == "BLE_OFF") {
            ble_output_enabled = false;
        } else if (command == "MODE_COMPACT") {
            current_output_mode = MODE_COMPACT_TEXT;
        } else if (command == "MODE_BINARY") {
            current_output_mode = MODE_BINARY;
        } else if (command == "MODE_COMBINED") {
            current_output_mode = MODE_COMBINED_TEXT;
        } else if (command == "MODE_COMBINED_BIN") {
            current_output_mode = MODE_COMBINED_BINARY;
        } else if (command == "MODE_BLE") {
            current_output_mode = MODE_BLE_OPTIMIZED;
        }
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(921600);
    delay(50);
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create BLE buffer mutex
    ble_buffer_mutex = xSemaphoreCreateMutex();
    
    // Initialize BLE
    ble_init();
    
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
    
    // Start high priority SPI task on Core 1
    xTaskCreatePinnedToCore(spi_rx_task, "spi_rx", 4096, NULL, 24, NULL, 1);
    
    // Start BLE tasks on Core 0
    xTaskCreatePinnedToCore(ble_transmission_task, "ble_tx", 4096, NULL, 20, NULL, 0);
    xTaskCreatePinnedToCore(ble_stats_task, "ble_stats", 2048, NULL, 10, NULL, 0);
    
    Serial.println("System initialized with BLE support");
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
        
        Serial.printf("STATS: T=%lu.%lus | LOCAL: %lu sps (%lu total) | REMOTE: %lu sps (%lu total) | COMBINED: %lu sps (%lu total) | FRAMES: L=%lu R=%lu | BLE: %s (%d%% buf)\n",
                     now / 1000, (now % 1000) / 100,
                     local_rate, local_samples_count,
                     remote_rate, remote_samples_count,
                     combined_rate, combined_samples_count,
                     local_frames_count, remote_frames_count,
                     deviceConnected ? "CONN" : "DISC",
                     (ble_buffer_count * 100) / BLE_BUFFER_SIZE);
        
        last_stats_time = now;
        last_local_count = local_samples_count;
        last_remote_count = remote_samples_count;
        last_combined_count = combined_samples_count;
    }
    
    // Handle BLE reconnection
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        Serial.println("Restarting BLE advertising");
        oldDeviceConnected = deviceConnected;
    }
    
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
    
    handle_serial_commands();
    vTaskDelay(pdMS_TO_TICKS(100));
}