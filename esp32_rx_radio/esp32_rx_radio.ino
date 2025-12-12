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


#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_SCLK 12
#define PIN_CS   10

// UART pins for Teensy communication (Serial1)
#define TEENSY_UART_TX_PIN 2   // GPIO2 for UART TX to Teensy
#define TEENSY_UART_RX_PIN 3   // GPIO3 for UART RX from Teensy

// RGB LED pin
#define RGB_LED_PIN 38         // GPIO38 for WS2812B RGB LED status indicator


#define FRAME_BYTES 144
#define QUEUED_XFERS 8
#define SAMPLES_PER_FRAME 10

#define ESPNOW_CHANNEL 1
static const uint8_t ESPNOW_PEER_MAC[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x01};  // SPI Slave MAC
#define USE_CUSTOM_MAC 1
static const uint8_t CUSTOM_STA_MAC_DUAL[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x03};  // RX Radio MAC

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
// UART FORWARDING CONFIGURATION
// ============================================================================

#define UART_BAUD_RATE 921600   // Safe baud rate matching Serial console
#define UART_TX_PIN 17          // GPIO17 for UART TX
#define UART_RX_PIN 18          // GPIO18 for UART RX (not used but defined)

// UART packet structure (24 bytes total)
struct __attribute__((packed)) UartPacket {
    uint8_t sync[2];        // 0xAA, 0x55
    uint8_t type;           // 'L' = local, 'R' = remote, 'C' = combined
    uint16_t frame_idx;     // Frame index
    uint8_t sample_idx;     // Sample index within frame
    int32_t lc[4];          // Load cell values
    uint16_t crc16;         // CRC16 of packet (excluding sync and crc)
};

// UART statistics
static unsigned long uart_packets_sent = 0;
static unsigned long uart_bytes_sent = 0;
static unsigned long uart_send_errors = 0;

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
// UART FORWARDING FUNCTIONS
// ============================================================================

static inline uint16_t calculate_uart_crc(const UartPacket* packet) {
    // CRC of everything except sync bytes and crc field
    const uint8_t* data = (const uint8_t*)packet + 2; // Skip sync bytes
    uint32_t length = sizeof(UartPacket) - 4; // Exclude sync and crc
    return crc16_ccitt_false(data, length);
}

static inline bool send_uart_packet(uint8_t type, uint16_t frame_idx, uint8_t sample_idx,
                                   int32_t lc1, int32_t lc2, int32_t lc3, int32_t lc4) {
    // Debug: Print first few samples to check for duplicates
    static int debug_count = 0;
    if (debug_count < 10) {
        Serial.printf("UART_TX %c: [%ld, %ld, %ld, %ld] frame=%d sample=%d\n", 
                     type, lc1, lc2, lc3, lc4, frame_idx, sample_idx);
        debug_count++;
    }
    
    UartPacket packet;
    packet.sync[0] = 0xAA;
    packet.sync[1] = 0x55;
    packet.type = type;
    packet.frame_idx = frame_idx;
    packet.sample_idx = sample_idx;
    packet.lc[0] = lc1;
    packet.lc[1] = lc2;
    packet.lc[2] = lc3;
    packet.lc[3] = lc4;
    packet.crc16 = calculate_uart_crc(&packet);
    
    size_t bytes_written = Serial2.write((uint8_t*)&packet, sizeof(packet));
    
    if (bytes_written == sizeof(packet)) {
        uart_packets_sent++;
        uart_bytes_sent += bytes_written;
        return true;
    } else {
        uart_send_errors++;
        return false;
    }
}

static inline void forward_sample_to_uart(char type, uint16_t frame_idx, uint8_t sample_idx,
                                         int32_t lc1, int32_t lc2, int32_t lc3, int32_t lc4) {
    // Send only individual sample - Redis-like system on slave handles combining
    send_uart_packet(type, frame_idx, sample_idx, lc1, lc2, lc3, lc4);
}

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

// ESP-NOW command statistics
static unsigned long espnow_commands_sent = 0;
static unsigned long espnow_command_errors = 0;

// Response capture for PING test
static volatile bool espnow_response_ready = false;
static String espnow_response_data = "";
static SemaphoreHandle_t response_mutex = NULL;

// Message chunking for long calibration messages
static String message_buffer = "";
static bool receiving_chunks = false;

// ============================================================================
// ESP-NOW COMMAND STRUCTURES AND FUNCTIONS
// ============================================================================

// ESP-NOW command packet (16 bytes)
struct __attribute__((packed)) ESPNowCommand {
    uint8_t magic[2];   // 0xCD, 0xD1 (Command magic)
    uint8_t command[8]; // Command string (null-terminated)
    uint32_t timestamp; // Timestamp for deduplication
    uint16_t crc16;     // CRC16 of packet
};

// ESP-NOW response packet (64 bytes)
struct __attribute__((packed)) ESPNowResponse {
    uint8_t magic[2];   // 0xAB, 0xCD (Response magic)
    char response[60];  // Response string (null-terminated)
    uint16_t crc16;     // CRC16 of packet
};

// Forward declarations
bool create_espnow_command(ESPNowCommand* cmd, const char* command_str);
bool validate_espnow_command(const ESPNowCommand* cmd);
bool validate_espnow_response(const ESPNowResponse* resp);

// ============================================================================
// ESP-NOW COMMAND FORWARDING
// ============================================================================

static bool send_espnow_command(const char* command_str) {
    ESPNowCommand cmd;
    if (!create_espnow_command(&cmd, command_str)) {
        Serial.printf("[RX_RADIO] ✗ Failed to create command packet for '%s'\n", command_str);
        espnow_command_errors++;
        return false;
    }
    
    esp_err_t result = esp_now_send(ESPNOW_PEER_MAC, (uint8_t*)&cmd, sizeof(cmd));
    if (result == ESP_OK) {
        espnow_commands_sent++;
        Serial.printf("[RX_RADIO] ✓ Command '%s' sent via ESP-NOW\n", command_str);
        return true;
    } else {
        espnow_command_errors++;
        Serial.printf("[RX_RADIO] ✗ Failed to send command '%s' via ESP-NOW (error: %d)\n", command_str, result);
        return false;
    }
}

// ============================================================================
// ESP-NOW COMMAND HELPER FUNCTION IMPLEMENTATIONS
// ============================================================================

bool create_espnow_command(ESPNowCommand* cmd, const char* command_str) {
    if (strlen(command_str) > 8) return false; // Command too long (allow exactly 8 chars)
    
    memset(cmd, 0, sizeof(ESPNowCommand));
    cmd->magic[0] = 0xCD;
    cmd->magic[1] = 0xD1;
    
    // Copy command string - for 8-char commands, don't null terminate (use full buffer)
    if (strlen(command_str) == 8) {
        memcpy(cmd->command, command_str, 8); // Copy all 8 chars without null terminator
    } else {
        strncpy((char*)cmd->command, command_str, 7); // Copy up to 7 chars
        cmd->command[7] = '\0'; // Null terminate for shorter commands
    }
    
    cmd->timestamp = millis();
    
    // Calculate CRC16 of everything except CRC field
    cmd->crc16 = crc16_ccitt_false((const uint8_t*)cmd, sizeof(ESPNowCommand) - 2);
    return true;
}

bool validate_espnow_command(const ESPNowCommand* cmd) {
    if (cmd->magic[0] != 0xCD || cmd->magic[1] != 0xD1) {
        return false;
    }
    
    uint16_t expected_crc = crc16_ccitt_false((const uint8_t*)cmd, sizeof(ESPNowCommand) - 2);
    return cmd->crc16 == expected_crc;
}

bool validate_espnow_response(const ESPNowResponse* resp) {
    if (resp->magic[0] != 0xAB || resp->magic[1] != 0xCD) {
        return false;
    }
    
    uint16_t expected_crc = crc16_ccitt_false((const uint8_t*)resp, sizeof(ESPNowResponse) - 2);
    return resp->crc16 == expected_crc;
}

static inline void count_sample(char type, uint16_t frame_idx, uint8_t sample_idx,
                               int32_t lc1, int32_t lc2, int32_t lc3, int32_t lc4) {
    // Count samples for statistics and forward to UART
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
    
    // Forward sample to UART slave ESP32
    forward_sample_to_uart(type, frame_idx, sample_idx, lc1, lc2, lc3, lc4);
}

static inline void process_frame_ultra_fast(const InnerFrame* frame, char type) {
    const uint8_t* samples = frame->samples;
    uint16_t frame_idx = frame->frame_idx;
    
    for (int sample = 0; sample < SAMPLES_PER_FRAME; sample++) {
        int32_t lc1, lc2, lc3, lc4;
        
        // Use the tested unpacking function from common_frame.h
        extract_load_cell_sample(samples, sample, &lc1, &lc2, &lc3, &lc4);
        
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
    static unsigned long last_debug_time = 0;
    static unsigned long debug_packet_count = 0;
    static unsigned long response_packet_count = 0;
    static unsigned long data_frame_count = 0;
    static unsigned long unknown_packet_count = 0;
    
    debug_packet_count++;
    
    // Log ALL packets for debugging
    Serial.printf("[RX_RADIO] ESP-NOW RX: len=%d bytes from %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  len,
                  recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                  recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
    
    // Check if this is a response packet (64 bytes)
    if (len == sizeof(ESPNowResponse)) {
        response_packet_count++;
        Serial.printf("[RX_RADIO] → Response packet detected (size=%d)\n", sizeof(ESPNowResponse));
        const ESPNowResponse* resp = (const ESPNowResponse*)data;
        Serial.printf("[RX_RADIO] → Magic: %02X %02X (expected AB CD), CRC: %04X\n", 
                      resp->magic[0], resp->magic[1], resp->crc16);
        
        if (validate_espnow_response(resp)) {
            String response_str = String(resp->response);
            Serial.printf("[RX_RADIO] ✓ Response validated: '%s'\n", response_str.c_str());
            
            // Handle chunked messages
            if (response_str.startsWith("[CHUNK:")) {
                if (response_str.startsWith("[CHUNK:END]")) {
                    // End of chunked message, send complete message
                    if (message_buffer.length() > 0) {
                        Serial.printf("[RX_RADIO] → Complete chunked message: %s\n", message_buffer.c_str());
                        Serial2.printf("##REMOTE:%s\n", message_buffer.c_str());
                        Serial2.flush();
                        message_buffer = "";
                        receiving_chunks = false;
                    }
                } else {
                    // Extract chunk number and content
                    int colon1 = response_str.indexOf(':', 7); // After "[CHUNK:"
                    int colon2 = response_str.indexOf(':', colon1 + 1);
                    
                    if (colon1 > 0 && colon2 > 0) {
                        int chunk_num = response_str.substring(7, colon1).toInt();
                        String chunk_content = response_str.substring(colon2 + 1);
                        
                        Serial.printf("[RX_RADIO] → Received chunk %d: %s\n", chunk_num, chunk_content.c_str());
                        
                        if (chunk_num == 0) {
                            // First chunk, start new message
                            message_buffer = chunk_content;
                            receiving_chunks = true;
                        } else {
                            // Subsequent chunk, append to buffer
                            message_buffer += chunk_content;
                        }
                    }
                }
            } else {
                // Regular (non-chunked) message
                // Capture response for PING test
                if (response_mutex != NULL && xSemaphoreTake(response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    espnow_response_data = response_str;
                    espnow_response_ready = true;
                    xSemaphoreGive(response_mutex);
                    Serial.printf("[RX_RADIO] → Response captured for PING test\n");
                }
                
                // Forward response to BLE Slave via Serial2 with REMOTE: prefix
                // Use "##" prefix to distinguish text from binary UART packets
                Serial.printf("[RX_RADIO] → Forwarding to BLE Slave via Serial2...\n");
                Serial2.printf("##REMOTE:%s\n", response_str.c_str());
                Serial2.flush();
                Serial.printf("[RX_RADIO] ✓ Response forwarded successfully: ##REMOTE:%s\n", response_str.c_str());
            }
            return;
        } else {
            Serial.printf("[RX_RADIO] ✗ Response CRC validation FAILED!\n");
            uint16_t expected_crc = crc16_ccitt_false((const uint8_t*)resp, sizeof(ESPNowResponse) - 2);
            Serial.printf("[RX_RADIO] → Expected CRC: %04X, Got: %04X\n", expected_crc, resp->crc16);
            return;
        }
    }
    
    // Check if this is a data frame
    if (len == INNER_FRAME_SIZE) {
        data_frame_count++;
    } else {
        unknown_packet_count++;
        Serial.printf("[RX_RADIO] ⚠ Unknown packet size: %d (expected %d for data or %d for response)\n", 
                      len, INNER_FRAME_SIZE, sizeof(ESPNowResponse));
    }
    
    // Debug output every 5 seconds
    unsigned long now = millis();
    if (now - last_debug_time >= 5000) {
        Serial.printf("ESP-NOW: Received %lu packets in last 5s, len=%d, expected=%d\n", 
                     debug_packet_count, len, INNER_FRAME_SIZE);
        debug_packet_count = 0;
        last_debug_time = now;
    }
    
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
    Serial.printf("Custom MAC set: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  CUSTOM_STA_MAC_DUAL[0], CUSTOM_STA_MAC_DUAL[1], CUSTOM_STA_MAC_DUAL[2],
                  CUSTOM_STA_MAC_DUAL[3], CUSTOM_STA_MAC_DUAL[4], CUSTOM_STA_MAC_DUAL[5]);
#endif
    
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("WiFi channel set to: %d\n", ESPNOW_CHANNEL);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!");
        return;
    }
    Serial.println("ESP-NOW initialized successfully");
    
    esp_now_register_recv_cb(espnow_rx_callback);
    
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, ESPNOW_PEER_MAC, 6);
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.encrypt = false;
    
    if (esp_now_add_peer(&peer_info) == ESP_OK) {
        Serial.printf("ESP-NOW peer added: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      ESPNOW_PEER_MAC[0], ESPNOW_PEER_MAC[1], ESPNOW_PEER_MAC[2],
                      ESPNOW_PEER_MAC[3], ESPNOW_PEER_MAC[4], ESPNOW_PEER_MAC[5]);
    } else {
        Serial.println("Failed to add ESP-NOW peer!");
    }
}

static void handle_serial_commands() {
    // Handle commands from USB Serial Monitor
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        Serial.printf("[RX_RADIO] USB Command received: %s\n", command.c_str());
        process_command(command);
    }
    
    // Handle commands from ESP32 BLE Slave via UART2
    if (Serial2.available()) {
        String command = Serial2.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if (command.length() > 0) {
            Serial.printf("[RX_RADIO] UART Command received from BLE Slave: %s\n", command.c_str());
            process_command(command);
        }
    }
}

static void process_command(String command) {
    // Local Teensy control commands
    if (command == "START" || command == "STOP" || command == "RESTART" || command == "RESET" || 
        command == "ZERO" || command == "ZERO_STATUS" || command == "ZERO_RESET") {
            Serial.printf("[RX_RADIO] Forwarding '%s' to Local Teensy...\n", command.c_str());
            
            // Debug: Check Serial1 buffer space before sending
            Serial.printf("[RX_RADIO] Serial1 TX buffer space: %d bytes\n", Serial1.availableForWrite());
            
            Serial1.println(command);
            Serial1.flush();
            Serial.printf("[RX_RADIO] Command sent, waiting for response...\n");
            
            // Wait for response from Teensy (longer timeout for ZERO command)
            unsigned long timeout_ms = (command == "ZERO" ? 10000 : 2000); // 10s for ZERO, 2s for others
            unsigned long timeout = millis() + timeout_ms;
            bool response_received = false;
            while (millis() < timeout) {
                if (Serial1.available()) {
                    String response = Serial1.readStringUntil('\n');
                    Serial.printf("[RX_RADIO] Local Teensy Response: %s\n", response.c_str());
                    response_received = true;
                    break;
                }
                delay(10);
            }
            if (!response_received) {
                Serial.printf("[RX_RADIO] ⚠ Local Teensy response timeout after %lums\n", timeout_ms);
                Serial.printf("[RX_RADIO] Debug: Serial1 RX buffer has %d bytes available\n", Serial1.available());
            }
    }
    // Local PING test
    else if (command == "LOCAL_PING") {
        Serial.println("[RX_RADIO] ================================================");
        Serial.println("[RX_RADIO] LOCAL PING TEST INITIATED");
        Serial.println("[RX_RADIO] Path: BLE Slave -> RX Radio -> Local Teensy");
        Serial.println("[RX_RADIO] ================================================");
        Serial.printf("[RX_RADIO] Step 1/2: Sending 'PING' to Local Teensy via UART...\n");
        
        unsigned long start_time = millis();
        Serial1.println("PING");
        Serial1.flush();
        
        Serial.printf("[RX_RADIO] ✓ UART PING sent successfully\n");
        Serial.printf("[RX_RADIO] Step 2/2: Waiting for PONG response via UART...\n");
        
        // Wait for response from Local Teensy
        unsigned long timeout = millis() + 3000; // 3 second timeout
        bool got_response = false;
        String response_text = "";
        
        while (millis() < timeout) {
            if (Serial1.available()) {
                response_text = Serial1.readStringUntil('\n');
                response_text.trim();
                got_response = true;
                break;
            }
            delay(10);
        }
        
        unsigned long round_trip = millis() - start_time;
        
        if (got_response) {
            Serial.println("[RX_RADIO] ================================================");
            Serial.printf("[RX_RADIO] ✓ PONG RECEIVED: %s\n", response_text.c_str());
            Serial.printf("[RX_RADIO] ✓ Round-trip time: %lu ms\n", round_trip);
            Serial.println("[RX_RADIO] ✓ LOCAL PING TEST SUCCESS!");
            Serial.println("[RX_RADIO] ================================================");
            
            // Forward to BLE Slave with LOCAL: prefix
            // Use "##" prefix to distinguish text from binary UART packets
            Serial2.printf("##LOCAL:%s\n", response_text.c_str());
            Serial2.flush();
            Serial.printf("[RX_RADIO] → Forwarded to BLE Slave: ##LOCAL:%s\n", response_text.c_str());
        } else {
            Serial.println("[RX_RADIO] ================================================");
            Serial.printf("[RX_RADIO] ✗ No PONG response after %lu ms\n", round_trip);
            Serial.println("[RX_RADIO] ✗ LOCAL PING TEST FAILED!");
            Serial.println("[RX_RADIO] ================================================");
            
            // Forward timeout to BLE Slave
            // Use "##" prefix to distinguish text from binary UART packets
            Serial2.println("##LOCAL:PONG_TIMEOUT");
            Serial2.flush();
        }
    }
    // Remote Teensy control commands (via ESP-NOW)
    else if (command == "REMOTE_START" || command == "REMOTE_STOP" || command == "REMOTE_RESTART" || command == "REMOTE_RESET" ||
             command == "REMOTE_ZERO" || command == "REMOTE_ZERO_STATUS" || command == "REMOTE_ZERO_RESET") {
        String teensy_command = command.substring(7); // Remove "REMOTE_" prefix
        Serial.printf("[RX_RADIO] Sending '%s' to Remote Teensy via ESP-NOW...\n", teensy_command.c_str());
        if (send_espnow_command(teensy_command.c_str())) {
            Serial.printf("[RX_RADIO] ✓ Remote command '%s' sent successfully\n", teensy_command.c_str());
        } else {
            Serial.printf("[RX_RADIO] ✗ Failed to send remote command '%s'\n", teensy_command.c_str());
        }
    }
    // Remote PING test
    else if (command == "REMOTE_PING") {
        Serial.println("[RX_RADIO] ================================================");
        Serial.println("[RX_RADIO] REMOTE PING TEST INITIATED");
        Serial.println("[RX_RADIO] Path: BLE Slave -> RX Radio -> SPI Slave -> Remote Teensy");
        Serial.println("[RX_RADIO] ================================================");
        Serial.printf("[RX_RADIO] Step 1/2: Sending 'PING' to Remote Teensy via ESP-NOW...\n");
        
        // Clear previous response
        if (response_mutex != NULL && xSemaphoreTake(response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            espnow_response_ready = false;
            espnow_response_data = "";
            xSemaphoreGive(response_mutex);
        }
        
        unsigned long start_time = millis();
        if (send_espnow_command("PING")) {
            Serial.printf("[RX_RADIO] ✓ ESP-NOW PING sent successfully\n");
            Serial.printf("[RX_RADIO] Step 2/2: Waiting for PONG response via ESP-NOW...\n");
            
            // Wait for response from SPI Slave (captured by ESP-NOW callback)
            unsigned long timeout = millis() + 5000; // 5 second timeout
            bool got_response = false;
            String response_text = "";
            
            while (millis() < timeout) {
                // Check if ESP-NOW callback captured a response
                if (response_mutex != NULL && xSemaphoreTake(response_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    if (espnow_response_ready) {
                        response_text = espnow_response_data;
                        espnow_response_ready = false;
                        got_response = true;
                        xSemaphoreGive(response_mutex);
                        break;
                    }
                    xSemaphoreGive(response_mutex);
                }
                delay(10);
            }
            
            if (got_response) {
                unsigned long round_trip = millis() - start_time;
                Serial.println("[RX_RADIO] ================================================");
                Serial.printf("[RX_RADIO] ✓ PONG RECEIVED: %s\n", response_text.c_str());
                Serial.printf("[RX_RADIO] ✓ Round-trip time: %lu ms\n", round_trip);
                Serial.println("[RX_RADIO] ✓ REMOTE PING TEST SUCCESS!");
                Serial.println("[RX_RADIO] ================================================");
            } else {
                unsigned long elapsed = millis() - start_time;
                Serial.println("[RX_RADIO] ================================================");
                Serial.printf("[RX_RADIO] ✗ No PONG response after %lu ms\n", elapsed);
                Serial.println("[RX_RADIO] ✗ REMOTE PING TEST FAILED!");
                Serial.println("[RX_RADIO] ================================================");
            }
        } else {
            Serial.println("[RX_RADIO] ================================================");
            Serial.printf("[RX_RADIO] ✗ Failed to send ESP-NOW PING\n");
            Serial.println("[RX_RADIO] ✗ REMOTE PING TEST FAILED!");
            Serial.println("[RX_RADIO] ================================================");
        }
    }
    // ALL commands - control both local and remote Teensy simultaneously
    else if (command == "ALL_START" || command == "ALL_STOP" || command == "ALL_RESTART" || command == "ALL_RESET" ||
             command == "ALL_ZERO" || command == "ALL_ZERO_STATUS" || command == "ALL_ZERO_RESET") {
        String teensy_command = command.substring(4); // Remove "ALL_" prefix
        Serial.printf("[RX_RADIO] Executing '%s' on BOTH Local and Remote Teensy...\n", teensy_command.c_str());
        
        bool local_success = false;
        bool remote_success = false;
        
        // Send to Local Teensy first
        Serial.printf("[RX_RADIO] 1/2 Sending '%s' to Local Teensy...\n", teensy_command.c_str());
        
        // Debug: Check Serial1 buffer space before sending
        Serial.printf("[RX_RADIO] Serial1 TX buffer space: %d bytes\n", Serial1.availableForWrite());
        
        Serial1.println(teensy_command);
        Serial1.flush();
        Serial.printf("[RX_RADIO] Local command sent, waiting for response...\n");
        
        // Wait for local response (longer timeout for ZERO commands)
        unsigned long timeout_ms = (teensy_command == "ZERO" ? 10000 : 2000);
        unsigned long timeout = millis() + timeout_ms;
        bool local_response_received = false;
        while (millis() < timeout) {
            if (Serial1.available()) {
                String response = Serial1.readStringUntil('\n');
                Serial.printf("[RX_RADIO] Local Teensy Response: %s\n", response.c_str());
                local_success = response.indexOf("OK") >= 0;
                local_response_received = true;
                break;
            }
            delay(10);
        }
        if (!local_response_received) {
            Serial.printf("[RX_RADIO] ⚠ Local Teensy response timeout after %lums\n", timeout_ms);
            Serial.printf("[RX_RADIO] Debug: Serial1 RX buffer has %d bytes available\n", Serial1.available());
        }
        
        // Send to Remote Teensy via ESP-NOW
        Serial.printf("[RX_RADIO] 2/2 Sending '%s' to Remote Teensy via ESP-NOW...\n", teensy_command.c_str());
        remote_success = send_espnow_command(teensy_command.c_str());
        
        // Summary
        Serial.printf("[RX_RADIO] === ALL_%s SUMMARY ===\n", teensy_command.c_str());
        Serial.printf("  Local Teensy:  %s\n", local_success ? "✓ SUCCESS" : "✗ FAILED");
        Serial.printf("  Remote Teensy: %s\n", remote_success ? "✓ SUCCESS" : "✗ FAILED");
        if (local_success && remote_success) {
            Serial.printf("[RX_RADIO] ✓ ALL_%s completed successfully on BOTH units\n", teensy_command.c_str());
        } else {
            Serial.printf("[RX_RADIO] ⚠ ALL_%s partially failed - check individual responses\n", teensy_command.c_str());
        }
        Serial.println("=============================");
    }
    // ESP32 control commands
    else if (command == "LOCAL_ON") {
        output_local_data = true;
        Serial.println("[RX_RADIO] ✓ Local data output enabled");
    } else if (command == "LOCAL_OFF") {
        output_local_data = false;
        Serial.println("[RX_RADIO] ✓ Local data output disabled");
    } else if (command == "REMOTE_ON") {
        output_remote_data = true;
        Serial.println("[RX_RADIO] ✓ Remote data output enabled");
    } else if (command == "REMOTE_OFF") {
        output_remote_data = false;
        Serial.println("[RX_RADIO] ✓ Remote data output disabled");
    } else if (command == "TEST_UART") {
        Serial.println("[RX_RADIO] === UART COMMUNICATION TEST ===");
        Serial.printf("[RX_RADIO] Testing UART communication with Local Teensy...\n");
        Serial.printf("[RX_RADIO] TX Pin: %d, RX Pin: %d, Baud: 115200\n", TEENSY_UART_TX_PIN, TEENSY_UART_RX_PIN);
        
        // Send a simple test command
        Serial1.println("HELP");
        Serial1.flush();
        Serial.printf("[RX_RADIO] Sent 'HELP' command, waiting for response...\n");
        
        unsigned long timeout = millis() + 3000; // 3 second timeout
        bool got_response = false;
        while (millis() < timeout) {
            if (Serial1.available()) {
                String response = Serial1.readStringUntil('\n');
                Serial.printf("[RX_RADIO] Local Teensy: %s\n", response.c_str());
                got_response = true;
                // Continue reading until timeout to get full response
            }
            delay(10);
        }
        
        if (!got_response) {
            Serial.println("[RX_RADIO] ✗ No response from Local Teensy - check wiring!");
            Serial.printf("[RX_RADIO] Expected: ESP32 TX(GPIO%d) -> Teensy RX3(Pin15)\n", TEENSY_UART_TX_PIN);
            Serial.printf("[RX_RADIO] Expected: ESP32 RX(GPIO%d) -> Teensy TX3(Pin14)\n", TEENSY_UART_RX_PIN);
        } else {
            Serial.println("[RX_RADIO] ✓ UART communication working!");
        }
        Serial.println("==========================================");
    } else if (command == "STATUS") {
        Serial.printf("[RX_RADIO] === ESP32 RX RADIO STATUS ===\n");
        Serial.printf("  Local data: %s\n", output_local_data ? "ON" : "OFF");
        Serial.printf("  Remote data: %s\n", output_remote_data ? "ON" : "OFF");
        Serial.printf("  Local samples: %lu\n", local_samples_count);
        Serial.printf("  Remote samples: %lu\n", remote_samples_count);
        Serial.printf("  UART packets: %lu\n", uart_packets_sent);
        Serial.printf("  ESP-NOW commands sent: %lu\n", espnow_commands_sent);
        Serial.printf("  ESP-NOW command errors: %lu\n", espnow_command_errors);
        Serial.printf("  Free heap: %lu bytes\n", ESP.getFreeHeap());
        Serial.println("=====================================");
    }
    // Local Teensy Automated Calibration
    else if (command == "AUTO_CAL_START" || command == "AUTOMATED_CALIBRATION") {
        Serial.println("[RX_RADIO] Starting LOCAL Teensy automated calibration...");
        Serial1.println("AUTO_CAL_START");
        Serial1.flush();
        Serial.println("[RX_RADIO] Waiting for calibration IDs from LOCAL Teensy...");
    }
    // Remote Teensy Automated Calibration (ESP-NOW has 8-char limit!)
    else if (command == "REMOTE_AUTO_CAL") {
        Serial.println("[RX_RADIO] Starting REMOTE Teensy automated calibration via ESP-NOW...");
        send_espnow_command("AUTOCAL");  // 7 chars - SPI Slave will translate to AUTO_CAL_START
    } else if (command == "REMOTE_CONTINUE") {
        Serial.println("[RX_RADIO] Sending CONTINUE to REMOTE Teensy via ESP-NOW...");
        Serial.printf("[RX_RADIO] DEBUG: Sending 'CONTINUE' via ESP-NOW\n");
        send_espnow_command("CONTINUE");  // 8 chars - fits exactly!
        Serial.printf("[RX_RADIO] DEBUG: CONTINUE sent to REMOTE Teensy via ESP-NOW\n");
    } else if (command == "REMOTE_SKIP") {
        Serial.println("[RX_RADIO] Sending SKIP to REMOTE Teensy via ESP-NOW...");
        Serial.printf("[RX_RADIO] DEBUG: Sending 'SKIP' via ESP-NOW\n");
        send_espnow_command("SKIP");  // 4 chars
        Serial.printf("[RX_RADIO] DEBUG: SKIP sent to REMOTE Teensy via ESP-NOW\n");
    } else if (command == "REMOTE_ABORT") {
        Serial.println("[RX_RADIO] Sending ABORT to REMOTE Teensy via ESP-NOW...");
        Serial.printf("[RX_RADIO] DEBUG: Sending 'ABORT' via ESP-NOW\n");
        send_espnow_command("ABORT");  // 5 chars
        Serial.printf("[RX_RADIO] DEBUG: ABORT sent to REMOTE Teensy via ESP-NOW\n");
    } else if (command == "REMOTE_AUTO_CAL_STATUS") {
        Serial.println("[RX_RADIO] Requesting calibration status from REMOTE Teensy...");
        send_espnow_command("CALSTAT");  // 7 chars - SPI Slave will translate to AUTO_CAL_STATUS
    }
    // Local calibration interactive commands
    else if (command == "CONTINUE") {
        Serial.println("[RX_RADIO] Forwarding CONTINUE to LOCAL Teensy...");
        Serial1.println("CONTINUE");
        Serial1.flush();
    } else if (command == "SKIP") {
        Serial.println("[RX_RADIO] Forwarding SKIP to LOCAL Teensy...");
        Serial1.println("SKIP");
        Serial1.flush();
    } else if (command == "ABORT") {
        Serial.println("[RX_RADIO] Forwarding ABORT to LOCAL Teensy...");
        Serial1.println("ABORT");
        Serial1.flush();
    } else if (command == "AUTO_CAL_STATUS") {
        Serial.println("[RX_RADIO] Requesting calibration status from LOCAL Teensy...");
        Serial1.println("AUTO_CAL_STATUS");
        Serial1.flush();
    } else if (command == "HELP") {
        Serial.println("[RX_RADIO] === AVAILABLE COMMANDS ===");
        Serial.println("  Local Teensy Control:");
        Serial.println("    START         - Start local Teensy data acquisition");
        Serial.println("    STOP          - Stop local Teensy data acquisition");
        Serial.println("    RESTART       - Restart local Teensy data acquisition");
        Serial.println("    RESET         - Reset local Teensy");
        Serial.println("    ZERO          - Zero local Teensy load cells (1500 samples)");
        Serial.println("    ZERO_STATUS   - Show local Teensy zeroing status");
        Serial.println("    ZERO_RESET    - Reset local Teensy zero offsets");
        Serial.println("  Remote Teensy Control (via ESP-NOW):");
        Serial.println("    REMOTE_START  - Start remote Teensy data acquisition");
        Serial.println("    REMOTE_STOP   - Stop remote Teensy data acquisition");
        Serial.println("    REMOTE_RESTART- Restart remote Teensy data acquisition");
        Serial.println("    REMOTE_RESET  - Reset remote Teensy");
        Serial.println("    REMOTE_ZERO   - Zero remote Teensy load cells (1500 samples)");
        Serial.println("    REMOTE_ZERO_STATUS - Show remote Teensy zeroing status");
        Serial.println("    REMOTE_ZERO_RESET  - Reset remote Teensy zero offsets");
        Serial.println("  Dual Teensy Control (Both Local & Remote):");
        Serial.println("    ALL_START     - Start BOTH Teensy units simultaneously");
        Serial.println("    ALL_STOP      - Stop BOTH Teensy units simultaneously");
        Serial.println("    ALL_RESTART   - Restart BOTH Teensy units simultaneously");
        Serial.println("    ALL_RESET     - Reset BOTH Teensy units simultaneously");
        Serial.println("    ALL_ZERO      - Zero BOTH Teensy units simultaneously");
        Serial.println("    ALL_ZERO_STATUS - Show zeroing status for BOTH units");
        Serial.println("    ALL_ZERO_RESET  - Reset zero offsets for BOTH units");
        Serial.println("  Test Commands:");
        Serial.println("    LOCAL_PING    - Ping local Teensy (UART round-trip test)");
        Serial.println("    REMOTE_PING   - Ping remote Teensy (ESP-NOW round-trip test)");
        Serial.println("  ESP32 Control:");
        Serial.println("    LOCAL_ON/OFF  - Enable/disable local data");
        Serial.println("    REMOTE_ON/OFF - Enable/disable remote data");
        Serial.println("    STATUS        - Show system status");
        Serial.println("    TEST_UART     - Test UART communication with local Teensy");
        Serial.println("    HELP          - Show this help");
        Serial.println("=====================================");
    } else if (command == "") {
        // Empty command, do nothing
    } else {
        Serial.printf("[RX_RADIO] ✗ Unknown command: '%s'\n", command.c_str());
        Serial.println("[RX_RADIO] Type 'HELP' for available commands");
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(921600);   // Safer baud rate to prevent USB driver issues
    delay(50);
    
    // Create response mutex for PING test synchronization
    response_mutex = xSemaphoreCreateMutex();
    if (response_mutex == NULL) {
        Serial.println("[RX_RADIO] ⚠ Failed to create response mutex!");
    } else {
        Serial.println("[RX_RADIO] ✓ Response mutex created");
    }
    
    // Initialize UART2 for forwarding data to slave ESP32
    Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial2.setTxBufferSize(8192);  // Increase TX buffer for high throughput
    Serial.printf("UART2 initialized at %d bps on TX pin %d\n", UART_BAUD_RATE, UART_TX_PIN);
    
    // Initialize Serial1 for Teensy communication
    Serial1.begin(115200, SERIAL_8N1, TEENSY_UART_RX_PIN, TEENSY_UART_TX_PIN);
    Serial.printf("Serial1 initialized at 115200 bps for Teensy communication (TX=%d, RX=%d)\n", 
                  TEENSY_UART_TX_PIN, TEENSY_UART_RX_PIN);
    
    // Test UART connection at startup
    delay(1000); // Give Teensy time to boot
    Serial.println("[RX_RADIO] Testing UART connection to Local Teensy...");
    Serial1.println("STATUS");
    Serial1.flush();
    
    unsigned long test_timeout = millis() + 2000;
    bool teensy_responded = false;
    while (millis() < test_timeout) {
        if (Serial1.available()) {
            String response = Serial1.readStringUntil('\n');
            Serial.printf("[RX_RADIO] Local Teensy startup response: %s\n", response.c_str());
            teensy_responded = true;
            break;
        }
        delay(10);
    }
    
    if (!teensy_responded) {
        Serial.println("[RX_RADIO] ⚠ WARNING: No response from Local Teensy during startup!");
        Serial.println("[RX_RADIO] Check UART wiring: ESP32 TX(GPIO2)->Teensy RX3(Pin15), ESP32 RX(GPIO3)->Teensy TX3(Pin14)");
        Serial.println("[RX_RADIO] Use 'TEST_UART' command to diagnose communication issues");
    } else {
        Serial.println("[RX_RADIO] ✓ Local Teensy UART communication verified");
    }
    
    // Send a startup test packet
    delay(1000);  // Wait for slave to initialize
    UartPacket startup_packet;
    startup_packet.sync[0] = 0xAA;
    startup_packet.sync[1] = 0x55;
    startup_packet.type = 'S';  // Startup packet
    startup_packet.frame_idx = 0;
    startup_packet.sample_idx = 0;
    startup_packet.lc[0] = 11111;
    startup_packet.lc[1] = 22222;
    startup_packet.lc[2] = 33333;
    startup_packet.lc[3] = 44444;
    startup_packet.crc16 = calculate_uart_crc(&startup_packet);
    
    size_t bytes_sent = Serial2.write((uint8_t*)&startup_packet, sizeof(startup_packet));
    Serial.printf("Startup UART test: Sent %d bytes\n", bytes_sent);
    
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
    
    Serial.println("[RX_RADIO] ==========================================");
    Serial.println("[RX_RADIO] ESP32 RX Radio ready for commands");
        Serial.println("[RX_RADIO] Local Teensy: START, STOP, RESTART, RESET, ZERO, ZERO_STATUS, ZERO_RESET");
        Serial.println("[RX_RADIO] Remote Teensy: REMOTE_START, REMOTE_STOP, REMOTE_RESTART, REMOTE_RESET, REMOTE_ZERO, REMOTE_ZERO_STATUS, REMOTE_ZERO_RESET");
        Serial.println("[RX_RADIO] Both Teensy: ALL_START, ALL_STOP, ALL_RESTART, ALL_RESET, ALL_ZERO, ALL_ZERO_STATUS, ALL_ZERO_RESET");
    Serial.println("[RX_RADIO] ESP32 commands: LOCAL_ON/OFF, REMOTE_ON/OFF, STATUS, HELP");
    Serial.println("[RX_RADIO] ==========================================");
}

void loop() {
    static unsigned long last_stats_time = 0;
    static unsigned long last_local_count = 0;
    static unsigned long last_remote_count = 0;
    static unsigned long last_combined_count = 0;
    static unsigned long last_uart_packets = 0;
    static unsigned long last_uart_bytes = 0;
    
    unsigned long now = millis();
    
    // Print stats every 5 seconds
    if (now - last_stats_time >= 5000) {
        unsigned long local_rate = (local_samples_count - last_local_count) / 5;
        unsigned long remote_rate = (remote_samples_count - last_remote_count) / 5;
        unsigned long combined_rate = (combined_samples_count - last_combined_count) / 5;
        unsigned long uart_packet_rate = (uart_packets_sent - last_uart_packets) / 5;
        unsigned long uart_byte_rate = (uart_bytes_sent - last_uart_bytes) / 5;
        unsigned long uart_kbps = (uart_byte_rate * 8) / 1000;  // Kbps
        
        Serial.printf("STATS: T=%lu.%lus | LOCAL: %lu sps (%lu total) | REMOTE: %lu sps (%lu total) | COMBINED: %lu sps (%lu total) | FRAMES: L=%lu R=%lu\n",
                     now / 1000, (now % 1000) / 100,
                     local_rate, local_samples_count,
                     remote_rate, remote_samples_count,
                     combined_rate, combined_samples_count,
                     local_frames_count, remote_frames_count);
        
        Serial.printf("UART: %lu pps (%lu total) | %lu Bps (%lu kbps) | Errors: %lu | Buffer: %d bytes free\n",
                     uart_packet_rate, uart_packets_sent,
                     uart_byte_rate, uart_kbps,
                     uart_send_errors, Serial2.availableForWrite());
        
        last_stats_time = now;
        last_local_count = local_samples_count;
        last_remote_count = remote_samples_count;
        last_combined_count = combined_samples_count;
        last_uart_packets = uart_packets_sent;
        last_uart_bytes = uart_bytes_sent;
    }
    
    // Forward calibration messages and IDs from Local Teensy to BLE Slave
    while (Serial1.available()) {
        String line = Serial1.readStringUntil('\n');
        line.trim();
        
        // Forward structured calibration IDs (new system)
        // Use "##" prefix to distinguish text from binary UART packets
        if (line.startsWith("CAL_")) {
            Serial.printf("[RX_RADIO] ID RX: %s\n", line.c_str());
            Serial2.printf("##LOCAL:%s\n", line.c_str());
            Serial2.flush();
        }
        // Also forward raw calibration messages (for debugging/fallback)
        else if (line.startsWith("[AUTO-CAL]")) {
            Serial.printf("[RX_RADIO] (raw cal msg): %s\n", line.c_str());
            Serial2.printf("##LOCAL:%s\n", line.c_str());
            Serial2.flush();
        }
        // Forward any other responses from Local Teensy
        else if (line.length() > 0) {
            Serial.printf("[RX_RADIO] Local Teensy: %s\n", line.c_str());
            Serial2.printf("##LOCAL:%s\n", line.c_str());
            Serial2.flush();
        }
    }
    
    handle_serial_commands();
    vTaskDelay(pdMS_TO_TICKS(100));
}
