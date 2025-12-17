#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "common_frame.h"

// ============================================================================
// UART CONFIGURATION (must match master ESP32)
// ============================================================================

#define UART_BAUD_RATE 921600   // Safe baud rate matching Serial console
#define UART_RX_PIN 18          // GPIO18 for UART RX from ESP32 RX Radio
#define UART_TX_PIN 17          // GPIO17 for UART TX to ESP32 RX Radio

// UART packet structure (24 bytes total) - must match master
struct __attribute__((packed)) UartPacket {
    uint8_t sync[2];        // 0xAA, 0x55
    uint8_t type;           // 'L' = local, 'R' = remote, 'C' = combined
    uint16_t frame_idx;     // Frame index
    uint8_t sample_idx;     // Sample index within frame
    int32_t lc[4];          // Load cell values
    uint16_t crc16;         // CRC16 of packet (excluding sync and crc)
};

// ============================================================================
// BLE CONFIGURATION
// ============================================================================

#define BLE_SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define BLE_DATA_CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"
#define BLE_CMD_CHARACTERISTIC_UUID "11111111-2222-3333-4444-555555555555"  // Bidirectional: WRITE + NOTIFY

BLEServer* pServer = nullptr;
BLECharacteristic* pDataCharacteristic = nullptr;  // Sensor data (binary NOTIFY)
BLECharacteristic* pCmdCharacteristic = nullptr;   // Commands (WRITE) + Responses (NOTIFY)
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ============================================================================
// DATA STORE
// ============================================================================

#define QUEUE_SIZE 20  // Buffer for incoming samples

struct SampleData {
    int32_t lc[4];
    uint32_t timestamp;
    uint16_t frame_idx;
    uint8_t sample_idx;
    bool consumed;
};

struct DataStore {
    // Local data queue
    SampleData local_queue[QUEUE_SIZE];
    int local_head;
    int local_tail;
    int local_count;
    
    // Remote data queue
    SampleData remote_queue[QUEUE_SIZE];
    int remote_head;
    int remote_tail;
    int remote_count;
    
    // Latest consumed samples (for fallback)
    int32_t last_local_lc[4];
    int32_t last_remote_lc[4];
    bool has_local_data;
    bool has_remote_data;
};

static DataStore data_store = {0};

// Hardware timer for 1000Hz sampling
hw_timer_t* sampling_timer = nullptr;
static volatile bool timer_sample_ready = false;
static volatile unsigned long timer_interrupts_count = 0;

// Packet rate control
static int packet_rate_counter = 0;

// Data flow tracking
static unsigned long last_data_received_ms = 0;
#define DATA_TIMEOUT_MS 500  // Consider data stale after 500ms (no BLE send)

// ============================================================================
// STATISTICS
// ============================================================================

static unsigned long uart_packets_received = 0;
static unsigned long uart_bytes_received = 0;
static unsigned long uart_crc_errors = 0;
static unsigned long uart_sync_errors = 0;
static unsigned long local_samples_received = 0;
static unsigned long remote_samples_received = 0;
static unsigned long ble_notifications_sent = 0;
static unsigned long ble_send_errors = 0;
static unsigned long timer_samples_generated = 0;
static unsigned long commands_forwarded = 0;
static unsigned long command_responses_received = 0;
static unsigned long command_timeouts = 0;
static unsigned long ble_commands_received = 0;
static unsigned long ble_command_errors = 0;

// Response capture for command forwarding
static volatile bool command_response_ready = false;
static String command_response_data = "";
static SemaphoreHandle_t command_response_mutex = NULL;

// ============================================================================
// BLE TRANSMISSION CONFIGURATION
// ============================================================================

#define BLE_SAMPLE_DECIMATION 1  // Send every sample for maximum throughput

static unsigned long sample_decimation_counter = 0;
static unsigned long samples_batched = 0;
static unsigned long ble_packets_sent = 0;

// Compact 8-channel load cell sample using 16-bit values
struct __attribute__((packed)) CompactLoadCellSample {
    int16_t local_lc[4];    // Local load cells 1-4 (8 bytes)
    int16_t remote_lc[4];   // Remote load cells 5-8 (8 bytes)
};  // Total: 16 bytes per sample

#define SAMPLES_PER_BLE_PACKET 10
struct __attribute__((packed)) OptimizedBLEPacket {
    uint8_t sample_count;
    CompactLoadCellSample samples[SAMPLES_PER_BLE_PACKET];
};  // Total: 1 + (16 * 10) = 161 bytes

static OptimizedBLEPacket current_ble_packet;
static int current_sample_count = 0;

// ============================================================================
// UART PACKET PROCESSING
// ============================================================================

static inline uint16_t calculate_uart_crc(const UartPacket* packet) {
    const uint8_t* data = (const uint8_t*)packet + 2;
    uint32_t length = sizeof(UartPacket) - 4;
    return crc16_ccitt_false(data, length);
}

static inline bool validate_uart_packet(const UartPacket* packet) {
    if (packet->sync[0] != 0xAA || packet->sync[1] != 0x55) {
        uart_sync_errors++;
        return false;
    }
    
    uint16_t expected_crc = calculate_uart_crc(packet);
    if (packet->crc16 != expected_crc) {
        uart_crc_errors++;
        return false;
    }
    
    return true;
}

static inline void process_uart_packet(const UartPacket* packet) {
    if (!validate_uart_packet(packet)) {
        return;
    }
    
    uart_packets_received++;
    uart_bytes_received += sizeof(UartPacket);
    
    switch (packet->type) {
        case 'L':
            local_samples_received++;
            add_to_queue('L', packet->lc, packet->frame_idx, packet->sample_idx);
            break;
            
        case 'R':
            remote_samples_received++;
            add_to_queue('R', packet->lc, packet->frame_idx, packet->sample_idx);
            break;
    }
}

// ============================================================================
// BLE JSON RESPONSE FUNCTIONS
// ============================================================================

// Send JSON response via BLE Command characteristic (bidirectional)
static void send_ble_json_response(const char* json) {
    if (deviceConnected && pCmdCharacteristic != nullptr) {
        pCmdCharacteristic->setValue((uint8_t*)json, strlen(json));
        pCmdCharacteristic->notify();
        Serial.printf("[BLE] -> %s\n", json);
    }
}

// Parse Teensy response and send as JSON via BLE
// Response formats from Teensy: "TARE:OK", "SHOW:LC1=off,a,n;LC2=...", "READ:v1;v2;v3;v4;tot", etc.
static void send_json_response(const char* prefix, const char* response, unsigned long round_trip_ms) {
    char json[300];
    
    // Determine if this is LOCAL or REMOTE
    bool is_remote = (strcmp(prefix, "REMOTE") == 0);
    const char* target = is_remote ? "REMOTE" : "LOCAL";
    
    // Parse the response format: "CMD:DATA" or "CMD:STATUS"
    String resp = String(response);
    int colon_pos = resp.indexOf(':');
    
    if (colon_pos <= 0) {
        // Unknown format - send raw
        snprintf(json, sizeof(json), "{\"target\":\"%s\",\"raw\":\"%s\",\"ms\":%lu}", 
                 target, response, round_trip_ms);
        send_ble_json_response(json);
        return;
    }
    
    String cmd = resp.substring(0, colon_pos);
    String data = resp.substring(colon_pos + 1);
    
    // Handle different command responses
    if (cmd == "TARE" || cmd == "FIT" || cmd == "CLEAR" || cmd == "ADD" || cmd == "ADD_CH") {
        // Simple OK/ERR responses
        bool ok = (data == "OK");
        if (ok) {
            snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"ok\":true,\"ms\":%lu}", 
                     target, cmd.c_str(), round_trip_ms);
        } else {
            snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"ok\":false,\"err\":\"%s\",\"ms\":%lu}", 
                     target, cmd.c_str(), data.c_str(), round_trip_ms);
        }
    }
    else if (cmd == "SHOW") {
        // Format: SHOW:LC1=off,a,n;LC2=off,a,n;LC3=off,a,n;LC4=off,a,n
        // Parse into JSON array
        long off[4] = {0};
        float a[4] = {0};
        int n[4] = {0};
        
        // Parse each LC section
        int lc_idx = 0;
        int start = 0;
        while (lc_idx < 4) {
            int semi = data.indexOf(';', start);
            String lc_data = (semi > 0) ? data.substring(start, semi) : data.substring(start);
            
            // Parse "LC1=off,a,n" - skip "LCx=" prefix
            int eq = lc_data.indexOf('=');
            if (eq > 0) {
                String vals = lc_data.substring(eq + 1);
                int c1 = vals.indexOf(',');
                int c2 = vals.indexOf(',', c1 + 1);
                if (c1 > 0 && c2 > 0) {
                    off[lc_idx] = vals.substring(0, c1).toInt();
                    a[lc_idx] = vals.substring(c1 + 1, c2).toFloat();
                    n[lc_idx] = vals.substring(c2 + 1).toInt();
                }
            }
            
            lc_idx++;
            if (semi < 0) break;
            start = semi + 1;
        }
        
        snprintf(json, sizeof(json), 
                 "{\"target\":\"%s\",\"cmd\":\"SHOW\",\"lc\":[{\"off\":%ld,\"a\":%.6f,\"n\":%d},{\"off\":%ld,\"a\":%.6f,\"n\":%d},{\"off\":%ld,\"a\":%.6f,\"n\":%d},{\"off\":%ld,\"a\":%.6f,\"n\":%d}],\"ms\":%lu}",
                 target, off[0], a[0], n[0], off[1], a[1], n[1], off[2], a[2], n[2], off[3], a[3], n[3], round_trip_ms);
    }
    else if (cmd == "READ" || cmd == "READ_RAW") {
        // Format: READ:v1;v2;v3;v4;tot
        long v[5] = {0};
        int idx = 0;
        int start = 0;
        while (idx < 5) {
            int semi = data.indexOf(';', start);
            String val = (semi > 0) ? data.substring(start, semi) : data.substring(start);
            v[idx] = val.toInt();
            idx++;
            if (semi < 0) break;
            start = semi + 1;
        }
        snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"v\":[%ld,%ld,%ld,%ld,%ld],\"ms\":%lu}", 
                 target, cmd.c_str(), v[0], v[1], v[2], v[3], v[4], round_trip_ms);
    }
    else if (cmd == "PTS") {
        // Format: PTS:n1;n2;n3;n4
        int n[4] = {0};
        int idx = 0;
        int start = 0;
        while (idx < 4) {
            int semi = data.indexOf(';', start);
            String val = (semi > 0) ? data.substring(start, semi) : data.substring(start);
            n[idx] = val.toInt();
            idx++;
            if (semi < 0) break;
            start = semi + 1;
        }
        snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"PTS\",\"n\":[%d,%d,%d,%d],\"ms\":%lu}", 
                 target, n[0], n[1], n[2], n[3], round_trip_ms);
    }
    else if (data == "TIMEOUT" || data == "ESPNOW_ERR") {
        // Error responses
        snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"ok\":false,\"err\":\"%s\",\"ms\":%lu}", 
                 target, cmd.c_str(), data.c_str(), round_trip_ms);
    }
    else {
        // Unknown command - send as generic response
        snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"data\":\"%s\",\"ms\":%lu}", 
                 target, cmd.c_str(), data.c_str(), round_trip_ms);
    }
    
    send_ble_json_response(json);
}

// Send simple status JSON for control commands (START, STOP, etc.)
static void send_control_json_response(const char* target, const char* cmd, bool ok, unsigned long ms) {
    char json[150];
    snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"ok\":%s,\"ms\":%lu}", 
             target, cmd, ok ? "true" : "false", ms);
    send_ble_json_response(json);
}

// Send PING response
static void send_ping_json_response(const char* target, unsigned long ms) {
    char json[100];
    snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"PING\",\"ok\":true,\"ms\":%lu}", target, ms);
    send_ble_json_response(json);
}

// Send timeout error
static void send_timeout_json_response(const char* target, const char* cmd, unsigned long ms) {
    char json[150];
    snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"ok\":false,\"err\":\"TIMEOUT\",\"ms\":%lu}", 
             target, cmd, ms);
    send_ble_json_response(json);
}

// ============================================================================
// BLE COMMAND PROCESSING
// ============================================================================

static void forward_command_to_rx_radio(String command) {
    bool is_ping = (command == "LOCAL_PING" || command == "REMOTE_PING");
    bool is_cal_command = (command.startsWith("LOCAL_CAL_") || command.startsWith("REMOTE_CAL_"));
    bool is_remote = (command.startsWith("REMOTE_") || command.startsWith("ALL_"));
    bool is_local = command.startsWith("LOCAL_") || (!is_remote && !command.startsWith("ALL_"));
    
    // Determine target for JSON response
    const char* target = "LOCAL";
    if (command.startsWith("REMOTE_")) target = "REMOTE";
    else if (command.startsWith("ALL_")) target = "ALL";
    
    // Extract base command (e.g., "START" from "REMOTE_START", "CAL_TARE" from "LOCAL_CAL_TARE")
    String base_cmd = command;
    if (command.startsWith("LOCAL_")) base_cmd = command.substring(6);
    else if (command.startsWith("REMOTE_")) base_cmd = command.substring(7);
    else if (command.startsWith("ALL_")) base_cmd = command.substring(4);
    
    Serial.printf("[BLE_SLAVE] Forwarding '%s' to RX Radio...\n", command.c_str());
        
        // Clear previous response
        if (command_response_mutex != NULL && xSemaphoreTake(command_response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            command_response_ready = false;
            command_response_data = "";
            xSemaphoreGive(command_response_mutex);
        }
        
        unsigned long start_time = millis();
        Serial2.println(command);
        Serial2.flush();
        commands_forwarded++;
        
    // Wait for response (longer timeout for calibration commands)
    unsigned long timeout = millis() + (is_cal_command ? 15000 : 5000);
        bool got_response = false;
        String response_text = "";
        
        while (millis() < timeout) {
            if (command_response_mutex != NULL && xSemaphoreTake(command_response_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (command_response_ready) {
                    response_text = command_response_data;
                    command_response_ready = false;
                    got_response = true;
                    xSemaphoreGive(command_response_mutex);
                    break;
                }
                xSemaphoreGive(command_response_mutex);
            }
            delay(10);
        }
        
        unsigned long round_trip_ms = millis() - start_time;
        
        if (got_response) {
        // Parse prefix (LOCAL: or REMOTE:) from response
        String prefix = "";
                String actual_response = response_text;
                if (response_text.startsWith("LOCAL:")) {
            prefix = "LOCAL";
            actual_response = response_text.substring(6);
                } else if (response_text.startsWith("REMOTE:")) {
            prefix = "REMOTE";
            actual_response = response_text.substring(7);
        }
        
        Serial.printf("[BLE_SLAVE] Response: %s (%lu ms)\n", actual_response.c_str(), round_trip_ms);
            command_responses_received++;
        
        // Send JSON response via BLE
            if (is_ping) {
            send_ping_json_response(target, round_trip_ms);
        } else if (is_cal_command) {
            // Calibration commands - parse the detailed response
            send_json_response(target, actual_response.c_str(), round_trip_ms);
            } else {
            // Control commands (START, STOP, etc.) - check for success indicators
            bool ok = (actual_response.indexOf("OK") >= 0 ||
                       actual_response.indexOf("RUNNING") >= 0 ||
                       actual_response.indexOf("STOPPED") >= 0);
            send_control_json_response(target, base_cmd.c_str(), ok, round_trip_ms);
            }
    } else {
        Serial.printf("[BLE_SLAVE] ✗ TIMEOUT after %lu ms\n", round_trip_ms);
            command_timeouts++;
        
        // Send timeout error via BLE
        send_timeout_json_response(target, base_cmd.c_str(), round_trip_ms);
    }
}

static void process_ble_command(String command) {
    command.trim();
    command.toUpperCase();
    
    if (command.length() == 0) {
        Serial.println("[BLE_SLAVE] ⚠ Empty BLE command received");
        ble_command_errors++;
        return;
    }
    
    Serial.printf("[BLE_SLAVE] BLE Command received: %s\n", command.c_str());
    ble_commands_received++;
    
    // Local commands (handled by BLE Slave)
    if (command == "STATS") {
        show_detailed_stats();
        send_control_json_response("BLE", "STATS", true, 0);
        return;
    }
    
    if (command == "RESET_STATS") {
        reset_all_stats();
        send_control_json_response("BLE", "RESET_STATS", true, 0);
        return;
    }
    
    if (command == "HELP") {
        show_help();
        send_control_json_response("BLE", "HELP", true, 0);
        return;
    }
    
    // Valid commands to forward to ESP32 RX Radio
    if (command == "START" || command == "STOP" || command == "RESTART" || command == "RESET" ||
        command == "REMOTE_START" || command == "REMOTE_STOP" || command == "REMOTE_RESTART" || command == "REMOTE_RESET" ||
        command == "ALL_START" || command == "ALL_STOP" || command == "ALL_RESTART" || command == "ALL_RESET" ||
        command == "LOCAL_PING" || command == "REMOTE_PING" ||
        command.startsWith("LOCAL_CAL_") || command.startsWith("REMOTE_CAL_")) {
        
        forward_command_to_rx_radio(command);
    } else {
        Serial.printf("[BLE_SLAVE] ✗ Unknown BLE command: '%s'\n", command.c_str());
        ble_command_errors++;
        
        // Send error via BLE
        char json[150];
        snprintf(json, sizeof(json), "{\"cmd\":\"%s\",\"ok\":false,\"err\":\"UNKNOWN_CMD\"}", command.c_str());
        send_ble_json_response(json);
    }
}

// ============================================================================
// BLE SERVER CALLBACKS
// ============================================================================

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("[BLE_SLAVE] ✓ BLE Client connected");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("[BLE_SLAVE] ✗ BLE Client disconnected");
    }
};

class MyCommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        String command = pCharacteristic->getValue().c_str();
        
        if (command.length() > 0) {
            Serial.printf("[BLE_SLAVE] BLE Write received: '%s' (%d bytes)\n", command.c_str(), command.length());
            process_ble_command(command);
        }
    }
};

// ============================================================================
// DATA STORE FUNCTIONS
// ============================================================================

static inline void add_to_queue(uint8_t type, const int32_t* lc, uint16_t frame_idx, uint8_t sample_idx) {
    uint32_t now = millis();
    last_data_received_ms = now;  // Track when we last received fresh data
    
    if (type == 'L') {
        if (data_store.local_count < QUEUE_SIZE) {
            SampleData* sample = &data_store.local_queue[data_store.local_tail];
            memcpy(sample->lc, lc, sizeof(sample->lc));
            sample->timestamp = now;
            sample->frame_idx = frame_idx;
            sample->sample_idx = sample_idx;
            sample->consumed = false;
            
            data_store.local_tail = (data_store.local_tail + 1) % QUEUE_SIZE;
            data_store.local_count++;
        } else {
            SampleData* sample = &data_store.local_queue[data_store.local_tail];
            memcpy(sample->lc, lc, sizeof(sample->lc));
            sample->timestamp = now;
            sample->frame_idx = frame_idx;
            sample->sample_idx = sample_idx;
            sample->consumed = false;
            
            data_store.local_tail = (data_store.local_tail + 1) % QUEUE_SIZE;
            data_store.local_head = (data_store.local_head + 1) % QUEUE_SIZE;
        }
    } else if (type == 'R') {
        if (data_store.remote_count < QUEUE_SIZE) {
            SampleData* sample = &data_store.remote_queue[data_store.remote_tail];
            memcpy(sample->lc, lc, sizeof(sample->lc));
            sample->timestamp = now;
            sample->frame_idx = frame_idx;
            sample->sample_idx = sample_idx;
            sample->consumed = false;
            
            data_store.remote_tail = (data_store.remote_tail + 1) % QUEUE_SIZE;
            data_store.remote_count++;
        } else {
            SampleData* sample = &data_store.remote_queue[data_store.remote_tail];
            memcpy(sample->lc, lc, sizeof(sample->lc));
            sample->timestamp = now;
            sample->frame_idx = frame_idx;
            sample->sample_idx = sample_idx;
            sample->consumed = false;
            
            data_store.remote_tail = (data_store.remote_tail + 1) % QUEUE_SIZE;
            data_store.remote_head = (data_store.remote_head + 1) % QUEUE_SIZE;
        }
    }
}

static inline bool get_next_sample(int32_t* local_lc, int32_t* remote_lc) {
    // Check if data flow has stopped (stale data)
    unsigned long now = millis();
    unsigned long age = (last_data_received_ms > 0) ? (now - last_data_received_ms) : 999999;
    
    if (age > DATA_TIMEOUT_MS) {
        // Data flow stopped - reset fallback flags and don't send
        data_store.has_local_data = false;
        data_store.has_remote_data = false;
        return false;
    }
    
    bool got_local = false;
    bool got_remote = false;
    
    // Initialize to zero (missing channels show as 0)
    memset(local_lc, 0, 4 * sizeof(int32_t));
    memset(remote_lc, 0, 4 * sizeof(int32_t));
    
    if (data_store.local_count > 0) {
        SampleData* sample = &data_store.local_queue[data_store.local_head];
        if (!sample->consumed) {
            memcpy(local_lc, sample->lc, sizeof(sample->lc));
            memcpy(data_store.last_local_lc, sample->lc, sizeof(sample->lc));
            sample->consumed = true;
            data_store.has_local_data = true;
            got_local = true;
            
            data_store.local_head = (data_store.local_head + 1) % QUEUE_SIZE;
            data_store.local_count--;
        }
    }
    
    if (data_store.remote_count > 0) {
        SampleData* sample = &data_store.remote_queue[data_store.remote_head];
        if (!sample->consumed) {
            memcpy(remote_lc, sample->lc, sizeof(sample->lc));
            memcpy(data_store.last_remote_lc, sample->lc, sizeof(sample->lc));
            sample->consumed = true;
            data_store.has_remote_data = true;
            got_remote = true;
            
            data_store.remote_head = (data_store.remote_head + 1) % QUEUE_SIZE;
            data_store.remote_count--;
        }
    }
    
    // Use last known data as fallback (only while data is flowing)
    if (!got_local && data_store.has_local_data) {
        memcpy(local_lc, data_store.last_local_lc, sizeof(data_store.last_local_lc));
        got_local = true;
    }
    
    if (!got_remote && data_store.has_remote_data) {
        memcpy(remote_lc, data_store.last_remote_lc, sizeof(data_store.last_remote_lc));
        got_remote = true;
    }
    
    // Return true if we have ANY data (local OR remote)
    return got_local || got_remote;
}

// ============================================================================
// HARDWARE TIMER FUNCTIONS (1000 Hz)
// ============================================================================

void IRAM_ATTR timer_sample_callback() {
    timer_interrupts_count++;
    timer_sample_ready = true;
}

static inline void create_8channel_sample() {
    int32_t local_lc[4];
    int32_t remote_lc[4];
    
    if (!get_next_sample(local_lc, remote_lc)) {
        return;
    }
    
    timer_samples_generated++;
    
    sample_decimation_counter++;
    if (sample_decimation_counter % BLE_SAMPLE_DECIMATION != 0) {
        return;
    }
    
    if (current_sample_count < SAMPLES_PER_BLE_PACKET) {
        CompactLoadCellSample* sample = &current_ble_packet.samples[current_sample_count];
        
        for (int i = 0; i < 4; i++) {
            sample->local_lc[i] = (int16_t)constrain(local_lc[i], -32768, 32767);
            sample->remote_lc[i] = (int16_t)constrain(remote_lc[i], -32768, 32767);
        }
        current_sample_count++;
    }
    
    packet_rate_counter++;
    if (current_sample_count >= SAMPLES_PER_BLE_PACKET || packet_rate_counter >= 10) {
        send_current_batch();
        packet_rate_counter = 0;
    }
}

// ============================================================================
// BATCH SAMPLE PROCESSING
// ============================================================================

static inline void send_current_batch() {
    // Only send if: samples ready AND connected
    // The data flow check in get_next_sample() already prevents sending when no data
    if (current_sample_count > 0 && deviceConnected) {
        current_ble_packet.sample_count = current_sample_count;
        size_t packet_size = 1 + (current_sample_count * sizeof(CompactLoadCellSample));
        
        try {
            pDataCharacteristic->setValue((uint8_t*)&current_ble_packet, packet_size);
            pDataCharacteristic->notify();
            ble_notifications_sent++;
            ble_packets_sent++;
            samples_batched += current_sample_count;
        } catch (...) {
            ble_send_errors++;
        }
        
        current_sample_count = 0;
    }
}

// ============================================================================
// UART READING TASK
// ============================================================================

static void uart_rx_task(void* param) {
    UartPacket packet;
    uint8_t* packet_ptr = (uint8_t*)&packet;
    int bytes_needed = sizeof(UartPacket);
    int bytes_received = 0;
    bool sync_found = false;
    
    Serial.printf("UART RX Task started, expecting %d byte packets\n", bytes_needed);
    
    while (true) {
        int available = Serial2.available();
        if (available > 0) {
            if (!sync_found) {
                uint8_t byte = Serial2.peek();
                
                // Check for text message prefix (##REMOTE: or ##LOCAL:)
                if (byte == '#') {
                    Serial2.read(); // consume the '#'
                    
                    // Check if it's ## (double hash) for calibration responses
                    if (Serial2.peek() == '#') {
                        Serial2.read(); // consume second '#'
                        
                        // Read single-line response (format: "REMOTE:response" or "LOCAL:response")
                        String text_response = Serial2.readStringUntil('\n');
                        text_response.trim();
                     
                     if (text_response.length() > 0) {
                            // Parse prefix and response
                             String prefix = "";
                            String response = text_response;
                            int colon_pos = text_response.indexOf(':');
                            if (colon_pos > 0 && colon_pos < 10) {
                                prefix = text_response.substring(0, colon_pos);
                                response = text_response.substring(colon_pos + 1);
                            }
                            
                            // Display response
                            Serial.printf("[%s] %s\n", prefix.c_str(), response.c_str());
                            
                            // Store for command response callback
                            if (command_response_mutex != NULL && xSemaphoreTake(command_response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                                command_response_data = text_response;
                                command_response_ready = true;
                                xSemaphoreGive(command_response_mutex);
                            }
                        }
                    }
                    bytes_received = 0;
                    continue;
                }
                
                byte = Serial2.read();
                
                if (byte == 0xAA) {
                    packet_ptr[0] = byte;
                    bytes_received = 1;
                } else if (bytes_received == 1 && byte == 0x55) {
                    packet_ptr[1] = byte;
                    bytes_received = 2;
                    sync_found = true;
                } else {
                    bytes_received = 0;
                }
            } else {
                int to_read = min(available, bytes_needed - bytes_received);
                
                if (to_read > 0) {
                    int actual_read = Serial2.readBytes(packet_ptr + bytes_received, to_read);
                    bytes_received += actual_read;
                    
                    if (bytes_received >= bytes_needed) {
                        process_uart_packet(&packet);
                        bytes_received = 0;
                        sync_found = false;
                    }
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

// ============================================================================
// BLE TRANSMISSION TASKS
// ============================================================================

static void timer_processing_task(void* param) {
    while (true) {
        if (timer_sample_ready) {
            timer_sample_ready = false;
            create_8channel_sample();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void ble_tx_task(void* param) {
    while (true) {
        if (deviceConnected) {
            if (current_sample_count > 0) {
                vTaskDelay(pdMS_TO_TICKS(5));
                if (current_sample_count > 0) {
                    send_current_batch();
                }
            }
        } else {
            current_sample_count = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// STATISTICS DISPLAY
// ============================================================================

static void show_detailed_stats() {
    unsigned long uptime_sec = millis() / 1000;
    unsigned long uptime_min = uptime_sec / 60;
    unsigned long uptime_hrs = uptime_min / 60;
    
    Serial.println("\n[BLE_SLAVE] === SYSTEM STATISTICS ===");
    Serial.printf("  Uptime: %02lu:%02lu:%02lu\n", uptime_hrs, uptime_min % 60, uptime_sec % 60);
    Serial.println("  --- UART ---");
    Serial.printf("  Packets: %lu (%lu bytes), CRC errors: %lu, Sync errors: %lu\n", 
                 uart_packets_received, uart_bytes_received, uart_crc_errors, uart_sync_errors);
    Serial.println("  --- Samples ---");
    Serial.printf("  Local: %lu | Remote: %lu | Timer: %lu | Batched: %lu\n", 
                 local_samples_received, remote_samples_received, timer_samples_generated, samples_batched);
    Serial.println("  --- BLE ---");
    Serial.printf("  Status: %s | Notifications: %lu | Packets: %lu | Errors: %lu\n", 
                 deviceConnected ? "CONNECTED" : "DISCONNECTED", ble_notifications_sent, ble_packets_sent, ble_send_errors);
    Serial.println("  --- Commands ---");
    Serial.printf("  BLE cmds: %lu | Forwarded: %lu | Responses: %lu | Timeouts: %lu\n", 
                 ble_commands_received, commands_forwarded, command_responses_received, command_timeouts);
    Serial.println("  --- Queues ---");
    Serial.printf("  Local: %d/%d | Remote: %d/%d | Batch: %d/%d | Heap: %lu bytes\n", 
                 data_store.local_count, QUEUE_SIZE, data_store.remote_count, QUEUE_SIZE, 
                 current_sample_count, SAMPLES_PER_BLE_PACKET, ESP.getFreeHeap());
    Serial.println("=====================================\n");
}

static void reset_all_stats() {
            uart_packets_received = 0;
            uart_bytes_received = 0;
            uart_crc_errors = 0;
            uart_sync_errors = 0;
            local_samples_received = 0;
            remote_samples_received = 0;
            ble_notifications_sent = 0;
            ble_send_errors = 0;
    timer_samples_generated = 0;
            commands_forwarded = 0;
            command_responses_received = 0;
            command_timeouts = 0;
            ble_commands_received = 0;
            ble_command_errors = 0;
    sample_decimation_counter = 0;
    samples_batched = 0;
    ble_packets_sent = 0;
    
    Serial.println("[BLE_SLAVE] ✓ All statistics reset");
}

static void show_help() {
    Serial.println("\n[BLE_SLAVE] === AVAILABLE COMMANDS ===");
    Serial.println("  Control:");
    Serial.println("    START, STOP, RESTART, RESET (local Teensy)");
    Serial.println("    REMOTE_START/STOP/RESTART/RESET (remote Teensy)");
    Serial.println("    ALL_START/STOP/RESTART/RESET (both)");
    Serial.println("  Ping:");
    Serial.println("    LOCAL_PING, REMOTE_PING");
    Serial.println("  Calibration:");
    Serial.println("    LOCAL_CAL_TARE, LOCAL_CAL_SHOW, LOCAL_CAL_ADD_<kg>");
    Serial.println("    LOCAL_CAL_ADD_CH_<ch>_<kg>, LOCAL_CAL_CLEAR, LOCAL_CAL_READ");
    Serial.println("    REMOTE_CAL_TARE, REMOTE_CAL_SHOW, etc.");
    Serial.println("  Stats:");
    Serial.println("    STATS, RESET_STATS, HELP");
    Serial.println("\n=== BLE CHARACTERISTICS ===");
    Serial.println("  Service: 12345678-1234-1234-1234-123456789abc");
    Serial.println("  Data:    87654321-... (NOTIFY binary sensor data)");
    Serial.println("  Cmd:     11111111-... (WRITE cmd, NOTIFY JSON response)");
    Serial.println("=====================================\n");
}

// ============================================================================
// SERIAL COMMANDS
// ============================================================================

static void handle_serial_commands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if (command.length() == 0) return;
        
        Serial.printf("[BLE_SLAVE] Command received: %s\n", command.c_str());
        
        // Forward control commands to ESP32 RX Radio
        if (command == "START" || command == "STOP" || command == "RESTART" || command == "RESET" ||
            command == "REMOTE_START" || command == "REMOTE_STOP" || command == "REMOTE_RESTART" || command == "REMOTE_RESET" ||
            command == "ALL_START" || command == "ALL_STOP" || command == "ALL_RESTART" || command == "ALL_RESET" ||
            command == "LOCAL_PING" || command == "REMOTE_PING" ||
            command.startsWith("LOCAL_CAL_") || command.startsWith("REMOTE_CAL_")) {
            
            forward_command_to_rx_radio(command);
        }
        else if (command == "STATS") {
            show_detailed_stats();
        }
        else if (command == "RESET_STATS") {
            reset_all_stats();
        }
        else if (command == "HELP") {
            show_help();
        } else {
            Serial.printf("[BLE_SLAVE] ✗ Unknown command: '%s'\n", command.c_str());
            Serial.println("[BLE_SLAVE] Type 'HELP' for available commands");
        }
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(921600);
    delay(100);
    
    command_response_mutex = xSemaphoreCreateMutex();
    if (command_response_mutex == NULL) {
        Serial.println("[BLE_SLAVE] ⚠ Failed to create mutex!");
    }
    
    Serial.println("\n[BLE_SLAVE] ESP32 BLE Slave - Load Cell Data Forwarder");
    Serial.println("[BLE_SLAVE] BLE Service: 12345678-1234-1234-1234-123456789abc");
    
    // Initialize UART2
    Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial2.setRxBufferSize(8192);
    Serial.printf("UART2 initialized at %d bps\n", UART_BAUD_RATE);
    
    // Initialize BLE
    BLEDevice::init("LoadCell_BLE_Server");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
    
    // Data characteristic - binary sensor data (NOTIFY)
    pDataCharacteristic = pService->createCharacteristic(
                        BLE_DATA_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pDataCharacteristic->addDescriptor(new BLE2902());
    
    // Command characteristic - bidirectional (WRITE commands, NOTIFY responses)
    pCmdCharacteristic = pService->createCharacteristic(
                        BLE_CMD_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_WRITE_NR |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pCmdCharacteristic->addDescriptor(new BLE2902());
    pCmdCharacteristic->setCallbacks(new MyCommandCallbacks());
    
    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
    
    Serial.println("[BLE_SLAVE] BLE server started, waiting for connections...");
    
    // Initialize hardware timer for 1000Hz sampling
    sampling_timer = timerBegin(1000000);
    timerAttachInterrupt(sampling_timer, &timer_sample_callback);
    timerAlarm(sampling_timer, 1000, true, 0);
    
    // Start tasks
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 8192, NULL, 23, NULL, 0);
    xTaskCreatePinnedToCore(timer_processing_task, "timer_proc", 4096, NULL, 24, NULL, 1);
    xTaskCreatePinnedToCore(ble_tx_task, "ble_tx", 4096, NULL, 22, NULL, 1);
    
    Serial.println("[BLE_SLAVE] ✓ System ready (type HELP for commands)");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    static unsigned long last_stats_time = 0;
    static unsigned long last_local_samples = 0;
    static unsigned long last_remote_samples = 0;
    static unsigned long last_ble_packets = 0;
    static unsigned long last_uart_packets = 0;
    static unsigned long last_samples_batched = 0;
    
    unsigned long now = millis();
    
    // Print live stats every 5 seconds
    if (now - last_stats_time >= 5000) {
        // Calculate rates (per second over 5 second window)
        float local_sps = (local_samples_received - last_local_samples) / 5.0f;
        float remote_sps = (remote_samples_received - last_remote_samples) / 5.0f;
        float ble_pps = (ble_packets_sent - last_ble_packets) / 5.0f;
        float uart_pps = (uart_packets_received - last_uart_packets) / 5.0f;
        float batch_sps = (samples_batched - last_samples_batched) / 5.0f;
        
        // Check data flow status
        unsigned long data_age = (last_data_received_ms > 0) ? (now - last_data_received_ms) : 999999;
        bool data_flowing = (data_age < DATA_TIMEOUT_MS);
        
        Serial.printf("📊 LIVE | L: %.0f | R: %.0f | UART: %.0f | BLE: %s %.0f (%.0f) | Q: L=%d R=%d | Flow: %s\n",
                     local_sps, remote_sps, uart_pps,
                     deviceConnected ? "✓" : "✗", ble_pps, batch_sps,
                     data_store.local_count, data_store.remote_count,
                     data_flowing ? "ON" : "OFF");
        
        // Save current values for next calculation
        last_stats_time = now;
        last_local_samples = local_samples_received;
        last_remote_samples = remote_samples_received;
        last_ble_packets = ble_packets_sent;
        last_uart_packets = uart_packets_received;
        last_samples_batched = samples_batched;
    }
    
    // Handle BLE connection changes
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
    delay(100);
}
