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
#define BLE_COMMAND_CHARACTERISTIC_UUID "11111111-2222-3333-4444-555555555555"

BLEServer* pServer = nullptr;
BLECharacteristic* pDataCharacteristic = nullptr;    // For sending sensor data
BLECharacteristic* pCommandCharacteristic = nullptr; // For receiving commands
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
// BLE COMMAND PROCESSING
// ============================================================================

static void forward_command_to_rx_radio(String command) {
    // Special formatting for PING commands
    bool is_local_ping = (command == "LOCAL_PING");
    bool is_remote_ping = (command == "REMOTE_PING");
    bool is_ping = (is_local_ping || is_remote_ping);
    
    if (is_ping) {
        Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
        if (is_local_ping) {
            Serial.println("║              LOCAL PING-PONG TEST                              ║");
        } else {
            Serial.println("║              REMOTE PING-PONG TEST                             ║");
        }
        Serial.println("╚════════════════════════════════════════════════════════════════╝");
        Serial.printf("  📤 SENDING: %s\n", command.c_str());
        if (is_local_ping) {
            Serial.println("  Path: BLE Slave → RX Radio → Local Teensy");
        } else {
            Serial.println("  Path: BLE Slave → RX Radio → SPI Slave → Remote Teensy");
        }
        Serial.println("  ──────────────────────────────────────────────────────────────");
    } else {
        Serial.printf("[BLE_SLAVE] Forwarding '%s' to ESP32 RX Radio...\n", command.c_str());
    }
    
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
    
    // Wait for response from ESP32 RX Radio
    unsigned long timeout = millis() + 5000;
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
        if (is_ping) {
            String source_indicator = "UNKNOWN";
            String actual_response = response_text;
            
            if (response_text.startsWith("LOCAL:")) {
                source_indicator = "🟢 LOCAL TEENSY";
                actual_response = response_text.substring(6);
            } else if (response_text.startsWith("REMOTE:")) {
                source_indicator = "🔵 REMOTE TEENSY";
                actual_response = response_text.substring(7);
            }
            
            Serial.printf("  📥 RECEIVED: %s (from %s)\n", actual_response.c_str(), source_indicator.c_str());
            Serial.printf("  ⏱️  Round-trip time: %lu ms\n", round_trip_ms);
            Serial.println("  ──────────────────────────────────────────────────────────────");
            Serial.println("  ✅ PING-PONG TEST SUCCESS!");
            Serial.println("╚════════════════════════════════════════════════════════════════╝\n");
        } else {
            Serial.printf("[BLE_SLAVE] Response: %s\n", response_text.c_str());
        }
        command_responses_received++;
    } else {
        if (is_ping) {
            Serial.printf("  ⏱️  Timeout after: %lu ms\n", round_trip_ms);
            Serial.println("  ──────────────────────────────────────────────────────────────");
            Serial.println("  ❌ PING-PONG TEST FAILED - No response received");
            Serial.println("╚════════════════════════════════════════════════════════════════╝\n");
        } else {
            Serial.println("[BLE_SLAVE] ⚠ No response from ESP32 RX Radio");
        }
        command_timeouts++;
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
        return;
    }
    
    if (command == "RESET_STATS") {
        reset_all_stats();
        return;
    }
    
    // Valid commands to forward to ESP32 RX Radio
    if (command == "START" || command == "STOP" || command == "RESTART" || command == "RESET" ||
        command == "REMOTE_START" || command == "REMOTE_STOP" || command == "REMOTE_RESTART" || command == "REMOTE_RESET" ||
        command == "ALL_START" || command == "ALL_STOP" || command == "ALL_RESTART" || command == "ALL_RESET" ||
        command == "LOCAL_PING" || command == "REMOTE_PING") {
        
        forward_command_to_rx_radio(command);
    } else {
        Serial.printf("[BLE_SLAVE] ✗ Unknown BLE command: '%s'\n", command.c_str());
        ble_command_errors++;
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
    bool got_local = false;
    bool got_remote = false;
    
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
    
    // Use last known data as fallback
    if (!got_local && data_store.has_local_data) {
        memcpy(local_lc, data_store.last_local_lc, sizeof(data_store.last_local_lc));
        got_local = true;
    }
    
    if (!got_remote && data_store.has_remote_data) {
        memcpy(remote_lc, data_store.last_remote_lc, sizeof(data_store.last_remote_lc));
        got_remote = true;
    }
    
    return got_local && got_remote;
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
                
                // Check for text message prefix
                if (byte == '#') {
                    String text_response = Serial2.readStringUntil('\n');
                    
                    if (text_response.startsWith("##")) {
                        text_response = text_response.substring(2);
                    } else if (text_response.startsWith("#")) {
                        text_response = text_response.substring(1);
                    }
                    
                    if (text_response.length() > 0) {
                        Serial.printf("[BLE_SLAVE] Text response: %s\n", text_response.c_str());
                        
                        if (command_response_mutex != NULL && xSemaphoreTake(command_response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            command_response_data = text_response;
                            command_response_ready = true;
                            xSemaphoreGive(command_response_mutex);
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
    
    Serial.println("\n╔══════════════════════════════════════════════════════════════════════════════╗");
    Serial.println("║                              📊 SYSTEM STATISTICS                            ║");
    Serial.println("╠══════════════════════════════════════════════════════════════════════════════╣");
    
    Serial.printf("║  ⏱️  Uptime: %02lu:%02lu:%02lu                                                        ║\n",
                 uptime_hrs, uptime_min % 60, uptime_sec % 60);
    Serial.println("║                                                                              ║");
    
    Serial.println("║  📡 UART Statistics                                                         ║");
    Serial.println("║  ────────────────────────────────────────────────────────────────────────  ║");
    Serial.printf("║    Packets received:     %-12lu                                       ║\n", uart_packets_received);
    Serial.printf("║    Bytes received:       %-12lu                                       ║\n", uart_bytes_received);
    Serial.printf("║    CRC errors:           %-12lu                                       ║\n", uart_crc_errors);
    Serial.printf("║    Sync errors:          %-12lu                                       ║\n", uart_sync_errors);
    Serial.println("║                                                                              ║");
    
    Serial.println("║  📦 Sample Statistics                                                       ║");
    Serial.println("║  ────────────────────────────────────────────────────────────────────────  ║");
    Serial.printf("║    Local samples:        %-12lu                                       ║\n", local_samples_received);
    Serial.printf("║    Remote samples:       %-12lu                                       ║\n", remote_samples_received);
    Serial.printf("║    Timer samples:        %-12lu                                       ║\n", timer_samples_generated);
    Serial.printf("║    Samples batched:      %-12lu                                       ║\n", samples_batched);
    Serial.println("║                                                                              ║");
    
    Serial.println("║  📶 BLE Statistics                                                          ║");
    Serial.println("║  ────────────────────────────────────────────────────────────────────────  ║");
    Serial.printf("║    Connection:           %-12s                                       ║\n", deviceConnected ? "CONNECTED" : "DISCONNECTED");
    Serial.printf("║    Notifications sent:   %-12lu                                       ║\n", ble_notifications_sent);
    Serial.printf("║    Packets sent:         %-12lu                                       ║\n", ble_packets_sent);
    Serial.printf("║    Send errors:          %-12lu                                       ║\n", ble_send_errors);
    Serial.println("║                                                                              ║");
    
    Serial.println("║  🎮 Command Statistics                                                      ║");
    Serial.println("║  ────────────────────────────────────────────────────────────────────────  ║");
    Serial.printf("║    BLE commands recv:    %-12lu                                       ║\n", ble_commands_received);
    Serial.printf("║    BLE command errors:   %-12lu                                       ║\n", ble_command_errors);
    Serial.printf("║    Commands forwarded:   %-12lu                                       ║\n", commands_forwarded);
    Serial.printf("║    Responses received:   %-12lu                                       ║\n", command_responses_received);
    Serial.printf("║    Command timeouts:     %-12lu                                       ║\n", command_timeouts);
    Serial.println("║                                                                              ║");
    
    Serial.println("║  📋 Queue Status                                                            ║");
    Serial.println("║  ────────────────────────────────────────────────────────────────────────  ║");
    Serial.printf("║    Local queue:          %d / %d                                            ║\n", data_store.local_count, QUEUE_SIZE);
    Serial.printf("║    Remote queue:         %d / %d                                            ║\n", data_store.remote_count, QUEUE_SIZE);
    Serial.printf("║    Current batch:        %d / %d                                            ║\n", current_sample_count, SAMPLES_PER_BLE_PACKET);
    Serial.println("║                                                                              ║");
    
    Serial.printf("║    Free heap:            %-12lu bytes                                 ║\n", ESP.getFreeHeap());
    
    Serial.println("╚══════════════════════════════════════════════════════════════════════════════╝\n");
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
            command == "LOCAL_PING" || command == "REMOTE_PING") {
            
            forward_command_to_rx_radio(command);
        }
        else if (command == "STATS") {
            show_detailed_stats();
        }
        else if (command == "RESET_STATS") {
            reset_all_stats();
        }
        else if (command == "HELP") {
            Serial.println("\n");
            Serial.println("╔══════════════════════════════════════════════════════════════════════════════╗");
            Serial.println("║                              📖 AVAILABLE COMMANDS                          ║");
            Serial.println("╠══════════════════════════════════════════════════════════════════════════════╣");
            Serial.println("║                                                                              ║");
            Serial.println("║  🟢 LOCAL TEENSY CONTROL                                                    ║");
            Serial.println("║  ────────────────────────────────────────────────────────────────────────  ║");
            Serial.println("║  • START             - Start local force plate data collection              ║");
            Serial.println("║  • STOP              - Stop local force plate data collection               ║");
            Serial.println("║  • RESTART           - Restart local force plate data collection            ║");
            Serial.println("║  • RESET             - Reset the local force plate system                   ║");
            Serial.println("║                                                                              ║");
            Serial.println("║  🔵 REMOTE TEENSY CONTROL                                                   ║");
            Serial.println("║  ────────────────────────────────────────────────────────────────────────  ║");
            Serial.println("║  • REMOTE_START      - Start remote force plate data collection             ║");
            Serial.println("║  • REMOTE_STOP       - Stop remote force plate data collection              ║");
            Serial.println("║  • REMOTE_RESTART    - Restart remote force plate data collection           ║");
            Serial.println("║  • REMOTE_RESET      - Reset the remote force plate system                  ║");
            Serial.println("║                                                                              ║");
            Serial.println("║  🔄 DUAL CONTROL (Both Plates)                                              ║");
            Serial.println("║  ────────────────────────────────────────────────────────────────────────  ║");
            Serial.println("║  • ALL_START         - Start both force plates                              ║");
            Serial.println("║  • ALL_STOP          - Stop both force plates                               ║");
            Serial.println("║  • ALL_RESTART       - Restart both force plates                            ║");
            Serial.println("║  • ALL_RESET         - Reset both force plate systems                       ║");
            Serial.println("║                                                                              ║");
            Serial.println("║  🔍 TESTING                                                                 ║");
            Serial.println("║  ────────────────────────────────────────────────────────────────────────  ║");
            Serial.println("║  • LOCAL_PING        - Test connection to local force plate                 ║");
            Serial.println("║  • REMOTE_PING       - Test connection to remote force plate                ║");
            Serial.println("║                                                                              ║");
            Serial.println("║  📊 STATISTICS                                                              ║");
            Serial.println("║  ────────────────────────────────────────────────────────────────────────  ║");
            Serial.println("║  • STATS             - Show detailed system statistics                      ║");
            Serial.println("║  • RESET_STATS       - Reset all statistics counters                        ║");
            Serial.println("║                                                                              ║");
            Serial.println("╚══════════════════════════════════════════════════════════════════════════════╝");
            Serial.println();
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
    
    Serial.println("ESP32 BLE Slave - Load Cell Data Forwarder + Command Gateway");
    Serial.println("=== BLE COMMAND INTERFACE ===");
    Serial.println("BLE Service: 12345678-1234-1234-1234-123456789abc");
    Serial.println("Data (Notify): 87654321-4321-4321-4321-cba987654321");
    Serial.println("Command (Write): 11111111-2222-3333-4444-555555555555");
    Serial.println();
    Serial.println("Commands: START, STOP, RESTART, RESET");
    Serial.println("          REMOTE_START, REMOTE_STOP, REMOTE_RESTART, REMOTE_RESET");
    Serial.println("          ALL_START, ALL_STOP, ALL_RESTART, ALL_RESET");
    Serial.println("          LOCAL_PING, REMOTE_PING, STATS, RESET_STATS, HELP");
    Serial.println("==============================");
    
    // Initialize UART2
    Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial2.setRxBufferSize(8192);
    Serial.printf("UART2 initialized at %d bps\n", UART_BAUD_RATE);
    
    // Initialize BLE
    BLEDevice::init("LoadCell_BLE_Server");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
    
    pDataCharacteristic = pService->createCharacteristic(
                        BLE_DATA_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pDataCharacteristic->addDescriptor(new BLE2902());
    
    pCommandCharacteristic = pService->createCharacteristic(
                        BLE_COMMAND_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_WRITE_NR
                      );
    pCommandCharacteristic->setCallbacks(new MyCommandCallbacks());
    
    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
    
    Serial.println("BLE server started, waiting for connections...");
    
    // Initialize hardware timer for 1000Hz sampling
    sampling_timer = timerBegin(1000000);
    timerAttachInterrupt(sampling_timer, &timer_sample_callback);
    timerAlarm(sampling_timer, 1000, true, 0);
    
    // Start tasks
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 8192, NULL, 23, NULL, 0);
    xTaskCreatePinnedToCore(timer_processing_task, "timer_proc", 4096, NULL, 24, NULL, 1);
    xTaskCreatePinnedToCore(ble_tx_task, "ble_tx", 4096, NULL, 22, NULL, 1);
    
    Serial.println("System ready");
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
        
        Serial.printf("📊 LIVE | Local: %.0f sps | Remote: %.0f sps | UART: %.0f pps | BLE: %s %.0f pps (%.0f sps) | Q: L=%d R=%d\n",
                     local_sps, remote_sps, uart_pps,
                     deviceConnected ? "✓" : "✗", ble_pps, batch_sps,
                     data_store.local_count, data_store.remote_count);
        
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
