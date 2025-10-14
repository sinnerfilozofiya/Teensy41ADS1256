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

// RGB LED pin
#define RGB_LED_PIN 38          // GPIO38 for WS2812B RGB LED status indicator

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
// REDIS-LIKE DATA STORE
// ============================================================================

// Redis-like data store with queues to prevent duplicates
#define REDIS_QUEUE_SIZE 20  // Buffer for incoming samples

struct SampleData {
    int32_t lc[4];
    uint32_t timestamp;
    uint16_t frame_idx;
    uint8_t sample_idx;
    bool consumed;  // Flag to prevent duplicate reads
};

struct RedisStore {
    // Local data queue
    SampleData local_queue[REDIS_QUEUE_SIZE];
    int local_head;
    int local_tail;
    int local_count;
    
    // Remote data queue
    SampleData remote_queue[REDIS_QUEUE_SIZE];
    int remote_head;
    int remote_tail;
    int remote_count;
    
    // Latest consumed samples (for fallback)
    int32_t last_local_lc[4];
    int32_t last_remote_lc[4];
    bool has_local_data;
    bool has_remote_data;
};

static RedisStore redis_store = {0};

// Hardware timer for 1000Hz sampling
hw_timer_t* sampling_timer = nullptr;
static volatile bool timer_sample_ready = false;
static volatile unsigned long timer_interrupts_count = 0;

// OPTIMIZATION: Packet rate control (100 packets/sec instead of 200)
static int packet_rate_counter = 0;

// ============================================================================
// DATA BUFFERING AND STATISTICS
// ============================================================================

// Statistics
static unsigned long uart_packets_received = 0;
static unsigned long uart_bytes_received = 0;
static unsigned long uart_crc_errors = 0;
static unsigned long uart_sync_errors = 0;
static unsigned long local_samples_received = 0;
static unsigned long remote_samples_received = 0;
static unsigned long combined_samples_received = 0;
static unsigned long ble_notifications_sent = 0;
static unsigned long ble_send_errors = 0;
static unsigned long timer_samples_generated = 0;

// Command forwarding statistics
static unsigned long commands_forwarded = 0;
static unsigned long command_responses_received = 0;
static unsigned long command_timeouts = 0;

// BLE command statistics
static unsigned long ble_commands_received = 0;
static unsigned long ble_command_errors = 0;

// Response capture for command forwarding
static volatile bool command_response_ready = false;
static String command_response_data = "";
static SemaphoreHandle_t command_response_mutex = NULL;

// BLE transmission queue - Optimized for maximum throughput
#define BLE_QUEUE_SIZE 200   // Smaller queue to prevent Windows BLE overflow
#define BLE_SAMPLE_DECIMATION 1  // Send every sample for maximum throughput

// Forward declarations
struct CompactLoadCellSample;
struct OptimizedBLEPacket;

// Statistics
static unsigned long sample_decimation_counter = 0;
static unsigned long samples_decimated = 0;
static unsigned long samples_batched = 0;
static unsigned long ble_packets_sent = 0;

// OPTIMIZED: Compact 8-channel load cell sample using 16-bit values
// Assumes load cell values fit in 16-bit range (-32768 to +32767)
// This reduces each sample from 32 bytes to 16 bytes (50% reduction!)
struct __attribute__((packed)) CompactLoadCellSample {
    int16_t local_lc[4];    // Local load cells 1-4 (8 bytes)
    int16_t remote_lc[4];   // Remote load cells 5-8 (8 bytes)
};  // Total: 16 bytes per sample (was 32 bytes)

// OPTIMIZED: More samples per packet with smaller sample size
#define SAMPLES_PER_BLE_PACKET 10  // 10 samples per packet = 160 bytes + header
struct __attribute__((packed)) OptimizedBLEPacket {
    uint8_t sample_count;                                    // Number of samples in this packet (1 byte)
    CompactLoadCellSample samples[SAMPLES_PER_BLE_PACKET];   // Array of compact 8-channel samples
};  // Total: 1 + (16 * 10) = 161 bytes (same size but 2x more samples!)

// Batch transmission system (after struct definitions)
static OptimizedBLEPacket current_ble_packet;
static int current_sample_count = 0;

// ============================================================================
// UART PACKET PROCESSING
// ============================================================================

static inline uint16_t calculate_uart_crc(const UartPacket* packet) {
    // CRC of everything except sync bytes and crc field
    const uint8_t* data = (const uint8_t*)packet + 2; // Skip sync bytes
    uint32_t length = sizeof(UartPacket) - 4; // Exclude sync and crc
    return crc16_ccitt_false(data, length);
}

static inline bool validate_uart_packet(const UartPacket* packet) {
    // Check sync bytes
    if (packet->sync[0] != 0xAA || packet->sync[1] != 0x55) {
        uart_sync_errors++;
        return false;
    }
    
    // Check CRC
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
    
    // Update Redis-like data store
    switch (packet->type) {
        case 'L':
            local_samples_received++;
            add_to_redis_queue('L', packet->lc, packet->frame_idx, packet->sample_idx);
            break;
            
        case 'R':
            remote_samples_received++;
            add_to_redis_queue('R', packet->lc, packet->frame_idx, packet->sample_idx);
            break;
            
        case 'C':
            combined_samples_received++;
            // Skip combined packets - we handle combining via timer
            break;
    }
}

// ============================================================================
// BLE COMMAND PROCESSING
// ============================================================================

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
    
    // Forward Teensy control commands to ESP32 RX Radio
    if (command == "START" || command == "STOP" || command == "RESTART" || command == "RESET" ||
        command == "REMOTE_START" || command == "REMOTE_STOP" || command == "REMOTE_RESTART" || command == "REMOTE_RESET" ||
        command == "ALL_START" || command == "ALL_STOP" || command == "ALL_RESTART" || command == "ALL_RESET" ||
        command == "ZERO" || command == "ZERO_STATUS" || command == "ZERO_RESET" ||
        command == "REMOTE_ZERO" || command == "REMOTE_ZERO_STATUS" || command == "REMOTE_ZERO_RESET" ||
        command == "ALL_ZERO" || command == "ALL_ZERO_STATUS" || command == "ALL_ZERO_RESET" ||
        command == "LOCAL_ON" || command == "LOCAL_OFF" || command == "REMOTE_ON" || command == "REMOTE_OFF" ||
        command == "STATUS" || command == "REMOTE_PING") {
        
        // Special formatting for REMOTE_PING
        bool is_ping = (command == "REMOTE_PING");
        if (is_ping) {
            Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
            Serial.println("║      REMOTE PING-PONG TEST - BLE SLAVE (via BLE)              ║");
            Serial.println("╚════════════════════════════════════════════════════════════════╝");
            Serial.printf("  📤 SENDING: %s\n", command.c_str());
            Serial.println("  Path: BLE Slave → RX Radio → SPI Slave → Remote Teensy");
            Serial.println("  ──────────────────────────────────────────────────────────────");
        } else {
            Serial.printf("[BLE_SLAVE] Forwarding BLE command '%s' to ESP32 RX Radio...\n", command.c_str());
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
        
        // Wait for response from ESP32 RX Radio (captured by UART RX task)
        bool is_zero_command = (command.indexOf("ZERO") >= 0);
        unsigned long timeout_ms = is_zero_command ? 15000 : 5000; // 15s for ZERO, 5s for others
        unsigned long timeout = millis() + timeout_ms;
        bool got_response = false;
        String response_text = "";
        
        while (millis() < timeout) {
            // Check if UART RX task captured a response
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
                Serial.printf("  📥 RECEIVED: %s\n", response_text.c_str());
                Serial.printf("  ⏱️  Round-trip time: %lu ms\n", round_trip_ms);
                Serial.println("  ──────────────────────────────────────────────────────────────");
                Serial.println("  ✅ PING-PONG TEST SUCCESS!");
                Serial.println("╚════════════════════════════════════════════════════════════════╝\n");
            } else {
                Serial.printf("[BLE_SLAVE] RX Radio Response: %s\n", response_text.c_str());
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
// REDIS-LIKE DATA STORE FUNCTIONS
// ============================================================================

static inline void add_to_redis_queue(uint8_t type, const int32_t* lc, uint16_t frame_idx, uint8_t sample_idx) {
    uint32_t now = millis();
    
    // Debug: Print first few samples to check for duplicates
    static int debug_count = 0;
    if (debug_count < 10) {
        Serial.printf("DEBUG %c: [%ld, %ld, %ld, %ld] frame=%d sample=%d\n", 
                     type, lc[0], lc[1], lc[2], lc[3], frame_idx, sample_idx);
        debug_count++;
    }
    
    if (type == 'L') {
        // Add to local queue
        if (redis_store.local_count < REDIS_QUEUE_SIZE) {
            SampleData* sample = &redis_store.local_queue[redis_store.local_tail];
            memcpy(sample->lc, lc, sizeof(sample->lc));
            sample->timestamp = now;
            sample->frame_idx = frame_idx;
            sample->sample_idx = sample_idx;
            sample->consumed = false;
            
            redis_store.local_tail = (redis_store.local_tail + 1) % REDIS_QUEUE_SIZE;
            redis_store.local_count++;
        } else {
            // Queue full - overwrite oldest (move head forward)
            SampleData* sample = &redis_store.local_queue[redis_store.local_tail];
            memcpy(sample->lc, lc, sizeof(sample->lc));
            sample->timestamp = now;
            sample->frame_idx = frame_idx;
            sample->sample_idx = sample_idx;
            sample->consumed = false;
            
            redis_store.local_tail = (redis_store.local_tail + 1) % REDIS_QUEUE_SIZE;
            redis_store.local_head = (redis_store.local_head + 1) % REDIS_QUEUE_SIZE;
        }
    } else if (type == 'R') {
        // Add to remote queue
        if (redis_store.remote_count < REDIS_QUEUE_SIZE) {
            SampleData* sample = &redis_store.remote_queue[redis_store.remote_tail];
            memcpy(sample->lc, lc, sizeof(sample->lc));
            sample->timestamp = now;
            sample->frame_idx = frame_idx;
            sample->sample_idx = sample_idx;
            sample->consumed = false;
            
            redis_store.remote_tail = (redis_store.remote_tail + 1) % REDIS_QUEUE_SIZE;
            redis_store.remote_count++;
        } else {
            // Queue full - overwrite oldest (move head forward)
            SampleData* sample = &redis_store.remote_queue[redis_store.remote_tail];
            memcpy(sample->lc, lc, sizeof(sample->lc));
            sample->timestamp = now;
            sample->frame_idx = frame_idx;
            sample->sample_idx = sample_idx;
            sample->consumed = false;
            
            redis_store.remote_tail = (redis_store.remote_tail + 1) % REDIS_QUEUE_SIZE;
            redis_store.remote_head = (redis_store.remote_head + 1) % REDIS_QUEUE_SIZE;
        }
    }
}

static inline bool get_next_redis_sample(int32_t* local_lc, int32_t* remote_lc) {
    bool got_local = false;
    bool got_remote = false;
    
    // Try to get unconsumed local sample
    if (redis_store.local_count > 0) {
        SampleData* sample = &redis_store.local_queue[redis_store.local_head];
        if (!sample->consumed) {
            memcpy(local_lc, sample->lc, sizeof(sample->lc));
            memcpy(redis_store.last_local_lc, sample->lc, sizeof(sample->lc));
            sample->consumed = true;
            redis_store.has_local_data = true;
            got_local = true;
            
            // Remove consumed sample from queue
            redis_store.local_head = (redis_store.local_head + 1) % REDIS_QUEUE_SIZE;
            redis_store.local_count--;
        }
    }
    
    // Try to get unconsumed remote sample
    if (redis_store.remote_count > 0) {
        SampleData* sample = &redis_store.remote_queue[redis_store.remote_head];
        if (!sample->consumed) {
            memcpy(remote_lc, sample->lc, sizeof(sample->lc));
            memcpy(redis_store.last_remote_lc, sample->lc, sizeof(sample->lc));
            sample->consumed = true;
            redis_store.has_remote_data = true;
            got_remote = true;
            
            // Remove consumed sample from queue
            redis_store.remote_head = (redis_store.remote_head + 1) % REDIS_QUEUE_SIZE;
            redis_store.remote_count--;
        }
    }
    
    // If we didn't get new samples, use last known data (fallback)
    if (!got_local && redis_store.has_local_data) {
        memcpy(local_lc, redis_store.last_local_lc, sizeof(redis_store.last_local_lc));
        got_local = true;
    }
    
    if (!got_remote && redis_store.has_remote_data) {
        memcpy(remote_lc, redis_store.last_remote_lc, sizeof(redis_store.last_remote_lc));
        got_remote = true;
    }
    
    return got_local && got_remote;
}

// ============================================================================
// HARDWARE TIMER FUNCTIONS (1000 Hz)
// ============================================================================

// Timer interrupt handler - MUST be in IRAM
void IRAM_ATTR timer_sample_callback() {
    timer_interrupts_count++;
    timer_sample_ready = true;
}

static inline void create_8channel_sample_from_redis() {
    int32_t local_lc[4];
    int32_t remote_lc[4];
    
    // Try to get next unique sample from Redis queues
    if (!get_next_redis_sample(local_lc, remote_lc)) {
        return; // Skip if we don't have both data sources
    }
    
    timer_samples_generated++;
    
    // Apply decimation
    sample_decimation_counter++;
    if (sample_decimation_counter % BLE_SAMPLE_DECIMATION != 0) {
        samples_decimated++;
        return;
    }
    
    // Add 8-channel sample to current batch (OPTIMIZED: Convert int32 to int16)
    if (current_sample_count < SAMPLES_PER_BLE_PACKET) {
        CompactLoadCellSample* sample = &current_ble_packet.samples[current_sample_count];
        
        // Convert int32_t to int16_t (with saturation to prevent overflow)
        for (int i = 0; i < 4; i++) {
            // Clamp to int16_t range (-32768 to 32767)
            sample->local_lc[i] = (int16_t)constrain(local_lc[i], -32768, 32767);
            sample->remote_lc[i] = (int16_t)constrain(remote_lc[i], -32768, 32767);
        }
        current_sample_count++;
    }
    
    // OPTIMIZATION: Send batch when full OR force send every 10 samples for 100 packets/sec
    packet_rate_counter++;
    if (current_sample_count >= SAMPLES_PER_BLE_PACKET || packet_rate_counter >= 10) {
        send_current_batch();
        packet_rate_counter = 0;  // Reset counter after sending
    }
}

// ============================================================================
// BATCH SAMPLE PROCESSING
// ============================================================================

static inline void send_current_batch() {
    if (current_sample_count > 0 && deviceConnected) {
        current_ble_packet.sample_count = current_sample_count;
        
        // Calculate packet size based on actual sample count (OPTIMIZED: Smaller samples)
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
        
        current_sample_count = 0; // Reset for next batch
    }
}

// Old queue system removed - now using batch transmission

// BLE transmission now handled by batch system in send_current_batch()

// ============================================================================
// UART READING TASK
// ============================================================================

static void uart_rx_task(void* param) {
    UartPacket packet;
    uint8_t* packet_ptr = (uint8_t*)&packet;
    int bytes_needed = sizeof(UartPacket);
    int bytes_received = 0;
    bool sync_found = false;
    
    static unsigned long last_debug_time = 0;
    static unsigned long debug_bytes_count = 0;
    static unsigned long debug_sync_attempts = 0;
    static unsigned long debug_packets_parsed = 0;
    
    Serial.printf("UART RX Task started, expecting %d byte packets\n", bytes_needed);
    
    while (true) {
        int available = Serial2.available();
        if (available > 0) {
            debug_bytes_count += available;
            
            if (!sync_found) {
                // Check if this might be a text response (not binary packet)
                uint8_t byte = Serial2.peek();  // Peek without consuming
                
                // If it's a printable ASCII character (likely text response), read as string
                if (byte >= 32 && byte < 127 && byte != 0xAA) {
                    String text_response = Serial2.readStringUntil('\n');
                    if (text_response.length() > 0) {
                        Serial.printf("[BLE_SLAVE] Text response received: %s\n", text_response.c_str());
                        
                        // Capture for command handler
                        if (command_response_mutex != NULL && xSemaphoreTake(command_response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            command_response_data = text_response;
                            command_response_ready = true;
                            xSemaphoreGive(command_response_mutex);
                            Serial.printf("[BLE_SLAVE] → Response captured for command handler\n");
                        }
                    }
                    bytes_received = 0;  // Reset
                    continue;  // Skip to next iteration
                }
                
                // Look for binary sync pattern
                byte = Serial2.read();
                debug_sync_attempts++;
                
                if (bytes_received == 0 && byte == 0xAA) {
                    packet_ptr[0] = byte;
                    bytes_received = 1;
                } else if (bytes_received == 1 && byte == 0x55) {
                    packet_ptr[1] = byte;
                    bytes_received = 2;
                    sync_found = true;
                } else {
                    bytes_received = 0;  // Reset if sync pattern broken
                }
            } else {
                // Read rest of packet
                int to_read = min(available, bytes_needed - bytes_received);
                
                if (to_read > 0) {
                    int actual_read = Serial2.readBytes(packet_ptr + bytes_received, to_read);
                    bytes_received += actual_read;
                    
                    if (bytes_received >= bytes_needed) {
                        // Complete packet received
                        process_uart_packet(&packet);
                        debug_packets_parsed++;
                        
                        // Reset for next packet
                        bytes_received = 0;
                        sync_found = false;
                    }
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));  // Small delay to prevent busy waiting
        }
        
        // Debug output every 5 seconds
        unsigned long now = millis();
        if (now - last_debug_time >= 5000) {
            Serial.printf("UART RX: %lu bytes, %lu sync attempts, %lu packets parsed in last 5s\n", 
                         debug_bytes_count, debug_sync_attempts, debug_packets_parsed);
            debug_bytes_count = 0;
            debug_sync_attempts = 0;
            debug_packets_parsed = 0;
            last_debug_time = now;
        }
    }
}

// ============================================================================
// BLE TRANSMISSION TASK
// ============================================================================

static void timer_processing_task(void* param) {
    while (true) {
        // Check if timer triggered a sample
        if (timer_sample_ready) {
            timer_sample_ready = false;
            create_8channel_sample_from_redis();
        }
        
        vTaskDelay(pdMS_TO_TICKS(1)); // Check every 1ms
    }
}

static void ble_tx_task(void* param) {
    while (true) {
        if (deviceConnected) {
            // Send any pending batch periodically (in case it's not full)
            if (current_sample_count > 0) {
                // Wait a bit to see if more samples arrive
                vTaskDelay(pdMS_TO_TICKS(5));
                
                // Send partial batch if we still have samples waiting
                if (current_sample_count > 0) {
                    send_current_batch();
                }
            }
        } else {
            // Reset batch when not connected
            current_sample_count = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10ms
    }
}

// ============================================================================
// SERIAL COMMANDS
// ============================================================================

static void handle_serial_commands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        Serial.printf("[BLE_SLAVE] Command received: %s\n", command.c_str());
        
        // Forward Teensy control commands to ESP32 RX Radio
        if (command == "START" || command == "STOP" || command == "RESTART" || command == "RESET" ||
            command == "REMOTE_START" || command == "REMOTE_STOP" || command == "REMOTE_RESTART" || command == "REMOTE_RESET" ||
            command == "ALL_START" || command == "ALL_STOP" || command == "ALL_RESTART" || command == "ALL_RESET" ||
            command == "ZERO" || command == "ZERO_STATUS" || command == "ZERO_RESET" ||
            command == "REMOTE_ZERO" || command == "REMOTE_ZERO_STATUS" || command == "REMOTE_ZERO_RESET" ||
            command == "ALL_ZERO" || command == "ALL_ZERO_STATUS" || command == "ALL_ZERO_RESET" ||
            command == "LOCAL_ON" || command == "LOCAL_OFF" || command == "REMOTE_ON" || command == "REMOTE_OFF" ||
            command == "REMOTE_PING") {
            
            // Special formatting for REMOTE_PING
            bool is_ping = (command == "REMOTE_PING");
            if (is_ping) {
                Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
                Serial.println("║         REMOTE PING-PONG TEST - BLE SLAVE                      ║");
                Serial.println("╚════════════════════════════════════════════════════════════════╝");
                Serial.printf("  📤 SENDING: %s\n", command.c_str());
                Serial.println("  Path: BLE Slave → RX Radio → SPI Slave → Remote Teensy");
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
            
            // Wait for response from ESP32 RX Radio (captured by UART RX task)
            bool is_zero_command = (command.indexOf("ZERO") >= 0);
            unsigned long timeout_ms = is_zero_command ? 15000 : 5000; // 15s for ZERO, 5s for others
            unsigned long timeout = millis() + timeout_ms;
            bool got_response = false;
            String response_text = "";
            
            while (millis() < timeout) {
                // Check if UART RX task captured a response
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
                    Serial.printf("  📥 RECEIVED: %s\n", response_text.c_str());
                    Serial.printf("  ⏱️  Round-trip time: %lu ms\n", round_trip_ms);
                    Serial.println("  ──────────────────────────────────────────────────────────────");
                    Serial.println("  ✅ PING-PONG TEST SUCCESS!");
                    Serial.println("╚════════════════════════════════════════════════════════════════╝\n");
                } else {
                    Serial.printf("[BLE_SLAVE] RX Radio Response: %s\n", response_text.c_str());
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
        // ESP32 RX Radio status commands
        else if (command == "RX_STATUS") {
            Serial.println("[BLE_SLAVE] Requesting status from ESP32 RX Radio...");
            Serial2.println("STATUS");
            Serial2.flush();
            
            // Wait for multi-line status response
            unsigned long timeout = millis() + 3000;
            while (millis() < timeout) {
                if (Serial2.available()) {
                    String response = Serial2.readStringUntil('\n');
                    if (response.length() > 0) {
                        Serial.printf("[BLE_SLAVE] RX Radio: %s\n", response.c_str());
                    }
                }
                delay(10);
            }
        }
        // BLE Slave specific commands
        else if (command == "STATS") {
            Serial.printf("UART: Packets=%lu, Bytes=%lu, CRC_Err=%lu, Sync_Err=%lu\n",
                         uart_packets_received, uart_bytes_received, uart_crc_errors, uart_sync_errors);
            Serial.printf("Samples: Local=%lu, Remote=%lu, Combined=%lu\n",
                         local_samples_received, remote_samples_received, combined_samples_received);
            Serial.printf("BLE: Connected=%s, Notifications=%lu, Errors=%lu\n",
                         deviceConnected ? "YES" : "NO", ble_notifications_sent, ble_send_errors);
            Serial.printf("Commands: Forwarded=%lu, Responses=%lu, Timeouts=%lu\n",
                         commands_forwarded, command_responses_received, command_timeouts);
            Serial.printf("BLE Commands: Received=%lu, Errors=%lu\n",
                         ble_commands_received, ble_command_errors);
        } else if (command == "RESET_STATS") {
            uart_packets_received = 0;
            uart_bytes_received = 0;
            uart_crc_errors = 0;
            uart_sync_errors = 0;
            local_samples_received = 0;
            remote_samples_received = 0;
            combined_samples_received = 0;
            ble_notifications_sent = 0;
            ble_send_errors = 0;
            commands_forwarded = 0;
            command_responses_received = 0;
            command_timeouts = 0;
            ble_commands_received = 0;
            ble_command_errors = 0;
            Serial.println("Statistics reset");
        } else if (command == "BLE_RESTART") {
            BLEDevice::startAdvertising();
            Serial.println("BLE advertising restarted");
        } else if (command == "UART_STATUS") {
            Serial.printf("UART Status:\n");
            Serial.printf("  Baud rate: %d\n", UART_BAUD_RATE);
            Serial.printf("  RX Pin: %d\n", UART_RX_PIN);
            Serial.printf("  Available bytes: %d\n", Serial2.available());
            Serial.printf("  RX buffer configured: 8192 bytes\n");
        } else if (command == "BATCH_STATUS") {
            Serial.printf("BLE Batch Status:\n");
            Serial.printf("  Current batch: %d/%d samples\n", current_sample_count, SAMPLES_PER_BLE_PACKET);
            Serial.printf("  Packets sent: %lu\n", ble_packets_sent);
            Serial.printf("  Samples batched: %lu\n", samples_batched);
            Serial.printf("  Timer samples: %lu\n", timer_samples_generated);
        } else if (command == "REDIS_STATUS") {
            Serial.printf("Redis Store Status:\n");
            Serial.printf("  Local queue: %d/%d samples (head=%d, tail=%d)\n", 
                         redis_store.local_count, REDIS_QUEUE_SIZE,
                         redis_store.local_head, redis_store.local_tail);
            Serial.printf("  Remote queue: %d/%d samples (head=%d, tail=%d)\n", 
                         redis_store.remote_count, REDIS_QUEUE_SIZE,
                         redis_store.remote_head, redis_store.remote_tail);
            Serial.printf("  Has data: Local=%s, Remote=%s\n", 
                         redis_store.has_local_data ? "YES" : "NO",
                         redis_store.has_remote_data ? "YES" : "NO");
            if (redis_store.has_local_data) {
                Serial.printf("  Last Local LC: [%ld, %ld, %ld, %ld]\n", 
                             redis_store.last_local_lc[0], redis_store.last_local_lc[1], 
                             redis_store.last_local_lc[2], redis_store.last_local_lc[3]);
            }
            if (redis_store.has_remote_data) {
                Serial.printf("  Last Remote LC: [%ld, %ld, %ld, %ld]\n", 
                             redis_store.last_remote_lc[0], redis_store.last_remote_lc[1], 
                             redis_store.last_remote_lc[2], redis_store.last_remote_lc[3]);
            }
        } else if (command == "HELP") {
            Serial.println("[BLE_SLAVE] === AVAILABLE COMMANDS ===");
            Serial.println("  Teensy Control (forwarded to RX Radio):");
            Serial.println("    START, STOP, RESTART, RESET, ZERO, ZERO_STATUS, ZERO_RESET");
            Serial.println("    REMOTE_START, REMOTE_STOP, REMOTE_RESTART, REMOTE_RESET, REMOTE_ZERO, REMOTE_ZERO_STATUS, REMOTE_ZERO_RESET");
            Serial.println("    ALL_START, ALL_STOP, ALL_RESTART, ALL_RESET, ALL_ZERO, ALL_ZERO_STATUS, ALL_ZERO_RESET");
            Serial.println("  ESP32 Control (forwarded to RX Radio):");
            Serial.println("    LOCAL_ON/OFF, REMOTE_ON/OFF");
            Serial.println("    RX_STATUS    - Get ESP32 RX Radio status");
            Serial.println("  Test Commands:");
            Serial.println("    REMOTE_PING  - Ping remote Teensy (via RX Radio -> SPI Slave -> Remote Teensy)");
            Serial.println("  BLE Slave Commands:");
            Serial.println("    STATS        - Show BLE slave statistics");
            Serial.println("    RESET_STATS  - Reset all statistics");
            Serial.println("    BLE_RESTART  - Restart BLE advertising");
            Serial.println("    UART_STATUS  - Show UART status");
            Serial.println("    BATCH_STATUS - Show BLE batch status");
            Serial.println("    REDIS_STATUS - Show Redis store status");
            Serial.println("    DEBUG_QUEUE  - Show queue contents");
            Serial.println("    HELP         - Show this help");
            Serial.println("=========================================");
        } else if (command == "DEBUG_QUEUE") {
            Serial.printf("Debug Queue Contents:\n");
            if (redis_store.local_count > 0) {
                SampleData* sample = &redis_store.local_queue[redis_store.local_head];
                Serial.printf("  Next Local: [%ld, %ld, %ld, %ld] frame=%d sample=%d\n",
                             sample->lc[0], sample->lc[1], sample->lc[2], sample->lc[3],
                             sample->frame_idx, sample->sample_idx);
            }
            if (redis_store.remote_count > 0) {
                SampleData* sample = &redis_store.remote_queue[redis_store.remote_head];
                Serial.printf("  Next Remote: [%ld, %ld, %ld, %ld] frame=%d sample=%d\n",
                             sample->lc[0], sample->lc[1], sample->lc[2], sample->lc[3],
                             sample->frame_idx, sample->sample_idx);
            }
        } else if (command == "") {
            // Empty command, do nothing
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
    
    // Create command response mutex for response capture
    command_response_mutex = xSemaphoreCreateMutex();
    if (command_response_mutex == NULL) {
        Serial.println("[BLE_SLAVE] ⚠ Failed to create command response mutex!");
    } else {
        Serial.println("[BLE_SLAVE] ✓ Command response mutex created");
    }
    
    Serial.println("ESP32 BLE Slave - Redis-like Load Cell Data Forwarder + Command Gateway");
    Serial.println("=== BLE COMMAND INTERFACE ENABLED ===");
    Serial.println("BLE Service UUID: 12345678-1234-1234-1234-123456789abc");
    Serial.println("Data Characteristic (Notify): 87654321-4321-4321-4321-cba987654321");
    Serial.println("Command Characteristic (Write): 11111111-2222-3333-4444-555555555555");
    Serial.println("Available BLE Commands:");
    Serial.println("  Teensy: START, STOP, RESTART, RESET, ZERO, ZERO_STATUS, ZERO_RESET");
    Serial.println("  Remote: REMOTE_START, REMOTE_STOP, REMOTE_RESTART, REMOTE_RESET, REMOTE_ZERO, REMOTE_ZERO_STATUS, REMOTE_ZERO_RESET");
    Serial.println("  Dual: ALL_START, ALL_STOP, ALL_RESTART, ALL_RESET, ALL_ZERO, ALL_ZERO_STATUS, ALL_ZERO_RESET");
    Serial.println("  ESP32: LOCAL_ON/OFF, REMOTE_ON/OFF, STATUS");
    Serial.println("Serial Commands: STATS, RESET_STATS, BLE_RESTART, UART_STATUS, HELP");
    Serial.println("======================================");
    
    // Initialize UART2 for receiving data from master ESP32
    Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial2.setRxBufferSize(8192);  // Increase RX buffer for high throughput
    Serial.printf("UART2 initialized at %d bps on RX pin %d\n", UART_BAUD_RATE, UART_RX_PIN);
    
    // Initialize BLE
    BLEDevice::init("LoadCell_BLE_Server");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
    
    // Data characteristic for sending sensor data (notifications)
    pDataCharacteristic = pService->createCharacteristic(
                        BLE_DATA_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pDataCharacteristic->addDescriptor(new BLE2902());
    
    // Command characteristic for receiving commands (write)
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
    pAdvertising->setMinPreferred(0x0);  // Set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    
    Serial.println("BLE server started with command interface, waiting for connections...");
    
    // Initialize hardware timer for 1000Hz sampling (1ms intervals)
    sampling_timer = timerBegin(1000000);  // 1MHz base frequency
    timerAttachInterrupt(sampling_timer, &timer_sample_callback);  // Attach interrupt
    timerAlarm(sampling_timer, 1000, true, 0);  // 1000 ticks = 1ms at 1MHz, auto-reload
    Serial.println("Hardware timer initialized at 1000Hz (1MHz base, 1000 tick interval)");
    
    // Start high-priority tasks
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 8192, NULL, 23, NULL, 0);        // Core 0, high priority
    xTaskCreatePinnedToCore(timer_processing_task, "timer_proc", 4096, NULL, 24, NULL, 1); // Core 1, highest priority
    xTaskCreatePinnedToCore(ble_tx_task, "ble_tx", 4096, NULL, 22, NULL, 1);          // Core 1, high priority
    
    Serial.println("Tasks started - Redis-like system ready to receive data");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    static unsigned long last_stats_time = 0;
    static unsigned long last_uart_packets = 0;
    static unsigned long last_uart_bytes = 0;
    static unsigned long last_ble_notifications = 0;
    
    unsigned long now = millis();
    
    // Print stats every 5 seconds
    if (now - last_stats_time >= 5000) {
        unsigned long uart_packet_rate = (uart_packets_received - last_uart_packets) / 5;
        unsigned long uart_byte_rate = (uart_bytes_received - last_uart_bytes) / 5;
        unsigned long uart_kbps = (uart_byte_rate * 8) / 1000;
        unsigned long ble_rate = (ble_notifications_sent - last_ble_notifications) / 5;
        
        Serial.printf("=== REDIS-LIKE SYSTEM STATS (T=%lu.%lus) ===\n", now / 1000, (now % 1000) / 100);
        
        // UART Reception Stats
        Serial.printf("UART RX: %lu pps (%lu kbps) | Local: %lu samples | Remote: %lu samples\n",
                     uart_packet_rate, uart_kbps, local_samples_received, remote_samples_received);
        
        // Redis Data Store Status
        Serial.printf("REDIS: Local queue [%d samples] | Remote queue [%d samples]\n",
                     redis_store.local_count, redis_store.remote_count);
        Serial.printf("       Has data: Local [%s] | Remote [%s]\n",
                     redis_store.has_local_data ? "YES" : "NO",
                     redis_store.has_remote_data ? "YES" : "NO");
        
        // Timer-Based Sampling Stats
        static unsigned long last_timer_interrupts = 0;
        static unsigned long last_timer_samples = 0;
        unsigned long timer_interrupt_rate = (timer_interrupts_count - last_timer_interrupts) / 5;
        unsigned long timer_sample_rate = (timer_samples_generated - last_timer_samples) / 5;
        Serial.printf("TIMER: %lu interrupts/sec | %lu samples/sec (target: 1000) | Total: %lu\n",
                     timer_interrupt_rate, timer_sample_rate, timer_samples_generated);
        last_timer_interrupts = timer_interrupts_count;
        last_timer_samples = timer_samples_generated;
        
        // BLE Transmission Stats
        Serial.printf("BLE: %s | %lu notifications/sec | Batch: %d/%d samples | %lu packets sent\n",
                     deviceConnected ? "CONNECTED" : "DISCONNECTED", ble_rate,
                     current_sample_count, SAMPLES_PER_BLE_PACKET, ble_packets_sent);
        
        // Error Summary
        if (uart_crc_errors > 0 || uart_sync_errors > 0 || ble_send_errors > 0) {
            Serial.printf("ERRORS: UART CRC=%lu, Sync=%lu | BLE=%lu\n",
                         uart_crc_errors, uart_sync_errors, ble_send_errors);
        }
        
        Serial.println("================================================");
        
        last_stats_time = now;
        last_uart_packets = uart_packets_received;
        last_uart_bytes = uart_bytes_received;
        last_ble_notifications = ble_notifications_sent;
    }
    
    // Handle BLE connection changes
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // Give the bluetooth stack time to get things ready
        pServer->startAdvertising(); // Restart advertising
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }
    
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
    
    handle_serial_commands();
    delay(100);
}
