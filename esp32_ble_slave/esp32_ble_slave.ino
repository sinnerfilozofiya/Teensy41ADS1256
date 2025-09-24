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
#define UART_RX_PIN 18          // GPIO18 for UART RX
#define UART_TX_PIN 17          // GPIO17 for UART TX (not used but defined)

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
#define BLE_CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
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

// BLE transmission queue - Reduced for Windows BLE compatibility
#define BLE_QUEUE_SIZE 200   // Smaller queue to prevent Windows BLE overflow
#define BLE_SAMPLE_DECIMATION 1  // Send every sample for maximum throughput
// Forward declarations
struct LoadCellSample;
struct BLEDataPacket;

// Statistics
static unsigned long sample_decimation_counter = 0;
static unsigned long samples_decimated = 0;
static unsigned long samples_batched = 0;
static unsigned long ble_packets_sent = 0;

// 8-channel load cell sample (minimal structure)
struct __attribute__((packed)) LoadCellSample {
    int32_t local_lc[4];    // Local load cells 1-4 (16 bytes)
    int32_t remote_lc[4];   // Remote load cells 5-8 (16 bytes)
};  // Total: 32 bytes per sample

// BLE packet with multiple 8-channel samples
#define SAMPLES_PER_BLE_PACKET 5  // 5 samples per packet = 160 bytes + header
struct __attribute__((packed)) BLEDataPacket {
    uint8_t sample_count;                           // Number of samples in this packet
    LoadCellSample samples[SAMPLES_PER_BLE_PACKET]; // Array of 8-channel samples
};  // Total: 1 + (32 * 5) = 161 bytes

// Batch transmission system (after struct definitions)
static BLEDataPacket current_ble_packet;
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
    
    // Add 8-channel sample to current batch
    if (current_sample_count < SAMPLES_PER_BLE_PACKET) {
        LoadCellSample* sample = &current_ble_packet.samples[current_sample_count];
        memcpy(sample->local_lc, local_lc, sizeof(sample->local_lc));
        memcpy(sample->remote_lc, remote_lc, sizeof(sample->remote_lc));
        current_sample_count++;
    }
    
    // Send batch when full
    if (current_sample_count >= SAMPLES_PER_BLE_PACKET) {
        send_current_batch();
    }
}

// ============================================================================
// BATCH SAMPLE PROCESSING
// ============================================================================

static inline void send_current_batch() {
    if (current_sample_count > 0 && deviceConnected) {
        current_ble_packet.sample_count = current_sample_count;
        
        // Calculate packet size based on actual sample count
        size_t packet_size = 1 + (current_sample_count * sizeof(LoadCellSample));
        
        try {
            pCharacteristic->setValue((uint8_t*)&current_ble_packet, packet_size);
            pCharacteristic->notify();
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
                // Look for sync pattern
                uint8_t byte = Serial2.read();
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
        
        if (command == "STATS") {
            Serial.printf("UART: Packets=%lu, Bytes=%lu, CRC_Err=%lu, Sync_Err=%lu\n",
                         uart_packets_received, uart_bytes_received, uart_crc_errors, uart_sync_errors);
            Serial.printf("Samples: Local=%lu, Remote=%lu, Combined=%lu\n",
                         local_samples_received, remote_samples_received, combined_samples_received);
            Serial.printf("BLE: Connected=%s, Notifications=%lu, Errors=%lu\n",
                         deviceConnected ? "YES" : "NO", ble_notifications_sent, ble_send_errors);
        } else if (command == "RESET") {
            uart_packets_received = 0;
            uart_bytes_received = 0;
            uart_crc_errors = 0;
            uart_sync_errors = 0;
            local_samples_received = 0;
            remote_samples_received = 0;
            combined_samples_received = 0;
            ble_notifications_sent = 0;
            ble_send_errors = 0;
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
        }
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(921600);
    delay(100);
    
    Serial.println("ESP32 BLE Slave - Redis-like Load Cell Data Forwarder");
    Serial.println("Commands: STATS, RESET, BLE_RESTART, UART_STATUS, BATCH_STATUS, REDIS_STATUS");
    
    // Initialize UART2 for receiving data from master ESP32
    Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial2.setRxBufferSize(8192);  // Increase RX buffer for high throughput
    Serial.printf("UART2 initialized at %d bps on RX pin %d\n", UART_BAUD_RATE, UART_RX_PIN);
    
    // Initialize BLE
    BLEDevice::init("LoadCell_BLE_Server");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
    
    pCharacteristic = pService->createCharacteristic(
                        BLE_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    
    pCharacteristic->addDescriptor(new BLE2902());
    
    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);  // Set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    
    Serial.println("BLE server started, waiting for connections...");
    
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
