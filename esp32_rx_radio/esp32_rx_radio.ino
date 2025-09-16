#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <nvs.h>
#include "common_frame.h"

// ============================================================================
// RADIO CONFIGURATION - MODIFY THESE FOR YOUR SETUP
// ============================================================================

// Transport selection (set one to 1, the other to 0)
#define USE_WIFI_UDP 0
#define USE_ESPNOW 1

// Wi-Fi Configuration
#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASSWORD "YourWiFiPassword"
#define UDP_PORT 30303

// ESP-NOW Configuration
#define ESPNOW_CHANNEL 1
static const uint8_t ESPNOW_PEER_MAC[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x01};  // TX radio MAC

// Custom MAC Configuration
#define USE_CUSTOM_MAC 1
static const uint8_t CUSTOM_STA_MAC_RX[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x02};  // RX radio MAC

// Processing queue size
#define PROCESS_QUEUE_SIZE 64

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Processing queue for received frames
static QueueHandle_t process_queue = NULL;

// Reception statistics
static volatile uint32_t rx_packets_ok = 0;
static volatile uint32_t rx_packets_bad_hdr = 0;
static volatile uint32_t rx_packets_len_err = 0;
static volatile uint32_t rx_packets_crc_err = 0;
static volatile uint32_t rx_frames_missed = 0;
static volatile uint32_t rx_queue_overruns = 0;

// Window statistics (reset every 5s)
static volatile uint32_t rx_win_ok = 0;
static volatile uint32_t rx_win_bad_hdr = 0;
static volatile uint32_t rx_win_len_err = 0;
static volatile uint32_t rx_win_crc_err = 0;
static volatile uint32_t rx_win_missed = 0;
static volatile uint32_t rx_win_overruns = 0;

// Timing statistics
static volatile uint32_t last_rx_us = 0;
static volatile uint32_t win_min_dt_us = UINT32_MAX;
static volatile uint32_t win_max_dt_us = 0;

// Frame continuity tracking
static volatile uint16_t last_frame_idx = 0;
static volatile bool have_frame_idx = false;

// Network objects
#if USE_WIFI_UDP
static WiFiUDP udp;
static bool wifi_connected = false;
#endif

#if USE_ESPNOW
static bool espnow_initialized = false;
static volatile uint32_t espnow_rx_count = 0;
#endif

// Timing
static uint32_t t0_ms;

// Debug control
static volatile bool output_raw_data = true;

// ============================================================================
// CUSTOM MAC ADDRESS FUNCTIONS
// ============================================================================

static void set_custom_mac_if_enabled() {
#if USE_CUSTOM_MAC
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, (uint8_t*)CUSTOM_STA_MAC_RX);
  
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("[RX] Custom STA MAC set: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
}

// ============================================================================
// WIFI UDP FUNCTIONS
// ============================================================================

#if USE_WIFI_UDP
static void wifi_init() {
  set_custom_mac_if_enabled();
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[RX] Connecting to WiFi: %s", WIFI_SSID);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 50) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    udp.begin(UDP_PORT);
    
    Serial.printf("\n[RX] WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[RX] Listening on UDP port: %d\n", UDP_PORT);
  } else {
    Serial.println("\n[RX] WiFi connection failed!");
    wifi_connected = false;
  }
}

static void wifi_check_connection() {
  static uint32_t last_check = 0;
  uint32_t now = millis();
  
  if (now - last_check > 5000) {  // Check every 5 seconds
    if (WiFi.status() != WL_CONNECTED && wifi_connected) {
      Serial.println("[RX] WiFi disconnected, attempting reconnect...");
      wifi_connected = false;
      WiFi.reconnect();
    } else if (WiFi.status() == WL_CONNECTED && !wifi_connected) {
      Serial.println("[RX] WiFi reconnected!");
      wifi_connected = true;
      udp.begin(UDP_PORT);
    }
    last_check = now;
  }
}
#endif

// ============================================================================
// ESP-NOW FUNCTIONS
// ============================================================================

#if USE_ESPNOW
static void espnow_rx_callback(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
  espnow_rx_count++;
  
  if (len != INNER_FRAME_SIZE) {
    rx_packets_len_err++;
    rx_win_len_err++;
    return;
  }
  
  // Queue the received frame for processing
  if (process_queue != NULL) {
    if (xQueueSendFromISR(process_queue, data, NULL) != pdTRUE) {
      rx_queue_overruns++;
      rx_win_overruns++;
    }
  }
}

static void espnow_init() {
  set_custom_mac_if_enabled();
  
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("[RX] ESP-NOW init failed!");
    return;
  }
  
  esp_now_register_recv_cb(espnow_rx_callback);
  
  // Add peer (TX radio)
  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, ESPNOW_PEER_MAC, 6);
  peer_info.channel = ESPNOW_CHANNEL;
  peer_info.encrypt = false;
  
  if (esp_now_add_peer(&peer_info) == ESP_OK) {
    espnow_initialized = true;
    Serial.printf("[RX] ESP-NOW initialized, channel %d\n", ESPNOW_CHANNEL);
    Serial.printf("[RX] Peer MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  ESPNOW_PEER_MAC[0], ESPNOW_PEER_MAC[1], ESPNOW_PEER_MAC[2],
                  ESPNOW_PEER_MAC[3], ESPNOW_PEER_MAC[4], ESPNOW_PEER_MAC[5]);
  } else {
    Serial.println("[RX] Failed to add ESP-NOW peer!");
  }
}
#endif

// ============================================================================
// FRAME PROCESSING FUNCTIONS
// ============================================================================

static void process_received_frame(const uint8_t* frame_data) {
  uint32_t now_us = micros();
  
  // Cast to inner frame structure
  const InnerFrame* frame = (const InnerFrame*)frame_data;
  
  // Validate inner frame (SYNC bytes and CRC)
  if (!validate_inner_frame(frame)) {
    rx_packets_crc_err++;
    rx_win_crc_err++;
    return;
  }
  
  // Track frame continuity using inner frame_idx
  if (have_frame_idx) {
    uint16_t expected = last_frame_idx + 1;
    uint16_t missed = (uint16_t)(frame->frame_idx - expected);
    if (missed) {
      rx_frames_missed += missed;
      rx_win_missed += missed;
    }
  } else {
    have_frame_idx = true;
  }
  last_frame_idx = frame->frame_idx;
  
  // Update timing statistics
  if (last_rx_us != 0) {
    uint32_t dt = now_us - last_rx_us;
    if (dt < win_min_dt_us) win_min_dt_us = dt;
    if (dt > win_max_dt_us) win_max_dt_us = dt;
  }
  last_rx_us = now_us;
  
  // Update success counters
  rx_packets_ok++;
  rx_win_ok++;
  
  // Output raw data if enabled
  if (output_raw_data) {
    process_load_cell_data_compact(frame);
  }
}

// Process load cell data from received frame (compact format)
static void process_load_cell_data_compact(const InnerFrame* frame) {
  const uint8_t* samples = frame->samples;
  
  for (int sample = 0; sample < 10; sample++) {  // 10 samples per frame
    int32_t lc1, lc2, lc3, lc4;
    extract_load_cell_sample(samples, sample, &lc1, &lc2, &lc3, &lc4);
    
    // Compact format: DATA:FRAME:SAMPLE:LC1:LC2:LC3:LC4
    Serial.printf("DATA:%u:%d:%ld:%ld:%ld:%ld\n", 
                  frame->frame_idx, sample, lc1, lc2, lc3, lc4);
  }
}

// ============================================================================
// NETWORK RX TASK
// ============================================================================

#if USE_WIFI_UDP
static void udp_rx_task(void* param) {
  uint8_t packet_buffer[UDP_PACKET_SIZE];
  
  Serial.println("[RX] UDP RX task started");
  
  while (true) {
    wifi_check_connection();
    
    if (!wifi_connected) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    int packet_size = udp.parsePacket();
    if (packet_size > 0) {
      if (packet_size != UDP_PACKET_SIZE) {
        rx_packets_len_err++;
        rx_win_len_err++;
        udp.flush();
        continue;
      }
      
      // Read the complete packet
      int bytes_read = udp.read(packet_buffer, UDP_PACKET_SIZE);
      if (bytes_read != UDP_PACKET_SIZE) {
        rx_packets_len_err++;
        rx_win_len_err++;
        continue;
      }
      
      // Validate radio header
      RadioHdr* hdr = (RadioHdr*)packet_buffer;
      if (hdr->magic != RADIO_MAGIC || hdr->len != INNER_FRAME_SIZE) {
        rx_packets_bad_hdr++;
        rx_win_bad_hdr++;
        continue;
      }
      
      // Queue the inner frame for processing
      uint8_t* frame_data = packet_buffer + RADIO_HEADER_SIZE;
      if (process_queue != NULL) {
        if (xQueueSend(process_queue, frame_data, 0) != pdTRUE) {
          rx_queue_overruns++;
          rx_win_overruns++;
        }
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));  // Small delay to prevent busy waiting
    }
  }
}
#endif

// ============================================================================
// FRAME PROCESSING TASK
// ============================================================================

static void frame_process_task(void* param) {
  uint8_t frame_buffer[INNER_FRAME_SIZE];
  
  Serial.println("[RX] Frame processing task started");
  
  while (true) {
    // Wait for frame from network RX task
    if (xQueueReceive(process_queue, frame_buffer, portMAX_DELAY) == pdTRUE) {
      process_received_frame(frame_buffer);
    }
  }
}

// ============================================================================
// STATISTICS TASK
// ============================================================================

static void stats_task(void* param) {
  const uint32_t PRINT_MS = 5000;
  uint32_t last_ms = millis();
  
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(100));
    uint32_t now = millis();
    if (now - last_ms < PRINT_MS) continue;
    
    double tsec = (now - t0_ms) / 1000.0;
    uint32_t ok = rx_packets_ok, bad_hdr = rx_packets_bad_hdr;
    uint32_t len_err = rx_packets_len_err, crc_err = rx_packets_crc_err;
    uint32_t missed = rx_frames_missed, overruns = rx_queue_overruns;
    
    uint32_t w_ok = rx_win_ok, w_bad_hdr = rx_win_bad_hdr;
    uint32_t w_len_err = rx_win_len_err, w_crc_err = rx_win_crc_err;
    uint32_t w_missed = rx_win_missed, w_overruns = rx_win_overruns;
    
    uint32_t dtmin = (win_min_dt_us == UINT32_MAX ? 0 : win_min_dt_us);
    uint32_t dtmax = win_max_dt_us;
    
    double kbps_total = (ok * INNER_FRAME_SIZE * 8.0) / tsec / 1000.0;
    double sps_total = (ok * 10 * 4) / tsec;  // 10 samples × 4 channels per frame
    double kbps_win = (w_ok * INNER_FRAME_SIZE * 8.0) / (PRINT_MS/1000.0) / 1000.0;
    double sps_win = (w_ok * 10 * 4) / (PRINT_MS/1000.0);
    
    Serial.printf("[RX] T=%.1fs | ok=%lu bad_hdr=%lu len_err=%lu crc_err=%lu missed=%lu overruns=%lu | win5s ok=%lu bad_hdr=%lu len_err=%lu crc_err=%lu missed=%lu overruns=%lu | rate=%.1f kb/s (5s=%.1f) | sps=%.1f (5s=%.1f) | jitter_us[min=%lu max=%lu] last_idx=%u\n",
      tsec, (unsigned long)ok, (unsigned long)bad_hdr, (unsigned long)len_err, 
      (unsigned long)crc_err, (unsigned long)missed, (unsigned long)overruns,
      (unsigned long)w_ok, (unsigned long)w_bad_hdr, (unsigned long)w_len_err,
      (unsigned long)w_crc_err, (unsigned long)w_missed, (unsigned long)w_overruns,
      kbps_total, kbps_win, sps_total, sps_win,
      (unsigned long)dtmin, (unsigned long)dtmax, (unsigned)last_frame_idx);
    
#if USE_ESPNOW
    Serial.printf("[RX] ESP-NOW: rx_count=%lu\n", (unsigned long)espnow_rx_count);
#endif
    
    // Reset 5s window
    rx_win_ok = rx_win_bad_hdr = rx_win_len_err = rx_win_crc_err = 0;
    rx_win_missed = rx_win_overruns = 0;
    win_min_dt_us = UINT32_MAX;
    win_max_dt_us = 0;
    last_ms = now;
  }
}

// ============================================================================
// SERIAL COMMAND HANDLING
// ============================================================================

static void handle_serial_commands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "DEBUG_ON") {
      output_raw_data = true;
      Serial.println("[RX] Raw data output ENABLED");
    } else if (command == "DEBUG_OFF") {
      output_raw_data = false;
      Serial.println("[RX] Raw data output DISABLED");
    } else if (command == "STATUS") {
      Serial.printf("[RX] Status: rx_ok=%lu, missed=%lu, output_raw_data=%s\n", 
                    (unsigned long)rx_packets_ok, (unsigned long)rx_frames_missed,
                    output_raw_data ? "ON" : "OFF");
#if USE_WIFI_UDP
      Serial.printf("[RX] WiFi: %s, IP: %s, Port: %d\n", 
                    wifi_connected ? "Connected" : "Disconnected",
                    WiFi.localIP().toString().c_str(), UDP_PORT);
#elif USE_ESPNOW
      Serial.printf("[RX] ESP-NOW: %s, Channel: %d, RX Count: %lu\n",
                    espnow_initialized ? "Initialized" : "Not initialized",
                    ESPNOW_CHANNEL, (unsigned long)espnow_rx_count);
#endif
    } else if (command == "STATS") {
      Serial.printf("[RX] Detailed stats:\n");
      Serial.printf("  Total: ok=%lu, bad_hdr=%lu, len_err=%lu, crc_err=%lu\n",
                    (unsigned long)rx_packets_ok, (unsigned long)rx_packets_bad_hdr,
                    (unsigned long)rx_packets_len_err, (unsigned long)rx_packets_crc_err);
      Serial.printf("  Missed frames: %lu, Queue overruns: %lu\n",
                    (unsigned long)rx_frames_missed, (unsigned long)rx_queue_overruns);
      Serial.printf("  Last frame idx: %u, Have idx: %s\n",
                    (unsigned)last_frame_idx, have_frame_idx ? "YES" : "NO");
    }
  }
}

// ============================================================================
// MAIN SETUP AND LOOP
// ============================================================================

void setup() {
  Serial.begin(921600);
  delay(200);
  Serial.println("\n[RX] RADIO RX starting");
  
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  
  // Create processing queue
  process_queue = xQueueCreate(PROCESS_QUEUE_SIZE, INNER_FRAME_SIZE);
  if (process_queue == NULL) {
    Serial.println("[RX] Failed to create processing queue!");
    while(1) delay(1000);
  }
  
  // Initialize networking
#if USE_WIFI_UDP
  Serial.println("[RX] Initializing Wi-Fi UDP...");
  wifi_init();
#elif USE_ESPNOW
  Serial.println("[RX] Initializing ESP-NOW...");
  espnow_init();
#endif
  
  // Start tasks
  t0_ms = millis();
  
#if USE_WIFI_UDP
  xTaskCreatePinnedToCore(udp_rx_task, "udp_rx", 4096, NULL, 8, NULL, 1);
#endif
  
  xTaskCreatePinnedToCore(frame_process_task, "frame_proc", 4096, NULL, 6, NULL, 0);
  xTaskCreatePinnedToCore(stats_task, "stats", 4096, NULL, 3, NULL, 1);
  
  Serial.println("[RX] All systems initialized - ready to receive data");
  Serial.println("[RX] Commands: DEBUG_ON, DEBUG_OFF, STATUS, STATS");
}

void loop() {
  handle_serial_commands();
  vTaskDelay(pdMS_TO_TICKS(100));
}
