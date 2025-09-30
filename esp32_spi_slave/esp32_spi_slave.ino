#include <Arduino.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
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
#define QUEUED_XFERS 32
#define SAMPLES_PER_FRAME 10

// ============================================================================
// RADIO CONFIGURATION - MODIFY THESE FOR YOUR SETUP
// ============================================================================

// Transport selection (set one to 1, the other to 0)
#define USE_WIFI_UDP 0
#define USE_ESPNOW 1

// Wi-Fi Configuration
#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASSWORD "YourWiFiPassword"
#define DEST_IP_STR "192.168.1.100"  // RX radio IP address
#define UDP_PORT 30303

// ESP-NOW Configuration
#define ESPNOW_CHANNEL 1
static const uint8_t ESPNOW_PEER_MAC[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x03};  // RX Radio MAC

// Custom MAC Configuration
#define USE_CUSTOM_MAC 1
static const uint8_t CUSTOM_STA_MAC_TX[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x01};  // SPI Slave MAC

// Network buffer size
#define NET_QUEUE_SIZE 64



// DMA-capable buffers + transactions
static uint8_t rxbuf[QUEUED_XFERS][FRAME_BYTES] __attribute__((aligned(4)));
static spi_slave_transaction_t trx[QUEUED_XFERS];

// SPI stats (shared between tasks)
static volatile uint32_t frames_ok=0, frames_crc=0, frames_missed=0;
static volatile uint16_t last_idx=0; static volatile bool have_idx=false;
static volatile uint32_t last_rx_us=0, win_min_dt_us=UINT32_MAX, win_max_dt_us=0;
static volatile uint32_t win_ok=0, win_crc=0, win_missed=0;
static uint32_t t0_ms;

// Debug mode control
static volatile bool debug_mode = false;
static volatile bool output_raw_data = true; // Enable by default for testing

// ============================================================================
// RADIO NETWORKING VARIABLES
// ============================================================================

// Network queue for frames to transmit
static QueueHandle_t net_queue = NULL;

// Network statistics
static volatile uint32_t net_sent_ok = 0;
static volatile uint32_t net_send_fail = 0;
static volatile uint32_t net_queue_drops = 0;
static volatile uint32_t net_win_sent = 0;
static volatile uint32_t net_win_fail = 0;
static volatile uint32_t net_win_drops = 0;

// Network objects
#if USE_WIFI_UDP
static WiFiUDP udp;
static IPAddress dest_ip;
static bool wifi_connected = false;
#endif

#if USE_ESPNOW
static bool espnow_initialized = false;
static volatile uint32_t espnow_send_success = 0;
static volatile uint32_t espnow_send_fail = 0;
#endif

// Radio sequence number (independent of inner frame_idx)
static uint16_t radio_seq = 0;

// ============================================================================
// CUSTOM MAC ADDRESS FUNCTIONS
// ============================================================================

static void set_custom_mac_if_enabled() {
#if USE_CUSTOM_MAC
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, (uint8_t*)CUSTOM_STA_MAC_TX);
  
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("[TX] Custom STA MAC set: %02X:%02X:%02X:%02X:%02X:%02X\n", 
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
  Serial.printf("[TX] Connecting to WiFi: %s", WIFI_SSID);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 50) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    dest_ip.fromString(DEST_IP_STR);
    udp.begin(0);  // Use any available port for sending
    
    Serial.printf("\n[TX] WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[TX] Target: %s:%d\n", DEST_IP_STR, UDP_PORT);
  } else {
    Serial.println("\n[TX] WiFi connection failed!");
    wifi_connected = false;
  }
}

static void wifi_send_frame(const uint8_t* frame_data) {
  if (!wifi_connected || WiFi.status() != WL_CONNECTED) {
    net_send_fail++;
    net_win_fail++;
    return;
  }
  
  // Build radio header
  RadioHdr hdr;
  hdr.magic = RADIO_MAGIC;
  hdr.seq = radio_seq++;
  hdr.len = FRAME_BYTES;
  
  // Send UDP packet (header + payload)
  udp.beginPacket(dest_ip, UDP_PORT);
  udp.write((uint8_t*)&hdr, sizeof(hdr));
  udp.write(frame_data, FRAME_BYTES);
  
  if (udp.endPacket()) {
    net_sent_ok++;
    net_win_sent++;
  } else {
    net_send_fail++;
    net_win_fail++;
  }
}

static void wifi_check_connection() {
  static uint32_t last_check = 0;
  uint32_t now = millis();
  
  if (now - last_check > 5000) {  // Check every 5 seconds
    if (WiFi.status() != WL_CONNECTED && wifi_connected) {
      Serial.println("[TX] WiFi disconnected, attempting reconnect...");
      wifi_connected = false;
      WiFi.reconnect();
    } else if (WiFi.status() == WL_CONNECTED && !wifi_connected) {
      Serial.println("[TX] WiFi reconnected!");
      wifi_connected = true;
    }
    last_check = now;
  }
}
#endif

// ============================================================================
// ESP-NOW FUNCTIONS
// ============================================================================

#if USE_ESPNOW
static void espnow_send_callback(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    espnow_send_success++;
  } else {
    espnow_send_fail++;
    net_send_fail++;
    net_win_fail++;
  }
}

static void espnow_init() {
  set_custom_mac_if_enabled();
  
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("[TX] ESP-NOW init failed!");
    return;
  }
  
  esp_now_register_send_cb(espnow_send_callback);
  
  // Add peer
  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, ESPNOW_PEER_MAC, 6);
  peer_info.channel = ESPNOW_CHANNEL;
  peer_info.encrypt = false;
  
  if (esp_now_add_peer(&peer_info) == ESP_OK) {
    espnow_initialized = true;
    Serial.printf("[TX] ESP-NOW initialized, channel %d\n", ESPNOW_CHANNEL);
    Serial.printf("[TX] Peer MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  ESPNOW_PEER_MAC[0], ESPNOW_PEER_MAC[1], ESPNOW_PEER_MAC[2],
                  ESPNOW_PEER_MAC[3], ESPNOW_PEER_MAC[4], ESPNOW_PEER_MAC[5]);
  } else {
    Serial.println("[TX] Failed to add ESP-NOW peer!");
  }
}

static void espnow_send_frame(const uint8_t* frame_data) {
  if (!espnow_initialized) {
    net_send_fail++;
    net_win_fail++;
    return;
  }
  
  // Send raw 144-byte frame (ESP-NOW has its own reliability)
  esp_err_t result = esp_now_send(ESPNOW_PEER_MAC, frame_data, FRAME_BYTES);
  
  if (result == ESP_OK) {
    net_sent_ok++;
    net_win_sent++;
  } else {
    net_send_fail++;
    net_win_fail++;
  }
}
#endif

// ============================================================================
// NETWORK TX TASK
// ============================================================================

static void net_tx_task(void* param) {
  uint8_t frame_buffer[FRAME_BYTES];
  
  Serial.println("[TX] Network TX task started");
  
  while (true) {
    // Wait for frame from SPI RX task
    if (xQueueReceive(net_queue, frame_buffer, portMAX_DELAY) == pdTRUE) {
      
#if USE_WIFI_UDP
      wifi_send_frame(frame_buffer);
      wifi_check_connection();
#elif USE_ESPNOW
      espnow_send_frame(frame_buffer);
#endif
      
    }
  }
}

// ============================================================================
// SPI FUNCTIONS (MODIFIED)
// ============================================================================

static void requeue_all() {
  for (int i=0;i<QUEUED_XFERS;i++) {
    memset(&trx[i], 0, sizeof(trx[i]));
    trx[i].length    = FRAME_BYTES * 8;
    trx[i].rx_buffer = rxbuf[i];
    trx[i].user      = (void*)(intptr_t)i;  // remember which buffer
    ESP_ERROR_CHECK(spi_slave_queue_trans(SPI2_HOST, &trx[i], portMAX_DELAY));
  }
}

static void spi_rx_task(void*){
  while (true) {
    spi_slave_transaction_t* ret;
    esp_err_t e = spi_slave_get_trans_result(SPI2_HOST, &ret, portMAX_DELAY);
    if (e != ESP_OK) continue;
    int idx = (int)(intptr_t)ret->user;
    InnerFrame* f = (InnerFrame*)rxbuf[idx];

    uint32_t now_us = micros();

    bool ok = (f->sync[0]==0xA5 && f->sync[1]==0x5A);
    if (ok) {
      uint16_t want = f->crc16;
      uint16_t got  = crc16_ccitt_false((uint8_t*)f, offsetof(InnerFrame, crc16));
      ok = (want == got);
    }
    if (!ok) {
      frames_crc++; win_crc++;
    } else {
      if (have_idx) {
        uint16_t expected = last_idx + 1;
        uint16_t missed = (uint16_t)(f->frame_idx - expected);
        if (missed) { frames_missed += missed; win_missed += missed; }
      } else {
        have_idx = true;
      }
      last_idx = f->frame_idx;

      if (last_rx_us != 0) {
        uint32_t dt = now_us - last_rx_us;
        if (dt < win_min_dt_us) win_min_dt_us = dt;
        if (dt > win_max_dt_us) win_max_dt_us = dt;
      }
      last_rx_us = now_us;

      frames_ok++; win_ok++;
      
      // Queue frame for network transmission (non-blocking)
      if (net_queue != NULL) {
        if (xQueueSend(net_queue, rxbuf[idx], 0) != pdTRUE) {
          net_queue_drops++;
          net_win_drops++;
        }
      }
      
      // Output raw data if debug mode is enabled
      if (output_raw_data) {
        process_load_cell_data_compact(f);
      }
    }

    // immediately re-queue the same buffer so we never run dry
    ESP_ERROR_CHECK(spi_slave_queue_trans(SPI2_HOST, ret, portMAX_DELAY));
  }
}

static void stats_task(void*){
  const uint32_t PRINT_MS = 5000;
  uint32_t last_ms = millis();
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(100));
    uint32_t now = millis();
    if (now - last_ms < PRINT_MS) continue;

    double tsec = (now - t0_ms) / 1000.0;
    uint32_t ok = frames_ok, crc = frames_crc, miss = frames_missed;
    uint32_t w_ok = win_ok, w_crc = win_crc, w_miss = win_missed;
    uint32_t dtmin = (win_min_dt_us==UINT32_MAX?0:win_min_dt_us);
    uint32_t dtmax = win_max_dt_us;

    double kbps_total = (ok * FRAME_BYTES * 8.0) / tsec / 1000.0;
    double sps_total  = (ok * SAMPLES_PER_FRAME * 4) / tsec; // 4 channels per sample
    double kbps_win   = (w_ok * FRAME_BYTES * 8.0) / (PRINT_MS/1000.0) / 1000.0;
    double sps_win    = (w_ok * SAMPLES_PER_FRAME * 4) / (PRINT_MS/1000.0); // 4 channels per sample

    // Network stats
    uint32_t net_ok = net_sent_ok, net_fail = net_send_fail, net_drops = net_queue_drops;
    uint32_t net_w_ok = net_win_sent, net_w_fail = net_win_fail, net_w_drops = net_win_drops;
    
    Serial.printf("[TX] T=%.1fs | SPI: ok=%lu crc=%lu miss=%lu | NET: sent=%lu fail=%lu drops=%lu | win5s SPI: ok=%lu crc=%lu miss=%lu | NET: sent=%lu fail=%lu drops=%lu | rate=%.1f kb/s (5s=%.1f) | sps=%.1f (5s=%.1f) | dt_us[min=%lu max=%lu] last_idx=%u\n",
      tsec,(unsigned long)ok,(unsigned long)crc,(unsigned long)miss,
      (unsigned long)net_ok,(unsigned long)net_fail,(unsigned long)net_drops,
      (unsigned long)w_ok,(unsigned long)w_crc,(unsigned long)w_miss,
      (unsigned long)net_w_ok,(unsigned long)net_w_fail,(unsigned long)net_w_drops,
      kbps_total,kbps_win,sps_total,sps_win,
      (unsigned long)dtmin,(unsigned long)dtmax,(unsigned)last_idx);

    // reset 5 s window
    win_ok=win_crc=win_missed=0;
    win_min_dt_us=UINT32_MAX; win_max_dt_us=0;
    net_win_sent=net_win_fail=net_win_drops=0;
    last_ms = now;
  }
}


// Process load cell data from received frame (compact format)
void process_load_cell_data_compact(InnerFrame* f) {
  for (int sample = 0; sample < SAMPLES_PER_FRAME; sample++) {
    int32_t lc1, lc2, lc3, lc4;
    extract_load_cell_sample(f->samples, sample, &lc1, &lc2, &lc3, &lc4);
    
    // Compact format: DATA:FRAME:SAMPLE:LC1:LC2:LC3:LC4
    Serial.printf("DATA:%u:%d:%ld:%ld:%ld:%ld\n", 
                  f->frame_idx, sample, lc1, lc2, lc3, lc4);
  }
}

// Process load cell data from received frame (verbose format)
void process_load_cell_data(InnerFrame* f) {
  Serial.printf("[ESP] Frame %u: Load Cell Data:\n", f->frame_idx);
  
  for (int sample = 0; sample < SAMPLES_PER_FRAME; sample++) {
    int32_t lc1, lc2, lc3, lc4;
    extract_load_cell_sample(f->samples, sample, &lc1, &lc2, &lc3, &lc4);
    
    Serial.printf("  Sample %d: LC1=%ld LC2=%ld LC3=%ld LC4=%ld\n", 
                  sample, lc1, lc2, lc3, lc4);
  }
}

// Handle serial commands
void handle_serial_commands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    Serial.printf("[SPI_SLAVE] Command received: %s\n", command.c_str());
    
    // Teensy control commands
    if (command == "START" || command == "STOP" || command == "RESTART" || command == "RESET") {
      Serial.printf("[SPI_SLAVE] Forwarding '%s' to Remote Teensy...\n", command.c_str());
      Serial1.println(command);
      Serial1.flush();
      
      // Wait for response from Teensy
      unsigned long timeout = millis() + 2000; // 2 second timeout
      while (millis() < timeout) {
        if (Serial1.available()) {
          String response = Serial1.readStringUntil('\n');
          Serial.printf("[SPI_SLAVE] Teensy Response: %s\n", response.c_str());
          break;
        }
        delay(10);
      }
      if (millis() >= timeout) {
        Serial.println("[SPI_SLAVE] ⚠ Teensy response timeout");
      }
    }
    // ESP32 control commands
    else if (command == "DEBUG_ON") {
      output_raw_data = true;
      Serial.println("[SPI_SLAVE] ✓ Raw data output enabled");
    } else if (command == "DEBUG_OFF") {
      output_raw_data = false;
      Serial.println("[SPI_SLAVE] ✓ Raw data output disabled");
    } else if (command == "STATUS") {
      Serial.printf("[SPI_SLAVE] === ESP32 SPI SLAVE STATUS ===\n");
      Serial.printf("  Frames OK: %lu\n", (unsigned long)frames_ok);
      Serial.printf("  Network sent: %lu\n", (unsigned long)net_sent_ok);
      Serial.printf("  Raw data: %s\n", output_raw_data ? "ON" : "OFF");
#if USE_WIFI_UDP
      Serial.printf("  WiFi: %s\n", wifi_connected ? "Connected" : "Disconnected");
      if (wifi_connected) {
        Serial.printf("  IP: %s\n", WiFi.localIP().toString().c_str());
      }
#elif USE_ESPNOW
      Serial.printf("  ESP-NOW: %s\n", espnow_initialized ? "Initialized" : "Not initialized");
      Serial.printf("  Channel: %d\n", ESPNOW_CHANNEL);
#endif
      Serial.printf("  Free heap: %lu bytes\n", ESP.getFreeHeap());
      Serial.println("========================================");
    } else if (command == "NET") {
      Serial.printf("[SPI_SLAVE] === NETWORK STATISTICS ===\n");
      Serial.printf("  Sent OK: %lu\n", (unsigned long)net_sent_ok);
      Serial.printf("  Send fail: %lu\n", (unsigned long)net_send_fail);
      Serial.printf("  Queue drops: %lu\n", (unsigned long)net_queue_drops);
#if USE_ESPNOW
      Serial.printf("  ESP-NOW success: %lu\n", (unsigned long)espnow_send_success);
      Serial.printf("  ESP-NOW fail: %lu\n", (unsigned long)espnow_send_fail);
#endif
      Serial.println("===================================");
    } else if (command == "HELP") {
      Serial.println("[SPI_SLAVE] === AVAILABLE COMMANDS ===");
      Serial.println("  Teensy Control:");
      Serial.println("    START       - Start Teensy data acquisition");
      Serial.println("    STOP        - Stop Teensy data acquisition");
      Serial.println("    RESTART     - Restart Teensy data acquisition");
      Serial.println("    RESET       - Reset Teensy");
      Serial.println("  ESP32 Control:");
      Serial.println("    DEBUG_ON/OFF - Enable/disable raw data output");
      Serial.println("    STATUS       - Show system status");
      Serial.println("    NET          - Show network statistics");
      Serial.println("    HELP         - Show this help");
      Serial.println("======================================");
    } else if (command == "") {
      // Empty command, do nothing
    } else {
      Serial.printf("[SPI_SLAVE] ✗ Unknown command: '%s'\n", command.c_str());
      Serial.println("[SPI_SLAVE] Type 'HELP' for available commands");
    }
  }
}

void setup() {
  Serial.begin(921600); // fast prints so we never stall
  delay(200);
  Serial.println("\n[TX] SPI SLAVE + RADIO TX starting");

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Create network queue
  net_queue = xQueueCreate(NET_QUEUE_SIZE, FRAME_BYTES);
  if (net_queue == NULL) {
    Serial.println("[TX] Failed to create network queue!");
    while(1) delay(1000);
  }

  // Initialize networking
#if USE_WIFI_UDP
  Serial.println("[TX] Initializing Wi-Fi UDP...");
  wifi_init();
#elif USE_ESPNOW
  Serial.println("[TX] Initializing ESP-NOW...");
  espnow_init();
#endif

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
  gpio_set_pull_mode((gpio_num_t)PIN_CS,   GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)PIN_MOSI, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)PIN_SCLK, GPIO_PULLUP_ONLY);

  requeue_all();

  // Initialize Serial1 for Teensy communication
  Serial1.begin(115200, SERIAL_8N1, TEENSY_UART_RX_PIN, TEENSY_UART_TX_PIN);
  Serial.printf("[TX] Serial1 initialized at 115200 bps for Teensy communication (TX=%d, RX=%d)\n", 
                TEENSY_UART_TX_PIN, TEENSY_UART_RX_PIN);

  // Start tasks
  t0_ms = millis();
  xTaskCreatePinnedToCore(spi_rx_task, "spi_rx", 4096, NULL, 9, NULL, 1);
  xTaskCreatePinnedToCore(net_tx_task, "net_tx", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(stats_task,  "stats",  4096, NULL, 3, NULL, 1);
  
  Serial.println("[TX] All systems initialized - ready for Teensy data");
  Serial.println("[SPI_SLAVE] ==========================================");
  Serial.println("[SPI_SLAVE] ESP32 SPI Slave ready for commands");
  Serial.println("[SPI_SLAVE] Teensy commands: START, STOP, RESTART, RESET");
  Serial.println("[SPI_SLAVE] ESP32 commands: DEBUG_ON/OFF, STATUS, NET, HELP");
  Serial.println("[SPI_SLAVE] ==========================================");
}

void loop() { 
  handle_serial_commands();
  vTaskDelay(pdMS_TO_TICKS(100)); 
}
