#include <Arduino.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_MOSI 39
#define PIN_MISO 40
#define PIN_SCLK 38
#define PIN_CS   41

#define FRAME_BYTES 144
#define QUEUED_XFERS 32
#define SAMPLES_PER_FRAME 10

struct __attribute__((packed)) Frame {
  uint8_t  sync[2];
  uint8_t  plate_id, proto_ver;
  uint16_t frame_idx;
  uint32_t t0_us;
  uint8_t  samples[120];
  uint16_t crc16;
  uint8_t  pad[12];
};

static inline uint16_t crc16_ccitt_false(const uint8_t* d, uint32_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= ((uint16_t)*d++) << 8;
    for (int i=0;i<8;i++) crc = (crc & 0x8000) ? ((crc<<1) ^ 0x1021) : (crc<<1);
  }
  return crc;
}

// DMA-capable buffers + transactions
static uint8_t rxbuf[QUEUED_XFERS][FRAME_BYTES] __attribute__((aligned(4)));
static spi_slave_transaction_t trx[QUEUED_XFERS];

// stats (shared between tasks)
static volatile uint32_t frames_ok=0, frames_crc=0, frames_missed=0;
static volatile uint16_t last_idx=0; static volatile bool have_idx=false;
static volatile uint32_t last_rx_us=0, win_min_dt_us=UINT32_MAX, win_max_dt_us=0;
static volatile uint32_t win_ok=0, win_crc=0, win_missed=0;
static uint32_t t0_ms;

// Debug mode control
static volatile bool debug_mode = false;
static volatile bool output_raw_data = true; // Enable by default for testing

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
    Frame* f = (Frame*)rxbuf[idx];

    uint32_t now_us = micros();

    bool ok = (f->sync[0]==0xA5 && f->sync[1]==0x5A);
    if (ok) {
      uint16_t want = f->crc16;
      uint16_t got  = crc16_ccitt_false((uint8_t*)f, offsetof(Frame, crc16));
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

    Serial.printf("[ESP] T=%.1fs | ok=%lu crc=%lu miss=%lu | win5s ok=%lu crc=%lu miss=%lu | rate=%.1f kb/s (5s=%.1f) | sps=%.1f (5s=%.1f) | dt_us[min=%lu max=%lu] last_idx=%u\n",
      tsec,(unsigned long)ok,(unsigned long)crc,(unsigned long)miss,
      (unsigned long)w_ok,(unsigned long)w_crc,(unsigned long)w_miss,
      kbps_total,kbps_win,sps_total,sps_win,
      (unsigned long)dtmin,(unsigned long)dtmax,(unsigned)last_idx);

    // reset 5 s window
    win_ok=win_crc=win_missed=0;
    win_min_dt_us=UINT32_MAX; win_max_dt_us=0;
    last_ms = now;
  }
}

// Helper function to extract int24 little-endian values
static inline int32_t unpack_int24_le(const uint8_t* p) {
  uint32_t u = p[0] | (p[1] << 8) | (p[2] << 16);
  // Sign extend from 24-bit to 32-bit
  if (u & 0x800000) {
    u |= 0xFF000000;
  }
  return (int32_t)u;
}

// Process load cell data from received frame (compact format)
void process_load_cell_data_compact(Frame* f) {
  // Extract load cell values from the frame
  uint8_t* s = f->samples;
  
  for (int sample = 0; sample < SAMPLES_PER_FRAME; sample++) {
    int32_t lc1 = unpack_int24_le(s + 0);
    int32_t lc2 = unpack_int24_le(s + 3);
    int32_t lc3 = unpack_int24_le(s + 6);
    int32_t lc4 = unpack_int24_le(s + 9);
    
    // Compact format: FRAME:SAMPLE:LC1:LC2:LC3:LC4
    Serial.printf("DATA:%u:%d:%ld:%ld:%ld:%ld\n", 
                  f->frame_idx, sample, lc1, lc2, lc3, lc4);
    
    s += 12; // 4 channels × 3 bytes each
  }
}

// Process load cell data from received frame (verbose format)
void process_load_cell_data(Frame* f) {
  // Extract load cell values from the frame
  uint8_t* s = f->samples;
  
  Serial.printf("[ESP] Frame %u: Load Cell Data:\n", f->frame_idx);
  
  for (int sample = 0; sample < SAMPLES_PER_FRAME; sample++) {
    int32_t lc1 = unpack_int24_le(s + 0);
    int32_t lc2 = unpack_int24_le(s + 3);
    int32_t lc3 = unpack_int24_le(s + 6);
    int32_t lc4 = unpack_int24_le(s + 9);
    
    Serial.printf("  Sample %d: LC1=%ld LC2=%ld LC3=%ld LC4=%ld\n", 
                  sample, lc1, lc2, lc3, lc4);
    
    s += 12; // 4 channels × 3 bytes each
  }
}

// Handle serial commands
void handle_serial_commands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "DEBUG_ON") {
      output_raw_data = true;
      Serial.println("[ESP] Raw data output ENABLED");
    } else if (command == "DEBUG_OFF") {
      output_raw_data = false;
      Serial.println("[ESP] Raw data output DISABLED");
    } else if (command == "STATUS") {
      Serial.printf("[ESP] Status: frames_ok=%lu, output_raw_data=%s\n", 
                    (unsigned long)frames_ok, output_raw_data ? "ON" : "OFF");
    }
  }
}

void setup() {
  Serial.begin(921600); // fast prints so we never stall
  delay(200);
  Serial.println("\n[ESP] SPI SLAVE (queued DMA) starting");

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

  t0_ms = millis();
  xTaskCreatePinnedToCore(spi_rx_task, "spi_rx", 4096, NULL, 9, NULL, 1);
  xTaskCreatePinnedToCore(stats_task,  "stats",  4096, NULL, 3, NULL, 1);
  
  Serial.println("[ESP] SPI Slave initialized - waiting for Teensy data");
}

void loop() { 
  handle_serial_commands();
  vTaskDelay(pdMS_TO_TICKS(100)); 
}
