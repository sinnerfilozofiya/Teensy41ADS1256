#include <Arduino.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_MOSI 39
#define PIN_MISO 40
#define PIN_SCLK 38
#define PIN_CS   41

#define BATCH_SIZE 100  // 100 samples per batch
#define QUEUED_XFERS 8  // Reduced for larger frames
#define SAMPLES_PER_BATCH 100

// Single sample structure (12 bytes: 4 channels × 3 bytes each)
struct __attribute__((packed)) Sample {
  uint8_t lc1[3];  // 24-bit little-endian
  uint8_t lc2[3];
  uint8_t lc3[3];
  uint8_t lc4[3];
};

// Batch frame structure (1220 bytes total)
struct __attribute__((packed)) BatchFrame {
  uint8_t  sync[2];           // 2 bytes: 0xA5, 0x5A
  uint8_t  plate_id;          // 1 byte
  uint8_t  proto_ver;         // 1 byte
  uint16_t batch_idx;         // 2 bytes: batch number
  uint32_t t0_us;             // 4 bytes: start timestamp
  uint64_t timestamp_us;      // 8 bytes: precise timestamp
  Sample   samples[BATCH_SIZE]; // 1200 bytes: 100 samples × 12 bytes
  uint16_t crc16;             // 2 bytes: CRC checksum
};

#define BATCH_FRAME_BYTES sizeof(BatchFrame)  // Should be 1220 bytes

static inline uint16_t crc16_ccitt_false(const uint8_t* d, uint32_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= ((uint16_t)*d++) << 8;
    for (int i=0;i<8;i++) crc = (crc & 0x8000) ? ((crc<<1) ^ 0x1021) : (crc<<1);
  }
  return crc;
}

// Unpack little-endian 24-bit signed integer
static inline int32_t unpack_int24_le(const uint8_t* p) {
  uint32_t u = p[0] | (p[1] << 8) | (p[2] << 16);
  // Sign extend from 24-bit to 32-bit
  if (u & 0x800000) {
    u |= 0xFF000000;
  }
  return (int32_t)u;
}

// DMA-capable buffers + transactions
static uint8_t rxbuf[QUEUED_XFERS][BATCH_FRAME_BYTES] __attribute__((aligned(4)));
static spi_slave_transaction_t trx[QUEUED_XFERS];

// stats (shared between tasks)
static volatile uint32_t batches_ok=0, batches_crc=0, batches_missed=0;
static volatile uint16_t last_batch_idx=0; static volatile bool have_batch_idx=false;
static volatile uint32_t last_rx_us=0, win_min_dt_us=UINT32_MAX, win_max_dt_us=0;
static volatile uint32_t win_ok=0, win_crc=0, win_missed=0;
static uint32_t t0_ms;

// Variables to track load cell values and sample counts
static uint32_t total_samples_received = 0;
static uint32_t samples_in_window = 0;

static void requeue_all() {
  for (int i=0;i<QUEUED_XFERS;i++) {
    memset(&trx[i], 0, sizeof(trx[i]));
    trx[i].length    = BATCH_FRAME_BYTES * 8;
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
    BatchFrame* batch = (BatchFrame*)rxbuf[idx];

    uint32_t now_us = micros();

    bool ok = (batch->sync[0]==0xA5 && batch->sync[1]==0x5A);
    if (ok) {
      uint16_t want = batch->crc16;
      uint16_t got  = crc16_ccitt_false((uint8_t*)batch, offsetof(BatchFrame, crc16));
      ok = (want == got);
    }
    if (!ok) {
      batches_crc++; win_crc++;
    } else {
      if (have_batch_idx) {
        uint16_t expected = last_batch_idx + 1;
        uint16_t missed = (uint16_t)(batch->batch_idx - expected);
        if (missed) { batches_missed += missed; win_missed += missed; }
      } else {
        have_batch_idx = true;
      }
      last_batch_idx = batch->batch_idx;

      if (last_rx_us != 0) {
        uint32_t dt = now_us - last_rx_us;
        if (dt < win_min_dt_us) win_min_dt_us = dt;
        if (dt > win_max_dt_us) win_max_dt_us = dt;
      }
      last_rx_us = now_us;

      batches_ok++; win_ok++;
      total_samples_received += BATCH_SIZE;
      samples_in_window += BATCH_SIZE;

      // Calculate time difference for synchronization analysis
      uint64_t esp_timestamp = ((uint64_t)millis() * 1000ULL) + (micros() % 1000);
      int64_t time_diff = (int64_t)(esp_timestamp - batch->timestamp_us);

      // Print batch header
      Serial.printf("[ESP] Batch:%u | Samples:%d | TotalSamples:%lu | diff:%lld us\n", 
                    batch->batch_idx, BATCH_SIZE, total_samples_received, time_diff);

      // Print ALL samples from the batch for CSV collection
      Serial.printf("[ESP] BatchData:%u Start\n", batch->batch_idx);
      for (int i = 0; i < BATCH_SIZE; i++) {
        int32_t lc1 = unpack_int24_le(batch->samples[i].lc1);
        int32_t lc2 = unpack_int24_le(batch->samples[i].lc2);
        int32_t lc3 = unpack_int24_le(batch->samples[i].lc3);
        int32_t lc4 = unpack_int24_le(batch->samples[i].lc4);
        Serial.printf("[ESP] Sample:%u:%d:%ld:%ld:%ld:%ld\n", 
                      batch->batch_idx, i, lc1, lc2, lc3, lc4);
      }
      Serial.printf("[ESP] BatchData:%u End\n", batch->batch_idx);
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
    uint32_t ok = batches_ok, crc = batches_crc, miss = batches_missed;
    uint32_t w_ok = win_ok, w_crc = win_crc, w_miss = win_missed;
    uint32_t dtmin = (win_min_dt_us==UINT32_MAX?0:win_min_dt_us);
    uint32_t dtmax = win_max_dt_us;

    double kbps_total = (ok * BATCH_FRAME_BYTES * 8.0) / tsec / 1000.0;
    double bps_total  = ok / tsec;  // Batches per second (should be ~10)
    double sps_total  = (ok * BATCH_SIZE) / tsec;  // Samples per second (should be ~1000)
    double kbps_win   = (w_ok * BATCH_FRAME_BYTES * 8.0) / (PRINT_MS/1000.0) / 1000.0;
    double bps_win    = (w_ok) / (PRINT_MS/1000.0);
    double sps_win    = (w_ok * BATCH_SIZE) / (PRINT_MS/1000.0);

    // Print comprehensive 5-second statistics
    Serial.println("\n========== ESP32 ADS1256 BATCH STATS (5 Second Window) ==========");
    Serial.printf("Runtime: %.1f seconds\n", tsec);
    Serial.printf("Total Batches: OK=%lu | CRC_ERR=%lu | MISSED=%lu\n", 
                  (unsigned long)ok, (unsigned long)crc, (unsigned long)miss);
    Serial.printf("Window (5s): OK=%lu | CRC_ERR=%lu | MISSED=%lu\n", 
                  (unsigned long)w_ok, (unsigned long)w_crc, (unsigned long)w_miss);
    Serial.printf("Data Rate: %.1f kb/s total | %.1f kb/s (5s window)\n", kbps_total, kbps_win);
    Serial.printf("Batch Rate: %.1f BPS total | %.1f BPS (5s window)\n", bps_total, bps_win);
    Serial.printf("Sample Rate: %.1f SPS total | %.1f SPS (5s window)\n", sps_total, sps_win);
    Serial.printf("Batch Timing: min=%lu us | max=%lu us | last_batch=%u\n", 
                  (unsigned long)dtmin, (unsigned long)dtmax, (unsigned)last_batch_idx);
    Serial.printf("Total Samples Received: %lu\n", total_samples_received);
    Serial.printf("Frame Size: %d bytes | Samples per Batch: %d\n", BATCH_FRAME_BYTES, BATCH_SIZE);
    Serial.println("================================================================\n");

    // reset 5 s window
    win_ok=win_crc=win_missed=0;
    samples_in_window=0;
    win_min_dt_us=UINT32_MAX; win_max_dt_us=0;
    last_ms = now;
  }
}

void setup() {
  Serial.begin(921600); // fast prints so we never stall
  delay(200);
  Serial.println("\n[ESP] SPI SLAVE for ADS1256 Batch Protocol (1000 SPS) starting");

  spi_bus_config_t bus{};
  bus.mosi_io_num = PIN_MOSI;
  bus.miso_io_num = PIN_MISO;
  bus.sclk_io_num = PIN_SCLK;
  bus.max_transfer_sz = BATCH_FRAME_BYTES;

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
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }