#define EN1 31
#define EN2 32
#define EN3 33
#define EN4 34

#define DI1 37
#define DI2 38
#define DI3 39
#define DI4 40

// ---- DWT cycle counter (Teensy 4.1) ----
static inline void dwt_init() {
  *(volatile uint32_t *)0xE000EDFC |= 0x01000000; // DEMCR TRCENA
  *(volatile uint32_t *)0xE0001004 = 0;          // CYCCNT
  *(volatile uint32_t *)0xE0001000 |= 1;         // enable CYCCNT
}
static inline uint32_t dwt_cycles() { return *(volatile uint32_t *)0xE0001004; }
static inline void wait_cycles(uint32_t c) {
  uint32_t s = dwt_cycles();
  while ((uint32_t)(dwt_cycles() - s) < c) {}
}

// WS2812 800kHz timings
static inline void ws_send_byte(uint8_t pin, uint8_t b) {
  const uint32_t F = F_CPU;
  const uint32_t T0H = (uint32_t)(F * 0.35e-6);
  const uint32_t T1H = (uint32_t)(F * 0.70e-6);
  const uint32_t TT  = (uint32_t)(F * 1.25e-6);

  for (int i = 7; i >= 0; --i) {
    bool one = b & (1 << i);
    uint32_t start = dwt_cycles();
    digitalWriteFast(pin, HIGH);
    wait_cycles(one ? T1H : T0H);
    digitalWriteFast(pin, LOW);
    while ((uint32_t)(dwt_cycles() - start) < TT) {}
  }
}

static inline uint8_t scale8(uint8_t v, uint8_t br) {
  return (uint16_t)v * (uint16_t)br / 255;
}

static inline void ws_show_1pixel(uint8_t pin, uint8_t r, uint8_t g, uint8_t b, uint8_t br) {
  r = scale8(r, br);
  g = scale8(g, br);
  b = scale8(b, br);

  noInterrupts();
  // WS2812 order: GRB
  ws_send_byte(pin, g);
  ws_send_byte(pin, r);
  ws_send_byte(pin, b);
  interrupts();

  delayMicroseconds(80); // latch
}

static inline void allEN(bool on) {
  digitalWriteFast(EN1, on);
  digitalWriteFast(EN2, on);
  digitalWriteFast(EN3, on);
  digitalWriteFast(EN4, on);
}

static inline void allDI_low() {
  digitalWriteFast(DI1, LOW);
  digitalWriteFast(DI2, LOW);
  digitalWriteFast(DI3, LOW);
  digitalWriteFast(DI4, LOW);
}

static inline void setAll(uint8_t r, uint8_t g, uint8_t b, uint8_t br) {
  ws_show_1pixel(DI1, r, g, b, br);
  ws_show_1pixel(DI2, r, g, b, br);
  ws_show_1pixel(DI3, r, g, b, br);
  ws_show_1pixel(DI4, r, g, b, br);
}

void setup() {
  pinMode(EN1, OUTPUT); pinMode(EN2, OUTPUT); pinMode(EN3, OUTPUT); pinMode(EN4, OUTPUT);
  pinMode(DI1, OUTPUT); pinMode(DI2, OUTPUT); pinMode(DI3, OUTPUT); pinMode(DI4, OUTPUT);

  // Hard boot sequence to reduce latch issues
  allEN(false);
  allDI_low();
  delay(500);

  dwt_init();

  allEN(true);
  delay(500);

  // Send "off" a few times to sync
  for (int i = 0; i < 5; i++) { setAll(0,0,0,0); delay(20); }
}

void loop() {
  const uint8_t br = 80;

  setAll(255,0,0,br);   delay(300);
  setAll(0,255,0,br);   delay(300);
  setAll(0,0,255,br);   delay(300);
  setAll(255,255,255,br); delay(300);

  // brightness sweep on white
  for (int b = 0; b <= 255; b += 5) { setAll(255,255,255,(uint8_t)b); delay(15); }
  for (int b = 255; b >= 0; b -= 5) { setAll(255,255,255,(uint8_t)b); delay(15); }

  // flash
  for (int i=0;i<8;i++){ setAll(255,0,0,120); delay(120); setAll(0,0,0,0); delay(120); }
}


