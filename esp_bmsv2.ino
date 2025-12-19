#include <Wire.h>
#include <math.h>

// ===========================
// CONFIG (EDIT HERE)
// ===========================
static const uint8_t BQ_ADDR = 0x6B;
static const int I2C_SDA = 8;
static const int I2C_SCL = 9;
static const uint32_t I2C_HZ = 400000;

static const float VREG_TARGET_V    = 7.20f;
static const float VSYSMIN_TARGET_V = 6.00f;
static const float ICHG_TARGET_A    = 2.50f;
static const float IIN_TARGET_A     = 3.00f;

static const float START_CHG_PCT = 90.0f;
static const float TAPER_A = 0.20f;
static const uint32_t TAPER_HOLD_MS = 60000;
static const uint32_t MIN_STATE_HOLD_MS = 10000;

static const uint32_t SAMPLE_MS = 500;
static const float EMA_ALPHA = 0.10f;

// VBAT calibration (verified by you)
static const float VBAT_GAIN     = 1.0000f;
static const float VBAT_OFFSET_V = -0.111f;

// Debug
#define CHG_DEBUG 1
#if CHG_DEBUG
  #define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINTF(...) do{}while(0)
#endif

// ===========================
// BQ REG MAP (subset)
// ===========================
static const uint8_t REG00_VSYSMIN  = 0x00;
static const uint8_t REG01_VREG     = 0x01;
static const uint8_t REG03_ICHG     = 0x03;
static const uint8_t REG06_IINDPM   = 0x06;
static const uint8_t REG0F_CTRL0    = 0x0F;  // EN_CHG bit5
static const uint8_t REG10_CTRL1    = 0x10;  // WATCHDOG bits[2:0]

// ADC control
static const uint8_t REG2E_ADC_CTRL = 0x2E;  // ADC_EN bit7, ADC_RATE bit6
static const uint8_t REG2F_ADC_DIS0 = 0x2F;  // channel disables (we clear VBAT disable bit)

// Measurements
static const uint8_t REG3B_VBAT_ADC = 0x3B;  // MSB-first, mV (confirmed)
static const uint8_t REG32_IBAT     = 0x32;  // LSB-first, signed, mA (confirmed)

// ===========================
// State
// ===========================
#define CHG_OFF 0
#define CHG_ON  1

static int chargeState = CHG_OFF;
static uint32_t lastSampleMs = 0;
static uint32_t lastStateChangeMs = 0;
static uint32_t taperStartMs = 0;

static float vBatFiltCal = NAN;
static float iBatFilt    = NAN;

// ===========================
// SOC LUT (VBATcal -> SOC)
// ===========================
struct Pt { float v; float soc; };
static const Pt SOC_LUT[] = {
  {5.80f,   0.0f},
  {6.00f,   5.0f},
  {6.20f,  10.0f},
  {6.30f,  20.0f},
  {6.40f,  30.0f},
  {6.50f,  40.0f},
  {6.56f,  50.0f},
  {6.60f,  60.0f},
  {6.64f,  70.0f},
  {6.68f,  80.0f},
  {6.74f,  90.0f},
  {7.00f,  95.0f},
  {7.10f,  98.0f},
  {7.20f, 100.0f}
};

static float socFromLUT(float v) {
  const int n = (int)(sizeof(SOC_LUT)/sizeof(SOC_LUT[0]));
  if (v <= SOC_LUT[0].v) return SOC_LUT[0].soc;
  if (v >= SOC_LUT[n-1].v) return SOC_LUT[n-1].soc;
  for (int i=0;i<n-1;i++){
    float v0=SOC_LUT[i].v, v1=SOC_LUT[i+1].v;
    if (v>=v0 && v<=v1){
      float s0=SOC_LUT[i].soc, s1=SOC_LUT[i+1].soc;
      float t=(v-v0)/(v1-v0);
      return s0 + t*(s1-s0);
    }
  }
  return 0.0f;
}

// ===========================
// I2C helpers
// ===========================
static bool bqProbe(){
  Wire.beginTransmission(BQ_ADDR);
  return (Wire.endTransmission()==0);
}

static bool bqWrite8(uint8_t reg, uint8_t val){
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission()==0);
}

static bool bqRead8(uint8_t reg, uint8_t &val){
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false)!=0) return false;
  if (Wire.requestFrom((uint8_t)BQ_ADDR,(uint8_t)1)!=1) return false;
  val=Wire.read();
  return true;
}

static bool bqWrite16_LSB(uint8_t reg, uint16_t val){
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(val & 0xFF));
  Wire.write((uint8_t)((val>>8) & 0xFF));
  return (Wire.endTransmission()==0);
}

static bool bqRead16_LSB(uint8_t reg, uint16_t &val){
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false)!=0) return false;
  if (Wire.requestFrom((uint8_t)BQ_ADDR,(uint8_t)2)!=2) return false;
  uint8_t lsb=Wire.read();
  uint8_t msb=Wire.read();
  val=(uint16_t)lsb | ((uint16_t)msb<<8);
  return true;
}

static bool bqRead16_MSB(uint8_t reg, uint16_t &val){
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false)!=0) return false;
  if (Wire.requestFrom((uint8_t)BQ_ADDR,(uint8_t)2)!=2) return false;
  uint8_t msb=Wire.read();
  uint8_t lsb=Wire.read();
  val=((uint16_t)msb<<8) | (uint16_t)lsb;
  return true;
}

// ===========================
// Encoders
// ===========================
static uint8_t encVSYSMIN_mV(float mV){
  if (mV<2500) mV=2500;
  if (mV>16000) mV=16000;
  return (uint8_t)((mV-2500.0f)/250.0f);
}
static uint16_t encVREG_mV(float mV){
  if (mV<3000) mV=3000;
  if (mV>18800) mV=18800;
  return (uint16_t)(mV/10.0f);
}
static uint16_t encI_mA(float mA, float minmA, float maxmA){
  if (mA<minmA) mA=minmA;
  if (mA>maxmA) mA=maxmA;
  return (uint16_t)(mA/10.0f);
}

// ===========================
// Charger control + ADC enable
// ===========================
static bool bqDisableWatchdog(){
  uint8_t r10=0;
  if (!bqRead8(REG10_CTRL1, r10)) return false;
  r10 &= ~0x07; // WD=0
  return bqWrite8(REG10_CTRL1, r10);
}

static bool bqChargerEnable(bool en){
  uint8_t r0f=0;
  if (!bqRead8(REG0F_CTRL0, r0f)) return false;
  if (en) r0f |= (1<<5);
  else    r0f &= ~(1<<5);
  return bqWrite8(REG0F_CTRL0, r0f);
}

// THIS WAS MISSING in your "all zeros" case:
static void bqAdcEnableBestEffort(){
  uint8_t r2e=0, r2f=0;

  if (bqRead8(REG2E_ADC_CTRL, r2e)){
    // bit7 ADC_EN = 1
    // bit6 ADC_RATE = 0 (continuous)  (keep it this way)
    r2e |= (1<<7);
    r2e &= ~(1<<6);
    bqWrite8(REG2E_ADC_CTRL, r2e);
  }

  if (bqRead8(REG2F_ADC_DIS0, r2f)){
    // Clear VBAT disable bit (bit4 in our earlier assumption)
    // (If your silicon maps slightly different, this still won't hurt much;
    //  key is ADC_EN above.)
    r2f &= ~(1<<4);
    bqWrite8(REG2F_ADC_DIS0, r2f);
  }

  // allow ADC to settle
  delay(20);
}

static bool bqInit2S_LiFePO4(){
  if (!bqDisableWatchdog()) return false;

  uint8_t vsys = encVSYSMIN_mV(VSYSMIN_TARGET_V*1000.0f) & 0x3F;
  if (!bqWrite8(REG00_VSYSMIN, vsys)) return false;

  uint16_t vreg = encVREG_mV(VREG_TARGET_V*1000.0f);
  if (!bqWrite16_LSB(REG01_VREG, vreg)) return false;

  uint16_t ichg = encI_mA(ICHG_TARGET_A*1000.0f, 50.0f, 5000.0f);
  if (!bqWrite16_LSB(REG03_ICHG, ichg)) return false;

  uint16_t iin  = encI_mA(IIN_TARGET_A*1000.0f, 100.0f, 3300.0f);
  if (!bqWrite16_LSB(REG06_IINDPM, iin)) return false;

  // Enable ADC before reading VBAT/IBAT
  bqAdcEnableBestEffort();

  return bqChargerEnable(false);
}

// ===========================
// Measurements
// ===========================
static float applyVbatCal(float vraw){
  return vraw*VBAT_GAIN + VBAT_OFFSET_V;
}

static bool readVBATraw(float &vbatV, uint16_t &raw_mV){
  uint16_t raw=0;
  if (!bqRead16_MSB(REG3B_VBAT_ADC, raw)) return false;
  raw_mV=raw;
  vbatV=raw/1000.0f;
  return true;
}

static bool readIBAT(float &ibatA, int16_t &raw_mA){
  uint16_t u=0;
  if (!bqRead16_LSB(REG32_IBAT, u)) return false;
  int16_t s=(int16_t)u;
  raw_mA = s;
  ibatA = (float)s/1000.0f;
  return true;
}

// ===========================
// State
// ===========================
static void setChargeState(int newState){
  if (newState == chargeState) return;
  uint32_t now=millis();
  if (lastStateChangeMs!=0 && (now-lastStateChangeMs < MIN_STATE_HOLD_MS)) return;

  if (!bqChargerEnable(newState==CHG_ON)){
    return;
  }

  chargeState=newState;
  lastStateChangeMs=now;
  taperStartMs=0;
}

// ===========================
// Arduino glue
// ===========================
void setup(){
  Serial.begin(115200);
  delay(600);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(50);

  if (!bqProbe()){
    // If you ever see this, your I2C wiring/pins are wrong or device not powered
    // Serial.println("ERROR: BQ no ACK");
  } else {
    bqInit2S_LiFePO4();
  }

  lastSampleMs = millis();
}

void loop(){
  uint32_t now=millis();
  if (now-lastSampleMs < SAMPLE_MS) return;
  lastSampleMs = now;

  if (!bqProbe()){
    return;
  }

  float vraw=0; uint16_t vmv=0;
  if (!readVBATraw(vraw, vmv)){
    return;
  }
  float vcal = applyVbatCal(vraw);

  float ibat=0; int16_t imA=0;
  if (!readIBAT(ibat, imA)){
    return;
  }

  // Filters (use calibrated VBAT for everything)
  if (isnan(vBatFiltCal)) vBatFiltCal = vcal;
  else vBatFiltCal = EMA_ALPHA*vcal + (1.0f-EMA_ALPHA)*vBatFiltCal;

  if (isnan(iBatFilt)) iBatFilt = ibat;
  else iBatFilt = EMA_ALPHA*ibat + (1.0f-EMA_ALPHA)*iBatFilt;

  float soc = socFromLUT(vBatFiltCal);

  // ---- AUTO CONTROL (VBATcal-based) ----
  if (chargeState==CHG_OFF){
    if (soc < START_CHG_PCT) setChargeState(CHG_ON);
  } else {
    if (vBatFiltCal >= VREG_TARGET_V && fabs(iBatFilt) <= TAPER_A){
      if (taperStartMs==0) taperStartMs=now;
      if ((now - taperStartMs) >= TAPER_HOLD_MS){
        setChargeState(CHG_OFF);
      }
    } else {
      taperStartMs=0;
    }
  }

  // Prints intentionally commented (enable by CHG_DEBUG=1 if needed)
  DBG_PRINTF("VBATraw=%.3fV(%u) VBATcal=%.3f fVBATcal=%.3f SOC=%.1f%% IBAT=%+.3fA raw=%d(mA) fIBAT=%+.3fA CHG=%s\n",
             vraw, vmv, vcal, vBatFiltCal, soc, ibat, (int)imA, iBatFilt,
             (chargeState==CHG_ON)?"ON":"OFF");
}
