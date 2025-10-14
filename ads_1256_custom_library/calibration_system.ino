// ============================================================================
// COMPREHENSIVE FORCE PLATE CALIBRATION SYSTEM
// ============================================================================
// Implements 3-step calibration process:
// Step A: ADC (ADS1256) calibration for different PGA/data-rate settings
// Step B: Per-load-cell calibration (tare, span, linearity, shunt, temp)
// Step C: Whole-plate matrix calibration for Fz & COP
// ============================================================================

#include <EEPROM.h>

// Include guard for ForceData structure
#ifndef FORCE_DATA_STRUCT_DEFINED
#define FORCE_DATA_STRUCT_DEFINED

// ForceData structure definition
struct ForceData {
  double fz;        // Vertical force (N)
  double mx;        // Moment about X axis (N⋅m)
  double my;        // Moment about Y axis (N⋅m)
  double cop_x;     // Center of pressure X (mm)
  double cop_y;     // Center of pressure Y (mm)
  bool valid;       // Data validity
};

#endif // FORCE_DATA_STRUCT_DEFINED

// ============================================================================
// CALIBRATION DATA STRUCTURES
// ============================================================================

// Step A: ADC Calibration Data
struct ADCCalibration {
  uint8_t pga_setting;      // PGA gain setting (1, 2, 4, 8, 16, 32, 64)
  uint8_t data_rate;        // Data rate setting (DR_30000, DR_15000, etc.)
  uint32_t ofc_value;       // Offset calibration (OFC0, OFC1, OFC2)
  uint32_t fsc_value;       // Full-scale calibration (FSC0, FSC1, FSC2)
  uint32_t timestamp;       // When calibration was performed
  bool valid;               // Calibration data validity flag
};

// Step B: Per-Load-Cell Calibration Data
struct LoadCellCalibration {
  // Basic coefficients
  int32_t b0_offset;        // Zero offset (tare) in ADC counts
  double b1_span;           // Span coefficient (N or kg per count)
  double b2_nonlinear;      // Optional 2nd-order nonlinearity coefficient
  
  // Shunt calibration
  double shunt_factor;      // Shunt calibration factor (S)
  int32_t shunt_reading;    // ADC reading with shunt applied
  
  // Temperature compensation
  double temp_coeff;        // Temperature coefficient (αT, counts/°C)
  double ref_temp;          // Reference temperature (T0)
  
  // Calibration metadata
  uint32_t timestamp;       // When calibration was performed
  double masses_used[10];   // Masses used for span calibration
  uint8_t num_points;       // Number of calibration points used
  double linearity_error;   // Maximum linearity error (%)
  bool valid;               // Calibration data validity flag
};

// Step C: Matrix Calibration Data
struct MatrixCalibration {
  // Linear mapping coefficients: Fz = a^T * r, Mx = b^T * r, My = c^T * r
  double a_fz[4];           // Force coefficients for each load cell
  double b_mx[4];           // Moment X coefficients for each load cell  
  double c_my[4];           // Moment Y coefficients for each load cell
  
  // Plate geometry and reference
  double plate_length;      // Plate length (mm)
  double plate_width;       // Plate width (mm)
  double gravity;           // Gravity constant used (m/s²)
  
  // Calibration quality metrics
  double fz_rms_error;      // RMS error in force measurement (N)
  double cop_rms_error_x;   // RMS error in COP X (mm)
  double cop_rms_error_y;   // RMS error in COP Y (mm)
  
  // Calibration metadata
  uint32_t timestamp;       // When calibration was performed
  uint8_t num_positions;    // Number of positions used
  uint8_t num_masses;       // Number of masses used per position
  bool valid;               // Calibration data validity flag
};

// Complete calibration state
struct CalibrationData {
  ADCCalibration adc_cal;
  LoadCellCalibration lc_cal[4];  // One for each load cell
  MatrixCalibration matrix_cal;
  uint32_t system_serial;         // System serial number
  uint16_t cal_version;           // Calibration data version
  uint16_t crc16;                 // Data integrity check
};

// ============================================================================
// GLOBAL CALIBRATION VARIABLES
// ============================================================================

static CalibrationData cal_data;
static bool calibration_loaded = false;

// EEPROM storage addresses
#define EEPROM_CAL_START_ADDR 0
#define EEPROM_CAL_SIZE sizeof(CalibrationData)

// Calibration states
enum CalibrationState {
  CAL_STATE_IDLE = 0,
  CAL_STATE_ADC_OFFSET = 1,
  CAL_STATE_ADC_GAIN = 2,
  CAL_STATE_LC_TARE = 3,
  CAL_STATE_LC_SPAN = 4,
  CAL_STATE_LC_SHUNT = 5,
  CAL_STATE_MATRIX_COLLECT = 6,
  CAL_STATE_MATRIX_COMPUTE = 7
};

static CalibrationState current_cal_state = CAL_STATE_IDLE;
static uint8_t current_cal_channel = 0;
// static uint8_t current_cal_point = 0;  // Reserved for future use

// Temporary data collection arrays
static double temp_masses[20];
static double temp_positions_x[20];
static double temp_positions_y[20];
static int32_t temp_readings[4][20];  // [channel][point]
static uint8_t temp_data_count = 0;

// ============================================================================
// STEP A: ADC CALIBRATION FUNCTIONS
// ============================================================================

bool perform_adc_self_calibration(uint8_t pga_setting, uint8_t data_rate) {
  Serial.printf("[CAL] Starting ADC self-calibration (PGA=%d, DR=0x%02X)\n", 
                pga_setting, data_rate);
  
  // Set the desired PGA and data rate
  SetRegisterValue(ADCON, pga_setting);
  SetRegisterValue(DRATE, data_rate);
  
  // Wait for settings to stabilize
  Serial.println("[CAL] ⏳ Waiting for ADC settings to stabilize...");
  delay(100);
  
  // Perform self-calibration (both offset and gain)
  Serial.println("[CAL] 🔄 Performing SELFCAL (offset + gain)...");
  SendCMD(SELFCAL);
  
  // Wait for calibration to complete with progress updates
  Serial.println("[CAL] ⏱️  Calibration in progress (up to 5 seconds)...");
  for (int i = 0; i < 5; i++) {
    delay(1000);
    Serial.printf("[CAL] 📊 Progress: %d/5 seconds elapsed\n", i + 1);
  }
  
  Serial.println("[CAL] 📖 Reading calibration results...");
  
  // Read calibration results
  uint32_t ofc = 0, fsc = 0;
  ofc |= ((uint32_t)GetRegisterValue(OFC2)) << 16;
  ofc |= ((uint32_t)GetRegisterValue(OFC1)) << 8;
  ofc |= GetRegisterValue(OFC0);
  
  fsc |= ((uint32_t)GetRegisterValue(FSC2)) << 16;
  fsc |= ((uint32_t)GetRegisterValue(FSC1)) << 8;
  fsc |= GetRegisterValue(FSC0);
  
  // Store calibration data
  cal_data.adc_cal.pga_setting = pga_setting;
  cal_data.adc_cal.data_rate = data_rate;
  cal_data.adc_cal.ofc_value = ofc;
  cal_data.adc_cal.fsc_value = fsc;
  cal_data.adc_cal.timestamp = millis();
  cal_data.adc_cal.valid = true;
  
  Serial.printf("[CAL] ✅ ADC Calibration Complete:\n");
  Serial.printf("[CAL]   OFC: 0x%06lX (%ld)\n", ofc, ofc);
  Serial.printf("[CAL]   FSC: 0x%06lX (%ld)\n", fsc, fsc);
  
  return true;
}

bool restore_adc_calibration() {
  if (!cal_data.adc_cal.valid) {
    Serial.println("[CAL] No valid ADC calibration data");
    return false;
  }
  
  Serial.println("[CAL] Restoring ADC calibration...");
  
  // Restore PGA and data rate
  SetRegisterValue(ADCON, cal_data.adc_cal.pga_setting);
  SetRegisterValue(DRATE, cal_data.adc_cal.data_rate);
  
  // Restore calibration coefficients
  SetRegisterValue(OFC0, cal_data.adc_cal.ofc_value & 0xFF);
  SetRegisterValue(OFC1, (cal_data.adc_cal.ofc_value >> 8) & 0xFF);
  SetRegisterValue(OFC2, (cal_data.adc_cal.ofc_value >> 16) & 0xFF);
  
  SetRegisterValue(FSC0, cal_data.adc_cal.fsc_value & 0xFF);
  SetRegisterValue(FSC1, (cal_data.adc_cal.fsc_value >> 8) & 0xFF);
  SetRegisterValue(FSC2, (cal_data.adc_cal.fsc_value >> 16) & 0xFF);
  
  Serial.printf("[CAL] ADC calibration restored (PGA=%d, DR=0x%02X)\n",
                cal_data.adc_cal.pga_setting, cal_data.adc_cal.data_rate);
  
  return true;
}

// ============================================================================
// STEP B: PER-LOAD-CELL CALIBRATION FUNCTIONS
// ============================================================================

bool start_load_cell_tare(uint8_t channel) {
  if (channel >= 4) return false;
  
  Serial.printf("[CAL] Starting tare calibration for Load Cell %d\n", channel + 1);
  Serial.println("[CAL] Ensure no load is applied to the plate");
  Serial.println("[CAL] Collecting 500 samples for averaging...");
  
  int32_t sum = 0;
  const int num_samples = 500;
  
  for (int i = 0; i < num_samples; i++) {
    int32_t reading = read_single_channel_fast(channel);
    sum += reading;
    
    if (i % 50 == 0) {
      Serial.printf("[CAL] Progress: %d/%d samples\n", i, num_samples);
    }
    delay(10);  // 10ms between samples for stability
  }
  
  int32_t average = sum / num_samples;
  
  // Store tare value
  cal_data.lc_cal[channel].b0_offset = average;
  cal_data.lc_cal[channel].timestamp = millis();
  
  Serial.printf("[CAL] Load Cell %d tare complete: %ld counts\n", 
                channel + 1, average);
  
  return true;
}

bool start_load_cell_span_calibration(uint8_t channel) {
  if (channel >= 4) return false;
  
  Serial.printf("[CAL] Starting span calibration for Load Cell %d\n", channel + 1);
  Serial.println("[CAL] Place known masses directly above this load cell");
  Serial.println("[CAL] Use at least 5 points: 0%, 25%, 50%, 75%, 100% of range");
  Serial.println("[CAL] Commands:");
  Serial.println("[CAL]   CAL_SPAN_ADD <mass_kg> - Add calibration point");
  Serial.println("[CAL]   CAL_SPAN_COMPUTE - Compute span coefficient");
  Serial.println("[CAL]   CAL_SPAN_CANCEL - Cancel span calibration");
  
  current_cal_state = CAL_STATE_LC_SPAN;
  current_cal_channel = channel;
  temp_data_count = 0;
  
  return true;
}

bool add_span_calibration_point(uint8_t channel, double mass_kg) {
  if (channel >= 4 || temp_data_count >= 20) return false;
  
  Serial.printf("[CAL] Adding span point: %.3f kg\n", mass_kg);
  Serial.println("[CAL] Collecting 100 samples...");
  
  int32_t sum = 0;
  const int num_samples = 100;
  
  for (int i = 0; i < num_samples; i++) {
    int32_t reading = read_single_channel_fast(channel);
    sum += reading;
    delay(5);
  }
  
  int32_t average = sum / num_samples;
  int32_t zeroed = average - cal_data.lc_cal[channel].b0_offset;
  
  temp_masses[temp_data_count] = mass_kg;
  temp_readings[channel][temp_data_count] = zeroed;
  temp_data_count++;
  
  Serial.printf("[CAL] Point added: %.3f kg = %ld counts (zeroed)\n", 
                mass_kg, zeroed);
  Serial.printf("[CAL] Total points: %d\n", temp_data_count);
  
  return true;
}

bool compute_span_calibration(uint8_t channel) {
  if (channel >= 4 || temp_data_count < 3) {
    Serial.println("[CAL] Need at least 3 calibration points");
    return false;
  }
  
  Serial.printf("[CAL] Computing span calibration for Load Cell %d\n", channel + 1);
  Serial.printf("[CAL] Using %d calibration points\n", temp_data_count);
  
  // Linear regression: mass = b1 * counts + b0
  // We want: counts per unit mass, so b1 = counts/mass
  double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
  
  for (int i = 0; i < temp_data_count; i++) {
    double mass = temp_masses[i];
    double counts = temp_readings[channel][i];
    
    sum_x += counts;
    sum_y += mass;
    sum_xy += counts * mass;
    sum_xx += counts * counts;
  }
  
  double n = temp_data_count;
  double slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
  double intercept = (sum_y - slope * sum_x) / n;
  
  // Convert to counts per kg (inverse of slope)
  double counts_per_kg = 1.0 / slope;
  
  // Calculate linearity error
  double max_error = 0;
  for (int i = 0; i < temp_data_count; i++) {
    double predicted_mass = slope * temp_readings[channel][i] + intercept;
    double error = fabs(predicted_mass - temp_masses[i]);
    if (error > max_error) max_error = error;
  }
  
  double linearity_percent = (max_error / temp_masses[temp_data_count-1]) * 100.0;
  
  // Store calibration data
  cal_data.lc_cal[channel].b1_span = counts_per_kg;
  cal_data.lc_cal[channel].linearity_error = linearity_percent;
  cal_data.lc_cal[channel].num_points = temp_data_count;
  
  // Copy mass data
  for (int i = 0; i < temp_data_count && i < 10; i++) {
    cal_data.lc_cal[channel].masses_used[i] = temp_masses[i];
  }
  
  cal_data.lc_cal[channel].valid = true;
  
  Serial.printf("[CAL] Span calibration complete:\n");
  Serial.printf("[CAL]   Span: %.2f counts/kg\n", counts_per_kg);
  Serial.printf("[CAL]   Linearity error: %.3f%%\n", linearity_percent);
  Serial.printf("[CAL]   Range: 0 - %.2f kg\n", temp_masses[temp_data_count-1]);
  
  current_cal_state = CAL_STATE_IDLE;
  temp_data_count = 0;
  
  return true;
}

bool perform_shunt_calibration(uint8_t channel) {
  if (channel >= 4) return false;
  
  Serial.printf("[CAL] Starting shunt calibration for Load Cell %d\n", channel + 1);
  Serial.println("[CAL] This requires hardware shunt resistor connection");
  Serial.println("[CAL] Measuring baseline reading...");
  
  // Get baseline reading (no shunt)
  int32_t baseline = 0;
  for (int i = 0; i < 50; i++) {
    baseline += read_single_channel_fast(channel);
    delay(10);
  }
  baseline /= 50;
  
  Serial.printf("[CAL] Baseline: %ld counts\n", baseline);
  Serial.println("[CAL] Now connect shunt resistor and press any key...");
  
  // Wait for user input (in real implementation, this would be automated)
  while (!Serial.available()) delay(100);
  Serial.read(); // Clear input
  
  // Get shunt reading
  int32_t shunt_reading = 0;
  for (int i = 0; i < 50; i++) {
    shunt_reading += read_single_channel_fast(channel);
    delay(10);
  }
  shunt_reading /= 50;
  
  int32_t shunt_delta = shunt_reading - baseline;
  
  // Store shunt calibration (factor would come from load cell datasheet)
  cal_data.lc_cal[channel].shunt_reading = shunt_delta;
  cal_data.lc_cal[channel].shunt_factor = 1.0; // Placeholder - use datasheet value
  
  Serial.printf("[CAL] Shunt calibration complete:\n");
  Serial.printf("[CAL]   Delta: %ld counts\n", shunt_delta);
  Serial.printf("[CAL]   Equivalent load: %.3f kg (using datasheet factor)\n", 
                shunt_delta / cal_data.lc_cal[channel].b1_span);
  
  return true;
}

// ============================================================================
// STEP C: MATRIX CALIBRATION FUNCTIONS
// ============================================================================

bool start_matrix_calibration() {
  Serial.println("[CAL] Starting whole-plate matrix calibration");
  Serial.println("[CAL] This calibration maps 4 load cell outputs to Fz, Mx, My");
  Serial.println("[CAL] You will need to place known masses at specific positions");
  Serial.println("[CAL] Recommended positions:");
  Serial.println("[CAL]   1. Center (0, 0)");
  Serial.println("[CAL]   2. Front center (0, +Y/2)");
  Serial.println("[CAL]   3. Back center (0, -Y/2)");
  Serial.println("[CAL]   4. Right center (+X/2, 0)");
  Serial.println("[CAL]   5. Left center (-X/2, 0)");
  Serial.println("[CAL]   6. Front-right corner (+X/2, +Y/2)");
  Serial.println("[CAL]   7. Front-left corner (-X/2, +Y/2)");
  Serial.println("[CAL]   8. Back-right corner (+X/2, -Y/2)");
  Serial.println("[CAL]   9. Back-left corner (-X/2, -Y/2)");
  Serial.println("[CAL] Commands:");
  Serial.println("[CAL]   CAL_MATRIX_ADD <mass_kg> <x_mm> <y_mm> - Add calibration point");
  Serial.println("[CAL]   CAL_MATRIX_COMPUTE - Compute matrix coefficients");
  Serial.println("[CAL]   CAL_MATRIX_CANCEL - Cancel matrix calibration");
  
  current_cal_state = CAL_STATE_MATRIX_COLLECT;
  temp_data_count = 0;
  
  return true;
}

bool add_matrix_calibration_point(double mass_kg, double x_mm, double y_mm) {
  if (temp_data_count >= 20) {
    Serial.println("[CAL] Maximum calibration points reached");
    return false;
  }
  
  Serial.printf("[CAL] Adding matrix point: %.3f kg at (%.1f, %.1f) mm\n", 
                mass_kg, x_mm, y_mm);
  Serial.println("[CAL] Collecting readings from all 4 load cells...");
  
  // Collect readings from all 4 load cells
  int32_t readings[4] = {0, 0, 0, 0};
  const int num_samples = 100;
  
  for (int sample = 0; sample < num_samples; sample++) {
    for (int ch = 0; ch < 4; ch++) {
      readings[ch] += read_single_channel_fast(ch);
      delay(2);
    }
  }
  
  // Average and apply per-cell calibration
  for (int ch = 0; ch < 4; ch++) {
    readings[ch] /= num_samples;
    // Convert to force using per-cell calibration
    int32_t zeroed = readings[ch] - cal_data.lc_cal[ch].b0_offset;
    temp_readings[ch][temp_data_count] = zeroed;
  }
  
  // Store position and mass
  temp_masses[temp_data_count] = mass_kg;
  temp_positions_x[temp_data_count] = x_mm;
  temp_positions_y[temp_data_count] = y_mm;
  temp_data_count++;
  
  Serial.printf("[CAL] Point added. Load cell readings (zeroed):\n");
  for (int ch = 0; ch < 4; ch++) {
    Serial.printf("[CAL]   LC%d: %ld counts\n", ch + 1, temp_readings[ch][temp_data_count-1]);
  }
  Serial.printf("[CAL] Total points: %d\n", temp_data_count);
  
  return true;
}

bool compute_matrix_calibration() {
  if (temp_data_count < 9) {
    Serial.println("[CAL] Need at least 9 calibration points for matrix calibration");
    return false;
  }
  
  Serial.printf("[CAL] Computing matrix calibration with %d points\n", temp_data_count);
  
  // Set up least squares problem: [A] * [x] = [b]
  // Where [x] contains the coefficients we want to solve for
  
  // For Fz: sum of all load cell forces should equal applied force
  // For Mx: moment about X axis = Fz * y_position  
  // For My: moment about Y axis = -Fz * x_position (sign depends on coordinate system)
  
  double A_fz[4] = {0, 0, 0, 0};  // Force coefficients
  double A_mx[4] = {0, 0, 0, 0};  // Moment X coefficients
  double A_my[4] = {0, 0, 0, 0};  // Moment Y coefficients
  
  // Simple approach: assume equal contribution from all cells for force
  // and position-weighted contribution for moments
  // This is a simplified version - full implementation would use proper least squares
  
  double total_force = 0;
  double total_moment_x = 0;
  double total_moment_y = 0;
  double total_lc_sum[4] = {0, 0, 0, 0};
  
  for (int i = 0; i < temp_data_count; i++) {
    double force = temp_masses[i] * 9.81; // Convert kg to N
    double moment_x = force * temp_positions_y[i] / 1000.0; // Convert mm to m
    double moment_y = -force * temp_positions_x[i] / 1000.0; // Convert mm to m
    
    total_force += force;
    total_moment_x += moment_x;
    total_moment_y += moment_y;
    
    for (int ch = 0; ch < 4; ch++) {
      total_lc_sum[ch] += temp_readings[ch][i];
    }
  }
  
  // Simple coefficient calculation (this is a basic approximation)
  for (int ch = 0; ch < 4; ch++) {
    if (total_lc_sum[ch] != 0) {
      A_fz[ch] = total_force / total_lc_sum[ch];
      A_mx[ch] = total_moment_x / total_lc_sum[ch];
      A_my[ch] = total_moment_y / total_lc_sum[ch];
    }
  }
  
  // Store matrix calibration
  for (int ch = 0; ch < 4; ch++) {
    cal_data.matrix_cal.a_fz[ch] = A_fz[ch];
    cal_data.matrix_cal.b_mx[ch] = A_mx[ch];
    cal_data.matrix_cal.c_my[ch] = A_my[ch];
  }
  
  cal_data.matrix_cal.num_positions = temp_data_count;
  cal_data.matrix_cal.gravity = 9.81;
  cal_data.matrix_cal.timestamp = millis();
  cal_data.matrix_cal.valid = true;
  
  Serial.println("[CAL] Matrix calibration complete:");
  Serial.println("[CAL] Force coefficients (a):");
  for (int ch = 0; ch < 4; ch++) {
    Serial.printf("[CAL]   a[%d] = %.6f\n", ch, A_fz[ch]);
  }
  Serial.println("[CAL] Moment X coefficients (b):");
  for (int ch = 0; ch < 4; ch++) {
    Serial.printf("[CAL]   b[%d] = %.6f\n", ch, A_mx[ch]);
  }
  Serial.println("[CAL] Moment Y coefficients (c):");
  for (int ch = 0; ch < 4; ch++) {
    Serial.printf("[CAL]   c[%d] = %.6f\n", ch, A_my[ch]);
  }
  
  current_cal_state = CAL_STATE_IDLE;
  temp_data_count = 0;
  
  return true;
}

// ============================================================================
// CALIBRATION DATA STORAGE AND MANAGEMENT
// ============================================================================

uint16_t calculate_calibration_crc() {
  return crc16_ccitt_false((uint8_t*)&cal_data, sizeof(CalibrationData) - 2);
}

bool save_calibration_to_eeprom() {
  Serial.println("[CAL] Saving calibration data to EEPROM...");
  
  // Update CRC
  cal_data.crc16 = calculate_calibration_crc();
  cal_data.cal_version = 1;
  cal_data.system_serial = 12345; // Should be unique per system
  
  // Write to EEPROM
  EEPROM.put(EEPROM_CAL_START_ADDR, cal_data);
  
  Serial.println("[CAL] Calibration data saved successfully");
  return true;
}

bool load_calibration_from_eeprom() {
  Serial.println("[CAL] Loading calibration data from EEPROM...");
  
  // Read from EEPROM
  EEPROM.get(EEPROM_CAL_START_ADDR, cal_data);
  
  // Verify CRC
  uint16_t calculated_crc = calculate_calibration_crc();
  if (cal_data.crc16 != calculated_crc) {
    Serial.println("[CAL] Calibration data CRC mismatch - data may be corrupted");
    return false;
  }
  
  Serial.printf("[CAL] Calibration data loaded (version %d, serial %lu)\n",
                cal_data.cal_version, cal_data.system_serial);
  
  calibration_loaded = true;
  return true;
}

void clear_calibration_data() {
  Serial.println("[CAL] Clearing all calibration data...");
  memset(&cal_data, 0, sizeof(CalibrationData));
  calibration_loaded = false;
  
  // Clear EEPROM
  for (unsigned int i = 0; i < EEPROM_CAL_SIZE; i++) {
    EEPROM.write(EEPROM_CAL_START_ADDR + i, 0xFF);
  }
  
  Serial.println("[CAL] Calibration data cleared");
}

void print_calibration_summary() {
  Serial.println("[CAL] ==========================================");
  Serial.println("[CAL] CALIBRATION SUMMARY");
  Serial.println("[CAL] ==========================================");
  
  // ADC Calibration
  if (cal_data.adc_cal.valid) {
    Serial.printf("[CAL] ADC: PGA=%d, DR=0x%02X, OFC=0x%06lX, FSC=0x%06lX\n",
                  cal_data.adc_cal.pga_setting, cal_data.adc_cal.data_rate,
                  cal_data.adc_cal.ofc_value, cal_data.adc_cal.fsc_value);
  } else {
    Serial.println("[CAL] ADC: Not calibrated");
  }
  
  // Load Cell Calibrations
  for (int ch = 0; ch < 4; ch++) {
    if (cal_data.lc_cal[ch].valid) {
      Serial.printf("[CAL] LC%d: Offset=%ld, Span=%.2f counts/kg, Error=%.3f%%\n",
                    ch + 1, cal_data.lc_cal[ch].b0_offset,
                    cal_data.lc_cal[ch].b1_span, cal_data.lc_cal[ch].linearity_error);
    } else {
      Serial.printf("[CAL] LC%d: Not calibrated\n", ch + 1);
    }
  }
  
  // Matrix Calibration
  if (cal_data.matrix_cal.valid) {
    Serial.printf("[CAL] Matrix: %d positions, Fz coeffs=[%.3f,%.3f,%.3f,%.3f]\n",
                  cal_data.matrix_cal.num_positions,
                  cal_data.matrix_cal.a_fz[0], cal_data.matrix_cal.a_fz[1],
                  cal_data.matrix_cal.a_fz[2], cal_data.matrix_cal.a_fz[3]);
  } else {
    Serial.println("[CAL] Matrix: Not calibrated");
  }
  
  Serial.println("[CAL] ==========================================");
}

// ============================================================================
// CALIBRATED MEASUREMENT FUNCTIONS
// ============================================================================

ForceData get_calibrated_force_data() {
  ForceData result = {0, 0, 0, 0, 0, false};
  
  if (!calibration_loaded || !cal_data.matrix_cal.valid) {
    return result;
  }
  
  // Read all 4 load cells
  int32_t raw_readings[4];
  for (int ch = 0; ch < 4; ch++) {
    raw_readings[ch] = read_single_channel_fast(ch);
  }
  
  // Apply per-cell calibration to get forces
  double cell_forces[4];
  for (int ch = 0; ch < 4; ch++) {
    if (cal_data.lc_cal[ch].valid) {
      int32_t zeroed = raw_readings[ch] - cal_data.lc_cal[ch].b0_offset;
      cell_forces[ch] = zeroed / cal_data.lc_cal[ch].b1_span * 9.81; // Convert kg to N
    } else {
      cell_forces[ch] = 0;
    }
  }
  
  // Apply matrix calibration
  result.fz = 0;
  result.mx = 0;
  result.my = 0;
  
  for (int ch = 0; ch < 4; ch++) {
    result.fz += cal_data.matrix_cal.a_fz[ch] * cell_forces[ch];
    result.mx += cal_data.matrix_cal.b_mx[ch] * cell_forces[ch];
    result.my += cal_data.matrix_cal.c_my[ch] * cell_forces[ch];
  }
  
  // Calculate center of pressure
  if (fabs(result.fz) > 1.0) { // Avoid division by zero
    result.cop_x = -result.my / result.fz * 1000.0; // Convert to mm
    result.cop_y = result.mx / result.fz * 1000.0;  // Convert to mm
    result.valid = true;
  }
  
  return result;
}
