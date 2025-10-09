// ============================================================================
// CALIBRATION VALIDATION AND TESTING TOOLS
// ============================================================================
// Provides comprehensive validation tools for force plate calibration
// including accuracy tests, repeatability tests, and diagnostic functions
// ============================================================================

// Include guard for ForceData structure (in case not already defined)
#ifndef FORCE_DATA_STRUCT_DEFINED
#define FORCE_DATA_STRUCT_DEFINED
struct ForceData {
  double fz;        // Vertical force (N)
  double mx;        // Moment about X axis (N⋅m)
  double my;        // Moment about Y axis (N⋅m)
  double cop_x;     // Center of pressure X (mm)
  double cop_y;     // Center of pressure Y (mm)
  bool valid;       // Data validity
};
#endif

// Forward declarations for int16 conversion functions (defined in main file)
int16_t force_to_int16_decigrams(double force_newtons);
int16_t moment_to_int16_scaled(double moment_nm);
int16_t cop_to_int16_mm(double cop_mm);

// ============================================================================
// VALIDATION TEST STRUCTURES
// ============================================================================

struct ValidationTest {
  char name[32];
  double expected_force;    // Expected force (N)
  double expected_cop_x;    // Expected COP X (mm)
  double expected_cop_y;    // Expected COP Y (mm)
  double measured_force;    // Measured force (N)
  double measured_cop_x;    // Measured COP X (mm)
  double measured_cop_y;    // Measured COP Y (mm)
  double force_error;       // Force error (%)
  double cop_error;         // COP error (mm)
  bool passed;              // Test passed flag
};

struct ValidationSuite {
  ValidationTest tests[20];
  uint8_t num_tests;
  uint8_t passed_tests;
  double max_force_error;
  double max_cop_error;
  double rms_force_error;
  double rms_cop_error;
};

static ValidationSuite validation_results;

// ============================================================================
// ACCURACY VALIDATION FUNCTIONS
// ============================================================================

bool start_accuracy_validation() {
  Serial.println("[VAL] ==========================================");
  Serial.println("[VAL] FORCE PLATE ACCURACY VALIDATION");
  Serial.println("[VAL] ==========================================");
  Serial.println("[VAL] This test validates calibration accuracy");
  Serial.println("[VAL] You will need calibrated masses and position measurements");
  Serial.println("[VAL] ");
  Serial.println("[VAL] Commands:");
  Serial.println("[VAL]   VAL_ADD_TEST <name> <mass_kg> <x_mm> <y_mm>");
  Serial.println("[VAL]   VAL_RUN_TESTS");
  Serial.println("[VAL]   VAL_SHOW_RESULTS");
  Serial.println("[VAL]   VAL_CLEAR_TESTS");
  
  // Clear previous results
  memset(&validation_results, 0, sizeof(ValidationSuite));
  
  return true;
}

bool add_validation_test(const char* name, double mass_kg, double x_mm, double y_mm) {
  if (validation_results.num_tests >= 20) {
    Serial.println("[VAL] Maximum number of tests reached (20)");
    return false;
  }
  
  ValidationTest* test = &validation_results.tests[validation_results.num_tests];
  
  strncpy(test->name, name, 31);
  test->name[31] = '\0';
  test->expected_force = mass_kg * 9.81; // Convert to Newtons
  test->expected_cop_x = x_mm;
  test->expected_cop_y = y_mm;
  
  Serial.printf("[VAL] Test added: %s (%.2f N at %.1f, %.1f mm)\n", 
                name, test->expected_force, x_mm, y_mm);
  
  validation_results.num_tests++;
  return true;
}

bool run_validation_tests() {
  if (validation_results.num_tests == 0) {
    Serial.println("[VAL] No tests defined. Add tests first.");
    return false;
  }
  
  Serial.printf("[VAL] Running %d validation tests...\n", validation_results.num_tests);
  Serial.println("[VAL] Place masses as instructed and press any key to continue");
  
  for (int i = 0; i < validation_results.num_tests; i++) {
    ValidationTest* test = &validation_results.tests[i];
    
    Serial.printf("[VAL] Test %d/%d: %s\n", i + 1, validation_results.num_tests, test->name);
    Serial.printf("[VAL] Place %.3f kg at position (%.1f, %.1f) mm\n", 
                  test->expected_force / 9.81, test->expected_cop_x, test->expected_cop_y);
    Serial.println("[VAL] Press any key when ready...");
    
    // Wait for user input
    while (!Serial.available()) delay(100);
    Serial.read(); // Clear input
    
    // Collect measurement data
    Serial.println("[VAL] Collecting measurement data...");
    
    double sum_fz = 0, sum_cop_x = 0, sum_cop_y = 0;
    int valid_samples = 0;
    const int num_samples = 50;
    
    for (int sample = 0; sample < num_samples; sample++) {
      ForceData data = get_calibrated_force_data();
      if (data.valid && data.fz > 1.0) { // Valid force reading
        sum_fz += data.fz;
        sum_cop_x += data.cop_x;
        sum_cop_y += data.cop_y;
        valid_samples++;
      }
      delay(20); // 20ms between samples
    }
    
    if (valid_samples < num_samples / 2) {
      Serial.printf("[VAL] Test %s FAILED: Insufficient valid samples (%d/%d)\n", 
                    test->name, valid_samples, num_samples);
      test->passed = false;
      continue;
    }
    
    // Calculate averages
    test->measured_force = sum_fz / valid_samples;
    test->measured_cop_x = sum_cop_x / valid_samples;
    test->measured_cop_y = sum_cop_y / valid_samples;
    
    // Calculate errors
    test->force_error = fabs(test->measured_force - test->expected_force) / test->expected_force * 100.0;
    test->cop_error = sqrt(pow(test->measured_cop_x - test->expected_cop_x, 2) + 
                          pow(test->measured_cop_y - test->expected_cop_y, 2));
    
    // Pass/fail criteria (adjust as needed)
    bool force_ok = test->force_error < 2.0; // 2% force error
    bool cop_ok = test->cop_error < 5.0;     // 5mm COP error
    test->passed = force_ok && cop_ok;
    
    if (test->passed) {
      validation_results.passed_tests++;
      Serial.printf("[VAL] Test %s PASSED\n", test->name);
    } else {
      Serial.printf("[VAL] Test %s FAILED (Force: %.2f%%, COP: %.2f mm)\n", 
                    test->name, test->force_error, test->cop_error);
    }
    
    Serial.printf("[VAL]   Expected: %.2f N at (%.1f, %.1f) mm\n", 
                  test->expected_force, test->expected_cop_x, test->expected_cop_y);
    Serial.printf("[VAL]   Measured: %.2f N at (%.1f, %.1f) mm\n", 
                  test->measured_force, test->measured_cop_x, test->measured_cop_y);
    Serial.println("[VAL] ");
  }
  
  // Calculate overall statistics
  calculate_validation_statistics();
  
  Serial.printf("[VAL] Validation complete: %d/%d tests passed\n", 
                validation_results.passed_tests, validation_results.num_tests);
  
  return true;
}

void calculate_validation_statistics() {
  double sum_force_error_sq = 0;
  double sum_cop_error_sq = 0;
  validation_results.max_force_error = 0;
  validation_results.max_cop_error = 0;
  
  for (int i = 0; i < validation_results.num_tests; i++) {
    ValidationTest* test = &validation_results.tests[i];
    
    sum_force_error_sq += test->force_error * test->force_error;
    sum_cop_error_sq += test->cop_error * test->cop_error;
    
    if (test->force_error > validation_results.max_force_error) {
      validation_results.max_force_error = test->force_error;
    }
    
    if (test->cop_error > validation_results.max_cop_error) {
      validation_results.max_cop_error = test->cop_error;
    }
  }
  
  validation_results.rms_force_error = sqrt(sum_force_error_sq / validation_results.num_tests);
  validation_results.rms_cop_error = sqrt(sum_cop_error_sq / validation_results.num_tests);
}

void show_validation_results() {
  Serial.println("[VAL] ==========================================");
  Serial.println("[VAL] VALIDATION RESULTS SUMMARY");
  Serial.println("[VAL] ==========================================");
  
  Serial.printf("[VAL] Tests passed: %d/%d (%.1f%%)\n", 
                validation_results.passed_tests, validation_results.num_tests,
                (double)validation_results.passed_tests / validation_results.num_tests * 100.0);
  
  Serial.printf("[VAL] Force accuracy:\n");
  Serial.printf("[VAL]   RMS error: %.3f%%\n", validation_results.rms_force_error);
  Serial.printf("[VAL]   Max error: %.3f%%\n", validation_results.max_force_error);
  
  Serial.printf("[VAL] COP accuracy:\n");
  Serial.printf("[VAL]   RMS error: %.2f mm\n", validation_results.rms_cop_error);
  Serial.printf("[VAL]   Max error: %.2f mm\n", validation_results.max_cop_error);
  
  Serial.println("[VAL] ");
  Serial.println("[VAL] Individual test results (int16 format only):");
  Serial.println("[VAL] Name                Force   COP(mm)   F.Err(%) C.Err(mm) Status");
  Serial.println("[VAL] ----------------------------------------------------------------");
  
  for (int i = 0; i < validation_results.num_tests; i++) {
    ValidationTest* test = &validation_results.tests[i];
    int16_t force_int16 = force_to_int16_decigrams(test->measured_force);
    int16_t cop_x_int16 = cop_to_int16_mm(test->measured_cop_x);
    int16_t cop_y_int16 = cop_to_int16_mm(test->measured_cop_y);
    
    Serial.printf("[VAL] %-18s %5d  %4d,%4d %6.2f   %6.2f    %s\n",
                  test->name, force_int16, cop_x_int16, cop_y_int16,
                  test->force_error, test->cop_error,
                  test->passed ? "PASS" : "FAIL");
  }
  
  Serial.println("[VAL] ==========================================");
}

// ============================================================================
// REPEATABILITY TESTING
// ============================================================================

bool run_repeatability_test(double mass_kg, double x_mm, double y_mm, int num_trials) {
  Serial.printf("[VAL] Starting repeatability test: %.3f kg at (%.1f, %.1f) mm\n", 
                mass_kg, x_mm, y_mm);
  Serial.printf("[VAL] Number of trials: %d\n", num_trials);
  
  if (num_trials > 50) {
    Serial.println("[VAL] Maximum 50 trials allowed");
    return false;
  }
  
  double force_readings[50];
  double cop_x_readings[50];
  double cop_y_readings[50];
  int valid_trials = 0;
  
  for (int trial = 0; trial < num_trials; trial++) {
    Serial.printf("[VAL] Trial %d/%d - Remove and replace mass, then press any key\n", 
                  trial + 1, num_trials);
    
    // Wait for user input
    while (!Serial.available()) delay(100);
    Serial.read(); // Clear input
    
    // Collect data
    Serial.println("[VAL] Measuring...");
    double sum_fz = 0, sum_cop_x = 0, sum_cop_y = 0;
    int valid_samples = 0;
    
    for (int sample = 0; sample < 20; sample++) {
      ForceData data = get_calibrated_force_data();
      if (data.valid && data.fz > 1.0) {
        sum_fz += data.fz;
        sum_cop_x += data.cop_x;
        sum_cop_y += data.cop_y;
        valid_samples++;
      }
      delay(50);
    }
    
    if (valid_samples >= 10) {
      force_readings[valid_trials] = sum_fz / valid_samples;
      cop_x_readings[valid_trials] = sum_cop_x / valid_samples;
      cop_y_readings[valid_trials] = sum_cop_y / valid_samples;
      valid_trials++;
      
      Serial.printf("[VAL] Trial %d: %.2f N at (%.1f, %.1f) mm\n", 
                    trial + 1, force_readings[valid_trials-1], 
                    cop_x_readings[valid_trials-1], cop_y_readings[valid_trials-1]);
    } else {
      Serial.printf("[VAL] Trial %d: Invalid (insufficient samples)\n", trial + 1);
    }
  }
  
  if (valid_trials < 3) {
    Serial.println("[VAL] Repeatability test failed: insufficient valid trials");
    return false;
  }
  
  // Calculate statistics
  double force_mean = 0, cop_x_mean = 0, cop_y_mean = 0;
  for (int i = 0; i < valid_trials; i++) {
    force_mean += force_readings[i];
    cop_x_mean += cop_x_readings[i];
    cop_y_mean += cop_y_readings[i];
  }
  force_mean /= valid_trials;
  cop_x_mean /= valid_trials;
  cop_y_mean /= valid_trials;
  
  double force_std = 0, cop_x_std = 0, cop_y_std = 0;
  for (int i = 0; i < valid_trials; i++) {
    force_std += pow(force_readings[i] - force_mean, 2);
    cop_x_std += pow(cop_x_readings[i] - cop_x_mean, 2);
    cop_y_std += pow(cop_y_readings[i] - cop_y_mean, 2);
  }
  force_std = sqrt(force_std / (valid_trials - 1));
  cop_x_std = sqrt(cop_x_std / (valid_trials - 1));
  cop_y_std = sqrt(cop_y_std / (valid_trials - 1));
  
  // Report results
  Serial.println("[VAL] ==========================================");
  Serial.println("[VAL] REPEATABILITY TEST RESULTS");
  Serial.println("[VAL] ==========================================");
  Serial.printf("[VAL] Valid trials: %d/%d\n", valid_trials, num_trials);
  Serial.printf("[VAL] Force (N):\n");
  Serial.printf("[VAL]   Mean: %.3f\n", force_mean);
  Serial.printf("[VAL]   Std Dev: %.3f (%.3f%%)\n", force_std, force_std/force_mean*100);
  Serial.printf("[VAL] COP X (mm):\n");
  Serial.printf("[VAL]   Mean: %.2f\n", cop_x_mean);
  Serial.printf("[VAL]   Std Dev: %.2f\n", cop_x_std);
  Serial.printf("[VAL] COP Y (mm):\n");
  Serial.printf("[VAL]   Mean: %.2f\n", cop_y_mean);
  Serial.printf("[VAL]   Std Dev: %.2f\n", cop_y_std);
  
  // Pass/fail criteria
  bool force_repeatable = (force_std / force_mean * 100) < 0.5; // 0.5% CV
  bool cop_repeatable = (cop_x_std < 2.0) && (cop_y_std < 2.0); // 2mm std dev
  
  Serial.printf("[VAL] Repeatability: %s\n", 
                (force_repeatable && cop_repeatable) ? "PASS" : "FAIL");
  Serial.println("[VAL] ==========================================");
  
  return force_repeatable && cop_repeatable;
}

// ============================================================================
// DIAGNOSTIC FUNCTIONS
// ============================================================================

void run_load_cell_diagnostics() {
  Serial.println("[DIAG] ==========================================");
  Serial.println("[DIAG] LOAD CELL DIAGNOSTICS");
  Serial.println("[DIAG] ==========================================");
  
  // Check zero stability
  Serial.println("[DIAG] Testing zero stability (no load)...");
  int32_t zero_readings[4][100];
  
  for (int sample = 0; sample < 100; sample++) {
    for (int ch = 0; ch < 4; ch++) {
      zero_readings[ch][sample] = read_single_channel_fast(ch);
    }
    delay(10);
  }
  
  for (int ch = 0; ch < 4; ch++) {
    int32_t sum = 0, min_val = INT32_MAX, max_val = INT32_MIN;
    for (int i = 0; i < 100; i++) {
      sum += zero_readings[ch][i];
      if (zero_readings[ch][i] < min_val) min_val = zero_readings[ch][i];
      if (zero_readings[ch][i] > max_val) max_val = zero_readings[ch][i];
    }
    int32_t mean = sum / 100;
    int32_t range = max_val - min_val;
    
    Serial.printf("[DIAG] LC%d Zero: Mean=%ld, Range=%ld counts", ch + 1, mean, range);
    if (range < 100) {
      Serial.println(" [GOOD]");
    } else if (range < 500) {
      Serial.println(" [FAIR]");
    } else {
      Serial.println(" [POOR - Check connections/noise]");
    }
  }
  
  // Check channel balance
  Serial.println("[DIAG] ");
  Serial.println("[DIAG] Testing channel balance...");
  Serial.println("[DIAG] Place uniform load across entire plate");
  Serial.println("[DIAG] Press any key when ready...");
  
  while (!Serial.available()) delay(100);
  Serial.read();
  
  int32_t loaded_readings[4] = {0, 0, 0, 0};
  for (int sample = 0; sample < 50; sample++) {
    for (int ch = 0; ch < 4; ch++) {
      loaded_readings[ch] += read_single_channel_fast(ch);
    }
    delay(20);
  }
  
  for (int ch = 0; ch < 4; ch++) {
    loaded_readings[ch] /= 50;
    int32_t zero_mean = 0;
    for (int i = 0; i < 100; i++) {
      zero_mean += zero_readings[ch][i];
    }
    zero_mean /= 100;
    
    int32_t delta = loaded_readings[ch] - zero_mean;
    Serial.printf("[DIAG] LC%d Load Response: %ld counts\n", ch + 1, delta);
  }
  
  // Check for balance
  int32_t total_response = 0;
  for (int ch = 0; ch < 4; ch++) {
    int32_t zero_mean = 0;
    for (int i = 0; i < 100; i++) {
      zero_mean += zero_readings[ch][i];
    }
    zero_mean /= 100;
    total_response += (loaded_readings[ch] - zero_mean);
  }
  
  double avg_response = total_response / 4.0;
  Serial.printf("[DIAG] Average response: %.1f counts\n", avg_response);
  
  for (int ch = 0; ch < 4; ch++) {
    int32_t zero_mean = 0;
    for (int i = 0; i < 100; i++) {
      zero_mean += zero_readings[ch][i];
    }
    zero_mean /= 100;
    
    double deviation = ((loaded_readings[ch] - zero_mean) - avg_response) / avg_response * 100.0;
    Serial.printf("[DIAG] LC%d Balance: %.2f%%", ch + 1, deviation);
    
    if (fabs(deviation) < 5.0) {
      Serial.println(" [GOOD]");
    } else if (fabs(deviation) < 15.0) {
      Serial.println(" [FAIR]");
    } else {
      Serial.println(" [POOR - Check mounting/calibration]");
    }
  }
  
  Serial.println("[DIAG] ==========================================");
}

void run_adc_diagnostics() {
  Serial.println("[DIAG] ==========================================");
  Serial.println("[DIAG] ADC DIAGNOSTICS");
  Serial.println("[DIAG] ==========================================");
  
  // Read all ADC registers
  Serial.println("[DIAG] ADC Register Status:");
  Serial.printf("[DIAG] STATUS: 0x%02X\n", GetRegisterValue(STATUS));
  Serial.printf("[DIAG] MUX:    0x%02X\n", GetRegisterValue(MUX));
  Serial.printf("[DIAG] ADCON:  0x%02X\n", GetRegisterValue(ADCON));
  Serial.printf("[DIAG] DRATE:  0x%02X\n", GetRegisterValue(DRATE));
  Serial.printf("[DIAG] IO:     0x%02X\n", GetRegisterValue(IO));
  
  // Read calibration registers
  uint32_t ofc = 0, fsc = 0;
  ofc |= ((uint32_t)GetRegisterValue(OFC2)) << 16;
  ofc |= ((uint32_t)GetRegisterValue(OFC1)) << 8;
  ofc |= GetRegisterValue(OFC0);
  
  fsc |= ((uint32_t)GetRegisterValue(FSC2)) << 16;
  fsc |= ((uint32_t)GetRegisterValue(FSC1)) << 8;
  fsc |= GetRegisterValue(FSC0);
  
  Serial.printf("[DIAG] OFC: 0x%06lX (%ld)\n", ofc, ofc);
  Serial.printf("[DIAG] FSC: 0x%06lX (%ld)\n", fsc, fsc);
  
  // Test DRDY functionality
  Serial.println("[DIAG] Testing DRDY signal...");
  uint32_t drdy_count = 0;
  uint32_t start_time = millis();
  
  while (millis() - start_time < 1000) {
    waitforDRDY();
    drdy_count++;
  }
  
  Serial.printf("[DIAG] DRDY frequency: %lu Hz", drdy_count);
  if (drdy_count > 25000 && drdy_count < 35000) {
    Serial.println(" [GOOD - ~30kHz expected]");
  } else {
    Serial.println(" [CHECK - Expected ~30kHz]");
  }
  
  Serial.println("[DIAG] ==========================================");
}
