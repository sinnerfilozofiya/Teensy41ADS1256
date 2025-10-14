// ============================================================================
// AUTOMATED CALIBRATION SYSTEM
// ============================================================================
// Provides guided, step-by-step automated calibration with user prompts
// and timing for the complete 3-step force plate calibration process
// ============================================================================

// ============================================================================
// AUTOMATED CALIBRATION STATE MANAGEMENT
// ============================================================================

enum AutoCalState {
  AUTO_CAL_IDLE = 0,
  AUTO_CAL_INTRO = 1,
  AUTO_CAL_STEP_A_INTRO = 2,
  AUTO_CAL_STEP_A_RUNNING = 3,
  AUTO_CAL_STEP_B_INTRO = 4,
  AUTO_CAL_STEP_B_TARE = 5,
  AUTO_CAL_STEP_B_SPAN_SETUP = 6,
  AUTO_CAL_STEP_B_SPAN_COLLECT = 7,
  AUTO_CAL_STEP_B_SPAN_COMPUTE = 8,
  AUTO_CAL_STEP_C_INTRO = 9,
  AUTO_CAL_STEP_C_COLLECT = 10,
  AUTO_CAL_STEP_C_COMPUTE = 11,
  AUTO_CAL_SAVE_VERIFY = 12,
  AUTO_CAL_COMPLETE = 13,
  AUTO_CAL_ERROR = 14
};

static AutoCalState auto_cal_state = AUTO_CAL_IDLE;
static uint32_t auto_cal_timer = 0;
static uint8_t auto_cal_current_cell = 0;
static uint8_t auto_cal_current_mass_point = 0;
static uint8_t auto_cal_current_position = 0;
static bool auto_cal_waiting_for_input = false;
static bool auto_cal_active = false;

// Calibration mass sequence (in kg) - now customizable
static double cal_masses[] = {0.0, 5.0, 10.0, 15.0, 20.0};
static uint8_t num_cal_masses = 5;
static const uint8_t max_cal_masses = 10;  // Maximum number of calibration masses

// Calibration configuration
struct CalibrationConfig {
  bool step_c_enabled;           // Whether Step C (matrix calibration) is enabled
  bool center_placement;         // Whether to place masses in center (true) or on corners (false)
  double custom_masses[10];      // Custom mass values
  uint8_t num_masses;           // Number of masses to use
  bool use_default_masses;      // Whether to use default mass sequence
};

static CalibrationConfig cal_config = {
  .step_c_enabled = true,        // Step C enabled by default (can be disabled)
  .center_placement = true,      // New default: place masses in center
  .custom_masses = {0.0, 5.0, 10.0, 15.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  .num_masses = 5,
  .use_default_masses = true
};

// Matrix calibration positions (x, y in mm)
struct CalPosition {
  double x, y;
  const char* description;
};

static const CalPosition matrix_positions[] = {
  {0.0, 0.0, "Center"},
  {0.0, 150.0, "Front Center"},
  {0.0, -150.0, "Back Center"},
  {150.0, 0.0, "Right Center"},
  {-150.0, 0.0, "Left Center"},
  {150.0, 150.0, "Front-Right Corner"},
  {-150.0, 150.0, "Front-Left Corner"},
  {150.0, -150.0, "Back-Right Corner"},
  {-150.0, -150.0, "Back-Left Corner"}
};
static const uint8_t num_matrix_positions = 9;

// ============================================================================
// CALIBRATION CONFIGURATION FUNCTIONS
// ============================================================================

void set_calibration_masses(double masses[], uint8_t count) {
  if (count > max_cal_masses) count = max_cal_masses;
  
  for (int i = 0; i < count && i < max_cal_masses; i++) {
    cal_config.custom_masses[i] = masses[i];
    cal_masses[i] = masses[i];
  }
  cal_config.num_masses = count;
  num_cal_masses = count;
  cal_config.use_default_masses = false;
  
  auto_cal_printf("[AUTO-CAL] ✓ Custom calibration masses set (%d masses)\n", count);
  auto_cal_println("[AUTO-CAL] Masses: ");
  for (int i = 0; i < count; i++) {
    auto_cal_printf("[AUTO-CAL]   %d. %.1f kg\n", i + 1, masses[i]);
  }
}

void reset_to_default_masses() {
  double default_masses[] = {0.0, 5.0, 10.0, 15.0, 20.0};
  for (int i = 0; i < 5; i++) {
    cal_config.custom_masses[i] = default_masses[i];
    cal_masses[i] = default_masses[i];
  }
  cal_config.num_masses = 5;
  num_cal_masses = 5;
  cal_config.use_default_masses = true;
  
  auto_cal_println("[AUTO-CAL] ✓ Reset to default calibration masses (0, 5, 10, 15, 20 kg)");
}

void enable_step_c(bool enabled) {
  cal_config.step_c_enabled = enabled;
  auto_cal_printf("[AUTO-CAL] ✓ Step C (Matrix Calibration) %s\n", 
                  enabled ? "ENABLED" : "DISABLED");
}

void set_center_placement(bool center) {
  cal_config.center_placement = center;
  auto_cal_printf("[AUTO-CAL] ✓ Mass placement: %s\n", 
                  center ? "CENTER of plate" : "INDIVIDUAL load cell corners");
}

void show_calibration_config() {
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 📋 CURRENT CALIBRATION CONFIGURATION");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_printf("[AUTO-CAL] Step C (Matrix Cal): %s\n", 
                  cal_config.step_c_enabled ? "ENABLED" : "DISABLED");
  auto_cal_printf("[AUTO-CAL] Mass Placement: %s\n", 
                  cal_config.center_placement ? "CENTER" : "CORNERS");
  auto_cal_printf("[AUTO-CAL] Number of Masses: %d\n", cal_config.num_masses);
  auto_cal_println("[AUTO-CAL] Calibration Masses:");
  for (int i = 0; i < cal_config.num_masses; i++) {
    auto_cal_printf("[AUTO-CAL]   %d. %.1f kg\n", i + 1, cal_config.custom_masses[i]);
  }
  auto_cal_printf("[AUTO-CAL] Using: %s masses\n", 
                  cal_config.use_default_masses ? "DEFAULT" : "CUSTOM");
  auto_cal_println("[AUTO-CAL] ==========================================");
}

// BLE-compatible configuration commands
void handle_cal_config_command(String command) {
  command.trim();
  command.toUpperCase();
  
  if (command.startsWith("CAL_CONFIG_MASSES ")) {
    // Parse: CAL_CONFIG_MASSES 5.0,10.0,15.0,20.0
    String masses_str = command.substring(18);
    masses_str.trim();
    
    double new_masses[max_cal_masses];
    uint8_t count = 0;
    
    // Always include 0.0 as first mass
    new_masses[0] = 0.0;
    count = 1;
    
    // Parse comma-separated masses
    int start = 0;
    int comma_pos = masses_str.indexOf(',');
    
    while (comma_pos != -1 && count < max_cal_masses) {
      String mass_str = masses_str.substring(start, comma_pos);
      mass_str.trim();
      new_masses[count] = mass_str.toFloat();
      count++;
      start = comma_pos + 1;
      comma_pos = masses_str.indexOf(',', start);
    }
    
    // Get the last mass
    if (start < (int)masses_str.length() && count < max_cal_masses) {
      String mass_str = masses_str.substring(start);
      mass_str.trim();
      new_masses[count] = mass_str.toFloat();
      count++;
    }
    
    if (count > 1) {
      set_calibration_masses(new_masses, count);
    } else {
      auto_cal_println("[AUTO-CAL] ❌ Invalid mass format. Use: CAL_CONFIG_MASSES 5.0,10.0,15.0,20.0");
    }
  }
  else if (command == "CAL_CONFIG_RESET_MASSES") {
    reset_to_default_masses();
  }
  else if (command == "CAL_CONFIG_ENABLE_STEP_C") {
    enable_step_c(true);
  }
  else if (command == "CAL_CONFIG_DISABLE_STEP_C") {
    enable_step_c(false);
  }
  else if (command == "CAL_CONFIG_CENTER_PLACEMENT") {
    set_center_placement(true);
  }
  else if (command == "CAL_CONFIG_CORNER_PLACEMENT") {
    set_center_placement(false);
  }
  else if (command == "CAL_CONFIG_SHOW") {
    show_calibration_config();
  }
  else {
    auto_cal_println("[AUTO-CAL] ❌ Unknown config command");
    auto_cal_println("[AUTO-CAL] Available commands:");
    auto_cal_println("[AUTO-CAL]   CAL_CONFIG_MASSES <mass1,mass2,mass3,...>");
    auto_cal_println("[AUTO-CAL]   CAL_CONFIG_RESET_MASSES");
    auto_cal_println("[AUTO-CAL]   CAL_CONFIG_ENABLE_STEP_C");
    auto_cal_println("[AUTO-CAL]   CAL_CONFIG_DISABLE_STEP_C");
    auto_cal_println("[AUTO-CAL]   CAL_CONFIG_CENTER_PLACEMENT");
    auto_cal_println("[AUTO-CAL]   CAL_CONFIG_CORNER_PLACEMENT");
    auto_cal_println("[AUTO-CAL]   CAL_CONFIG_SHOW");
  }
}

// ============================================================================
// AUTOMATED CALIBRATION FUNCTIONS
// ============================================================================

void start_automated_calibration() {
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 🚀 AUTOMATED FORCE PLATE CALIBRATION");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] This automated system will guide you through");
  auto_cal_println("[AUTO-CAL] the flexible calibration process:");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📋 STEP A: ADC (ADS1256) Calibration");
  auto_cal_println("[AUTO-CAL] 📋 STEP B: Load Cell Calibration (CENTER placement)");
  if (cal_config.step_c_enabled) {
    auto_cal_println("[AUTO-CAL] 📋 STEP C: Matrix Calibration (OPTIONAL)");
  } else {
    auto_cal_println("[AUTO-CAL] 📋 STEP C: Matrix Calibration (DISABLED)");
  }
  auto_cal_println("[AUTO-CAL] ");
  
  // Show current configuration
  auto_cal_println("[AUTO-CAL] 📋 CURRENT CONFIGURATION:");
  auto_cal_printf("[AUTO-CAL]   Mass Placement: %s\n", 
                  cal_config.center_placement ? "CENTER of plate" : "Individual corners");
  auto_cal_printf("[AUTO-CAL]   Step C (Matrix): %s\n", 
                  cal_config.step_c_enabled ? "ENABLED" : "DISABLED");
  auto_cal_printf("[AUTO-CAL]   Calibration Masses (%d): ", cal_config.num_masses);
  for (int i = 0; i < cal_config.num_masses; i++) {
    if (i > 0) auto_cal_printf(", ");
    auto_cal_printf("%.1fkg", cal_config.custom_masses[i]);
  }
  auto_cal_println("");
  auto_cal_println("[AUTO-CAL] ");
  
  // Estimate time based on configuration
  int estimated_time = 30; // Step A: 30 minutes base
  if (cal_config.center_placement) {
    estimated_time += 15; // Step B center: 15 minutes
  } else {
    estimated_time += 40; // Step B corners: 40 minutes
  }
  if (cal_config.step_c_enabled) {
    estimated_time += 25; // Step C: 25 minutes
  }
  
  auto_cal_printf("[AUTO-CAL] ⏱️  ESTIMATED TIME: %d-%d minutes\n", estimated_time, estimated_time + 15);
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📦 EQUIPMENT NEEDED:");
  auto_cal_printf("[AUTO-CAL]   • Calibrated masses: ");
  for (int i = 1; i < cal_config.num_masses; i++) { // Skip 0kg
    if (i > 1) auto_cal_printf(", ");
    auto_cal_printf("%.1fkg", cal_config.custom_masses[i]);
  }
  auto_cal_println("");
  if (cal_config.step_c_enabled) {
    auto_cal_println("[AUTO-CAL]   • Measuring tape/ruler (for Step C)");
  }
  auto_cal_println("[AUTO-CAL]   • Level, stable surface");
  auto_cal_println("[AUTO-CAL]   • Patience and attention to detail");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 🎯 At each step, you can:");
  auto_cal_println("[AUTO-CAL]   • Type 'CONTINUE' to proceed");
  auto_cal_println("[AUTO-CAL]   • Type 'SKIP' to skip current step");
  auto_cal_println("[AUTO-CAL]   • Type 'ABORT' to cancel calibration");
  auto_cal_println("[AUTO-CAL]   • Type 'STATUS' to see current progress");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 💡 To change configuration, use:");
  auto_cal_println("[AUTO-CAL]   • 'CAL_CONFIG_SHOW' - Show current settings");
  auto_cal_println("[AUTO-CAL]   • 'CAL_CONFIG_MASSES 5,10,15' - Set custom masses");
  auto_cal_println("[AUTO-CAL]   • 'CAL_CONFIG_DISABLE_STEP_C' - Skip matrix calibration");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] Ready to begin? Type 'CONTINUE' to start...");
  
  auto_cal_state = AUTO_CAL_INTRO;
  auto_cal_timer = millis();
  auto_cal_waiting_for_input = true;
  auto_cal_active = true;
  auto_cal_current_cell = 0;
  auto_cal_current_mass_point = 0;
  auto_cal_current_position = 0;
}

void handle_auto_cal_input(String command) {
  if (!auto_cal_active) return;
  
  command.trim();
  command.toUpperCase();
  
  // Handle configuration commands even during calibration
  if (command.startsWith("CAL_CONFIG_")) {
    handle_cal_config_command(command);
    return;
  }
  
  if (command == "ABORT") {
    auto_cal_println("[AUTO-CAL] ❌ Calibration aborted by user");
    auto_cal_state = AUTO_CAL_IDLE;
    auto_cal_active = false;
    auto_cal_waiting_for_input = false;
    return;
  }
  
  if (command == "STATUS") {
    show_auto_cal_status();
    return;
  }
  
  if (!auto_cal_waiting_for_input) {
    auto_cal_println("[AUTO-CAL] ⚠️ Please wait for the current step to complete");
    return;
  }
  
  if (command == "CONTINUE") {
    auto_cal_waiting_for_input = false;
    
    // Special handling for span collection
    if (auto_cal_state == AUTO_CAL_STEP_B_SPAN_COLLECT) {
      handle_step_b_span_collect_continue();
    } else if (auto_cal_state == AUTO_CAL_STEP_C_COLLECT) {
      handle_step_c_collect_continue();
    } else {
      advance_auto_calibration();
    }
  } else if (command == "SKIP") {
    auto_cal_println("[AUTO-CAL] ⏭️ Skipping current step...");
    auto_cal_waiting_for_input = false;
    skip_current_step();
  } else {
    auto_cal_println("[AUTO-CAL] ❓ Unknown command. Use: CONTINUE, SKIP, ABORT, STATUS, or CAL_CONFIG_*");
  }
}

void show_auto_cal_status() {
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 📊 CALIBRATION PROGRESS STATUS");
  auto_cal_println("[AUTO-CAL] ==========================================");
  
  const char* state_names[] = {
    "Idle", "Introduction", "Step A Intro", "Step A Running",
    "Step B Intro", "Step B Tare", "Step B Span Setup", "Step B Span Collect",
    "Step B Span Compute", "Step C Intro", "Step C Collect", "Step C Compute",
    "Save & Verify", "Complete", "Error"
  };
  
  auto_cal_printf("[AUTO-CAL] Current State: %s\n", state_names[auto_cal_state]);
  
  if (auto_cal_state >= AUTO_CAL_STEP_B_TARE && auto_cal_state <= AUTO_CAL_STEP_B_SPAN_COMPUTE) {
    auto_cal_printf("[AUTO-CAL] Current Load Cell: %d/4\n", auto_cal_current_cell + 1);
    if (auto_cal_state >= AUTO_CAL_STEP_B_SPAN_COLLECT) {
      auto_cal_printf("[AUTO-CAL] Current Mass Point: %d/%d\n", auto_cal_current_mass_point + 1, num_cal_masses);
    }
  }
  
  if (auto_cal_state >= AUTO_CAL_STEP_C_COLLECT && auto_cal_state <= AUTO_CAL_STEP_C_COMPUTE) {
    auto_cal_printf("[AUTO-CAL] Current Position: %d/%d\n", auto_cal_current_position + 1, num_matrix_positions);
  }
  
  auto_cal_printf("[AUTO-CAL] Waiting for input: %s\n", auto_cal_waiting_for_input ? "YES" : "NO");
  auto_cal_println("[AUTO-CAL] ==========================================");
}

void advance_auto_calibration() {
  switch (auto_cal_state) {
    case AUTO_CAL_INTRO:
      start_step_a_intro();
      break;
      
    case AUTO_CAL_STEP_A_INTRO:
      start_step_a_execution();
      break;
      
    case AUTO_CAL_STEP_A_RUNNING:
      start_step_b_intro();
      break;
      
    case AUTO_CAL_STEP_B_INTRO:
      start_step_b_tare();
      break;
      
    case AUTO_CAL_STEP_B_TARE:
      start_step_b_span_setup();
      break;
      
    case AUTO_CAL_STEP_B_SPAN_SETUP:
      start_step_b_span_collect();
      break;
      
    case AUTO_CAL_STEP_B_SPAN_COLLECT:
      handle_step_b_span_collect();
      break;
      
    case AUTO_CAL_STEP_B_SPAN_COMPUTE:
      handle_step_b_span_compute();
      break;
      
    case AUTO_CAL_STEP_C_INTRO:
      start_step_c_collect();
      break;
      
    case AUTO_CAL_STEP_C_COLLECT:
      handle_step_c_collect();
      break;
      
    case AUTO_CAL_STEP_C_COMPUTE:
      start_save_verify();
      break;
      
    case AUTO_CAL_SAVE_VERIFY:
      complete_calibration();
      break;
      
    default:
      auto_cal_println("[AUTO-CAL] ❌ Invalid state");
      auto_cal_state = AUTO_CAL_ERROR;
      break;
  }
}

void skip_current_step() {
  auto_cal_println("[AUTO-CAL] ⏭️ Step skipped - moving to next phase");
  
  switch (auto_cal_state) {
    case AUTO_CAL_STEP_A_INTRO:
    case AUTO_CAL_STEP_A_RUNNING:
      start_step_b_intro();
      break;
      
    case AUTO_CAL_STEP_B_INTRO:
    case AUTO_CAL_STEP_B_TARE:
    case AUTO_CAL_STEP_B_SPAN_SETUP:
    case AUTO_CAL_STEP_B_SPAN_COLLECT:
    case AUTO_CAL_STEP_B_SPAN_COMPUTE:
      auto_cal_state = AUTO_CAL_STEP_C_INTRO;
      start_step_c_intro();
      break;
      
    case AUTO_CAL_STEP_C_INTRO:
    case AUTO_CAL_STEP_C_COLLECT:
    case AUTO_CAL_STEP_C_COMPUTE:
      start_save_verify();
      break;
      
    default:
      advance_auto_calibration();
      break;
  }
}

// ============================================================================
// STEP A: ADC CALIBRATION
// ============================================================================

void start_step_a_intro() {
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 📋 STEP A: ADC (ADS1256) CALIBRATION");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 🎯 PURPOSE:");
  auto_cal_println("[AUTO-CAL]   Calibrate the ADS1256 ADC for accurate voltage");
  auto_cal_println("[AUTO-CAL]   measurements at your current PGA and data rate");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] ⏱️  DURATION: ~30 seconds");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📋 WHAT WILL HAPPEN:");
  auto_cal_println("[AUTO-CAL]   • SELFOCAL: Offset calibration");
  auto_cal_println("[AUTO-CAL]   • SELFGCAL: Gain calibration");
  auto_cal_println("[AUTO-CAL]   • Store OFC and FSC register values");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] ✅ PREPARATION:");
  auto_cal_println("[AUTO-CAL]   • Ensure stable power supply");
  auto_cal_println("[AUTO-CAL]   • No external loads on ADC inputs");
  auto_cal_println("[AUTO-CAL]   • System has been running for >2 minutes");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] Ready to start ADC calibration?");
  auto_cal_println("[AUTO-CAL] Type 'CONTINUE' to proceed...");
  
  auto_cal_state = AUTO_CAL_STEP_A_INTRO;
  auto_cal_waiting_for_input = true;
}

void start_step_a_execution() {
  auto_cal_println("[AUTO-CAL] 🔄 Starting ADC self-calibration...");
  auto_cal_println("[AUTO-CAL] Please wait - this may take up to 30 seconds...");
  
  auto_cal_state = AUTO_CAL_STEP_A_RUNNING;
  auto_cal_waiting_for_input = false;
  auto_cal_timer = millis();
  
  // Perform ADC calibration
  if (perform_adc_self_calibration(PGA_64, DR_30000)) {
    auto_cal_println("[AUTO-CAL] ✅ ADC calibration completed successfully!");
    auto_cal_println("[AUTO-CAL] ");
    delay(2000); // Give user time to read
    start_step_b_intro();
  } else {
    auto_cal_println("[AUTO-CAL] ❌ ADC calibration failed!");
    auto_cal_println("[AUTO-CAL] Check connections and power supply");
    auto_cal_println("[AUTO-CAL] Type 'CONTINUE' to retry or 'SKIP' to continue anyway...");
    auto_cal_waiting_for_input = true;
  }
}

// ============================================================================
// STEP B: LOAD CELL CALIBRATION
// ============================================================================

void start_step_b_intro() {
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 📋 STEP B: LOAD CELL CALIBRATION");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 🎯 PURPOSE:");
  if (cal_config.center_placement) {
    auto_cal_println("[AUTO-CAL]   Calibrate all 4 load cells simultaneously");
    auto_cal_println("[AUTO-CAL]   using masses placed at the CENTER of the plate");
  } else {
    auto_cal_println("[AUTO-CAL]   Calibrate each of the 4 load cells individually");
    auto_cal_println("[AUTO-CAL]   for accurate force measurements");
  }
  auto_cal_println("[AUTO-CAL] ");
  
  // Estimate duration based on placement method
  if (cal_config.center_placement) {
    auto_cal_println("[AUTO-CAL] ⏱️  DURATION: ~15-20 minutes");
  } else {
    auto_cal_println("[AUTO-CAL] ⏱️  DURATION: ~30-40 minutes");
  }
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📋 WHAT WILL HAPPEN:");
  auto_cal_println("[AUTO-CAL]   1. Tare (zero) all 4 load cells");
  
  if (cal_config.center_placement) {
    auto_cal_printf("[AUTO-CAL]   2. Place masses in CENTER (%d mass points)\n", cal_config.num_masses);
    auto_cal_println("[AUTO-CAL]   3. Record all 4 load cell responses simultaneously");
    auto_cal_println("[AUTO-CAL]   4. Calculate calibration coefficients for all cells");
  } else {
    auto_cal_printf("[AUTO-CAL]   2. Span calibration for each cell (%d mass points)\n", cal_config.num_masses);
    auto_cal_println("[AUTO-CAL]   3. Linearity analysis and coefficient calculation");
  }
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📦 MASSES NEEDED:");
  auto_cal_printf("[AUTO-CAL]   • ");
  for (int i = 1; i < cal_config.num_masses; i++) { // Skip 0kg
    if (i > 1) auto_cal_printf(", ");
    auto_cal_printf("%.1fkg", cal_config.custom_masses[i]);
  }
  auto_cal_println(" calibrated masses");
  
  if (cal_config.center_placement) {
    auto_cal_println("[AUTO-CAL]   • Ability to place masses at the CENTER of the plate");
  } else {
    auto_cal_println("[AUTO-CAL]   • Ability to place masses directly above each corner");
  }
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] ⚠️  IMPORTANT:");
  auto_cal_println("[AUTO-CAL]   • Remove ALL loads from the plate before starting");
  
  if (cal_config.center_placement) {
    auto_cal_println("[AUTO-CAL]   • Place masses at the EXACT CENTER of the plate");
    auto_cal_println("[AUTO-CAL]   • This method calibrates all 4 load cells together");
    auto_cal_println("[AUTO-CAL]   • Much faster than individual corner calibration");
  } else {
    auto_cal_println("[AUTO-CAL]   • Place masses as centrally as possible above each corner");
    auto_cal_println("[AUTO-CAL]   • Avoid placing masses between load cells");
  }
  auto_cal_println("[AUTO-CAL]   • Wait for stable readings between mass changes");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] Ready to start load cell calibration?");
  auto_cal_println("[AUTO-CAL] Type 'CONTINUE' to proceed...");
  
  auto_cal_state = AUTO_CAL_STEP_B_INTRO;
  auto_cal_waiting_for_input = true;
}

void start_step_b_tare() {
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 🔄 STEP B1: TARE (ZERO) ALL LOAD CELLS");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] ✅ ENSURE:");
  auto_cal_println("[AUTO-CAL]   • NO masses or objects on the force plate");
  auto_cal_println("[AUTO-CAL]   • Plate is level and stable");
  auto_cal_println("[AUTO-CAL]   • No external forces applied");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 🔄 Taring all 4 load cells...");
  
  auto_cal_state = AUTO_CAL_STEP_B_TARE;
  auto_cal_waiting_for_input = false;
  
  // Tare all load cells
  bool all_tared = true;
  for (int i = 0; i < 4; i++) {
    auto_cal_printf("[AUTO-CAL] Taring Load Cell %d...\n", i + 1);
    if (!start_load_cell_tare(i)) {
      auto_cal_printf("[AUTO-CAL] ❌ Failed to tare Load Cell %d\n", i + 1);
      all_tared = false;
    }
    delay(500);
  }
  
  if (all_tared) {
    auto_cal_println("[AUTO-CAL] ✅ All load cells tared successfully!");
    auto_cal_println("[AUTO-CAL] ");
    delay(2000);
    start_step_b_span_setup();
  } else {
    auto_cal_println("[AUTO-CAL] ❌ Some load cells failed to tare");
    auto_cal_println("[AUTO-CAL] Type 'CONTINUE' to retry or 'SKIP' to continue anyway...");
    auto_cal_waiting_for_input = true;
  }
}

void start_step_b_span_setup() {
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 🔄 STEP B2: SPAN CALIBRATION SETUP");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📋 SPAN CALIBRATION PROCESS:");
  
  if (cal_config.center_placement) {
    auto_cal_printf("[AUTO-CAL]   We will calibrate all load cells with %d mass points:\n", cal_config.num_masses);
    for (int i = 0; i < cal_config.num_masses; i++) {
      auto_cal_printf("[AUTO-CAL]   • %.1fkg", cal_config.custom_masses[i]);
      if (cal_config.custom_masses[i] == 0.0) {
        auto_cal_printf(" (no load)");
      } else {
        auto_cal_printf(" mass");
      }
      auto_cal_println("");
    }
    auto_cal_println("[AUTO-CAL] ");
    auto_cal_println("[AUTO-CAL] 🎯 CENTER PLACEMENT METHOD:");
    auto_cal_println("[AUTO-CAL]   1. Place mass at the CENTER of the plate");
    auto_cal_println("[AUTO-CAL]   2. Wait for reading to stabilize (~10 seconds)");
    auto_cal_println("[AUTO-CAL]   3. System records ALL 4 load cell responses");
    auto_cal_println("[AUTO-CAL]   4. Remove mass and prepare for next weight");
    auto_cal_println("[AUTO-CAL] ");
    auto_cal_println("[AUTO-CAL] ⚠️  CRITICAL:");
    auto_cal_println("[AUTO-CAL]   • Place masses at the EXACT CENTER of the plate");
    auto_cal_println("[AUTO-CAL]   • All 4 load cells will share the load equally");
    auto_cal_println("[AUTO-CAL]   • This method is faster and requires fewer masses");
  } else {
    auto_cal_printf("[AUTO-CAL]   We will calibrate each load cell with %d mass points:\n", cal_config.num_masses);
    for (int i = 0; i < cal_config.num_masses; i++) {
      auto_cal_printf("[AUTO-CAL]   • %.1fkg", cal_config.custom_masses[i]);
      if (cal_config.custom_masses[i] == 0.0) {
        auto_cal_printf(" (no load)");
      } else {
        auto_cal_printf(" mass");
      }
      auto_cal_println("");
    }
    auto_cal_println("[AUTO-CAL] ");
    auto_cal_println("[AUTO-CAL] 🎯 FOR EACH LOAD CELL:");
    auto_cal_println("[AUTO-CAL]   1. Place mass directly above the corner");
    auto_cal_println("[AUTO-CAL]   2. Wait for reading to stabilize (~10 seconds)");
    auto_cal_println("[AUTO-CAL]   3. System will automatically record the reading");
    auto_cal_println("[AUTO-CAL]   4. Remove mass and prepare for next point");
    auto_cal_println("[AUTO-CAL] ");
    auto_cal_println("[AUTO-CAL] ⚠️  CRITICAL:");
    auto_cal_println("[AUTO-CAL]   • Place masses as close to the load cell as possible");
    auto_cal_println("[AUTO-CAL]   • Avoid placing masses between load cells");
  }
  
  auto_cal_println("[AUTO-CAL]   • Wait for 'READY FOR NEXT' message before changing masses");
  auto_cal_println("[AUTO-CAL] ");
  
  if (cal_config.center_placement) {
    auto_cal_println("[AUTO-CAL] Ready to start CENTER calibration?");
    auto_cal_println("[AUTO-CAL] Type 'CONTINUE' to begin...");
  } else {
    auto_cal_println("[AUTO-CAL] Ready to start span calibration?");
    auto_cal_println("[AUTO-CAL] Type 'CONTINUE' to begin with Load Cell 1...");
  }
  
  auto_cal_state = AUTO_CAL_STEP_B_SPAN_SETUP;
  auto_cal_waiting_for_input = true;
  auto_cal_current_cell = 0;
  auto_cal_current_mass_point = 0;
}

void start_step_b_span_collect() {
  if (cal_config.center_placement) {
    // Center placement: calibrate all cells simultaneously
    auto_cal_current_mass_point = 0;
    auto_cal_state = AUTO_CAL_STEP_B_SPAN_COLLECT;
    
    auto_cal_println("[AUTO-CAL] ==========================================");
    auto_cal_println("[AUTO-CAL] 🔄 CENTER PLACEMENT CALIBRATION");
    auto_cal_println("[AUTO-CAL] ==========================================");
    
    // Start center calibration for all load cells
    start_center_calibration();
    handle_step_b_span_collect();
  } else {
    // Corner placement: calibrate each cell individually (original method)
    if (auto_cal_current_cell >= 4) {
      // All cells done, move to compute phase
      auto_cal_current_cell = 0;
      auto_cal_state = AUTO_CAL_STEP_B_SPAN_COMPUTE;
      handle_step_b_span_compute();
      return;
    }
    
    // Start span calibration for current cell
    start_load_cell_span_calibration(auto_cal_current_cell);
    auto_cal_current_mass_point = 0;
    auto_cal_state = AUTO_CAL_STEP_B_SPAN_COLLECT;
    
    auto_cal_println("[AUTO-CAL] ==========================================");
    auto_cal_printf("[AUTO-CAL] 🔄 LOAD CELL %d SPAN CALIBRATION\n", auto_cal_current_cell + 1);
    auto_cal_println("[AUTO-CAL] ==========================================");
    
    // Start with first mass point
    handle_step_b_span_collect();
  }
}

void handle_step_b_span_collect() {
  if (cal_config.center_placement) {
    // Center placement logic
    if (auto_cal_current_mass_point >= num_cal_masses) {
      // All mass points done, compute coefficients for all cells
      auto_cal_println("[AUTO-CAL] 🔄 Computing span coefficients for all load cells...");
      
      bool all_computed = true;
      for (int cell = 0; cell < 4; cell++) {
        if (!compute_center_calibration(cell)) {
          auto_cal_printf("[AUTO-CAL] ❌ Load Cell %d center calibration failed!\n", cell + 1);
          all_computed = false;
        }
      }
      
      if (all_computed) {
        auto_cal_println("[AUTO-CAL] ✅ All load cells center calibration complete!");
      } else {
        auto_cal_println("[AUTO-CAL] ⚠️ Some load cells had calibration issues");
      }
      
      delay(2000);
      // Move to Step C or complete
      if (cal_config.step_c_enabled) {
        start_step_c_intro();
      } else {
        start_save_verify();
      }
      return;
    }
    
    double current_mass = cal_masses[auto_cal_current_mass_point];
    
    auto_cal_println("[AUTO-CAL] ");
    auto_cal_printf("[AUTO-CAL] 📦 MASS POINT %d/%d: %.1f kg\n", 
                  auto_cal_current_mass_point + 1, num_cal_masses, current_mass);
    
    if (current_mass == 0.0) {
      auto_cal_println("[AUTO-CAL] 🔄 Remove all masses from the plate");
    } else {
      auto_cal_printf("[AUTO-CAL] 🔄 Place %.1f kg at the CENTER of the plate\n", current_mass);
    }
    
    auto_cal_println("[AUTO-CAL] Wait for reading to stabilize, then type 'CONTINUE'...");
    auto_cal_waiting_for_input = true;
  } else {
    // Corner placement logic (original)
    if (auto_cal_current_mass_point >= num_cal_masses) {
      // All mass points for this cell done, compute coefficients
      auto_cal_printf("[AUTO-CAL] 🔄 Computing span coefficients for Load Cell %d...\n", auto_cal_current_cell + 1);
      
      if (compute_span_calibration(auto_cal_current_cell)) {
        auto_cal_printf("[AUTO-CAL] ✅ Load Cell %d span calibration complete!\n", auto_cal_current_cell + 1);
      } else {
        auto_cal_printf("[AUTO-CAL] ❌ Load Cell %d span calibration failed!\n", auto_cal_current_cell + 1);
      }
      
      auto_cal_current_cell++;
      auto_cal_current_mass_point = 0;
      
      if (auto_cal_current_cell < 4) {
        auto_cal_println("[AUTO-CAL] ");
        auto_cal_printf("[AUTO-CAL] 📋 Moving to Load Cell %d...\n", auto_cal_current_cell + 1);
        auto_cal_println("[AUTO-CAL] Type 'CONTINUE' when ready...");
        auto_cal_waiting_for_input = true;
      } else {
        // All cells done
        delay(2000);
        if (cal_config.step_c_enabled) {
          start_step_c_intro();
        } else {
          start_save_verify();
        }
      }
      return;
    }
    
    double current_mass = cal_masses[auto_cal_current_mass_point];
    
    auto_cal_println("[AUTO-CAL] ");
    auto_cal_printf("[AUTO-CAL] 📦 MASS POINT %d/%d: %.1f kg\n", 
                  auto_cal_current_mass_point + 1, num_cal_masses, current_mass);
    
    if (current_mass == 0.0) {
      auto_cal_printf("[AUTO-CAL] 🔄 Remove all masses from Load Cell %d area\n", auto_cal_current_cell + 1);
    } else {
      auto_cal_printf("[AUTO-CAL] 🔄 Place %.1f kg directly above Load Cell %d\n", 
                    current_mass, auto_cal_current_cell + 1);
    }
    
    auto_cal_println("[AUTO-CAL] Wait for reading to stabilize, then type 'CONTINUE'...");
    auto_cal_waiting_for_input = true;
  }
}

void handle_step_b_span_collect_continue() {
  double current_mass = cal_masses[auto_cal_current_mass_point];
  
  auto_cal_printf("[AUTO-CAL] 📊 Recording %.1f kg calibration point...\n", current_mass);
  
  if (cal_config.center_placement) {
    // Center placement: record all 4 load cells simultaneously
    if (add_center_calibration_point(current_mass)) {
      auto_cal_printf("[AUTO-CAL] ✅ All 4 load cells recorded successfully!\n");
      auto_cal_current_mass_point++;
      
      // Small delay then continue to next mass point
      delay(1000);
      handle_step_b_span_collect();
    } else {
      auto_cal_printf("[AUTO-CAL] ❌ Failed to record calibration point!\n");
      auto_cal_println("[AUTO-CAL] Type 'CONTINUE' to retry or 'SKIP' to skip this point...");
      auto_cal_waiting_for_input = true;
    }
  } else {
    // Corner placement: record single load cell (original method)
    if (add_span_calibration_point(auto_cal_current_cell, current_mass)) {
      auto_cal_printf("[AUTO-CAL] ✅ Point recorded successfully!\n");
      auto_cal_current_mass_point++;
      
      // Small delay then continue to next mass point
      delay(1000);
      handle_step_b_span_collect();
    } else {
      auto_cal_printf("[AUTO-CAL] ❌ Failed to record calibration point!\n");
      auto_cal_println("[AUTO-CAL] Type 'CONTINUE' to retry or 'SKIP' to skip this point...");
      auto_cal_waiting_for_input = true;
    }
  }
}

void handle_step_b_span_compute() {
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 🔄 STEP B3: COMPUTING SPAN COEFFICIENTS");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📊 Analyzing calibration data for all load cells...");
  auto_cal_println("[AUTO-CAL] Computing span coefficients and linearity errors...");
  auto_cal_println("[AUTO-CAL] ");
  
  delay(2000);
  
  auto_cal_println("[AUTO-CAL] ✅ Load cell calibration complete!");
  auto_cal_println("[AUTO-CAL] ");
  delay(2000);
  
  start_step_c_intro();
}

// ============================================================================
// CENTER CALIBRATION FUNCTIONS (NEW METHOD)
// ============================================================================

// Global arrays to store center calibration data
static double center_cal_masses[10];
static int32_t center_cal_readings[4][10];  // [channel][mass_point]
static uint8_t center_cal_count = 0;

bool start_center_calibration() {
  auto_cal_println("[AUTO-CAL] Starting center placement calibration for all load cells");
  center_cal_count = 0;
  memset(center_cal_masses, 0, sizeof(center_cal_masses));
  memset(center_cal_readings, 0, sizeof(center_cal_readings));
  return true;
}

bool add_center_calibration_point(double mass_kg) {
  if (center_cal_count >= 10) {
    auto_cal_println("[AUTO-CAL] ❌ Maximum calibration points reached");
    return false;
  }
  
  auto_cal_printf("[AUTO-CAL] Adding center calibration point: %.3f kg\n", mass_kg);
  auto_cal_println("[AUTO-CAL] Collecting 100 samples from all 4 load cells...");
  
  int32_t sums[4] = {0, 0, 0, 0};
  const int num_samples = 100;
  
  for (int sample = 0; sample < num_samples; sample++) {
    for (int ch = 0; ch < 4; ch++) {
      int32_t reading = read_single_channel_fast(ch);
      sums[ch] += reading;
      delay(2);  // Small delay between readings
    }
  }
  
  // Calculate averages and apply offsets
  for (int ch = 0; ch < 4; ch++) {
    int32_t average = sums[ch] / num_samples;
    // Get the offset from the calibration system
    extern int32_t zero_offsets[4];
    extern bool offsets_applied;
    
    int32_t zeroed = average;
    if (offsets_applied) {
      zeroed = average - zero_offsets[ch];
    }
    
    center_cal_readings[ch][center_cal_count] = zeroed;
  }
  
  center_cal_masses[center_cal_count] = mass_kg;
  center_cal_count++;
  
  auto_cal_printf("[AUTO-CAL] Point added: %.3f kg\n", mass_kg);
  auto_cal_println("[AUTO-CAL] Load cell readings (zeroed):");
  for (int ch = 0; ch < 4; ch++) {
    auto_cal_printf("[AUTO-CAL]   LC%d: %ld counts\n", ch + 1, center_cal_readings[ch][center_cal_count-1]);
  }
  auto_cal_printf("[AUTO-CAL] Total points: %d\n", center_cal_count);
  
  return true;
}

bool compute_center_calibration(uint8_t channel) {
  if (channel >= 4 || center_cal_count < 3) {
    auto_cal_println("[AUTO-CAL] ❌ Need at least 3 calibration points");
    return false;
  }
  
  auto_cal_printf("[AUTO-CAL] Computing center calibration for Load Cell %d\n", channel + 1);
  auto_cal_printf("[AUTO-CAL] Using %d calibration points\n", center_cal_count);
  
  // Linear regression: mass = b1 * counts + b0
  // We want: counts per unit mass, so b1 = counts/mass
  double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
  
  for (int i = 0; i < center_cal_count; i++) {
    double mass = center_cal_masses[i];
    double counts = center_cal_readings[channel][i];
    
    sum_x += counts;
    sum_y += mass;
    sum_xy += counts * mass;
    sum_xx += counts * counts;
  }
  
  double n = center_cal_count;
  double slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
  double intercept = (sum_y - slope * sum_x) / n;
  
  // Convert to counts per kg (inverse of slope)
  double counts_per_kg = 1.0 / slope;
  
  // Calculate linearity error
  double max_error = 0;
  for (int i = 0; i < center_cal_count; i++) {
    double predicted_mass = slope * center_cal_readings[channel][i] + intercept;
    double error = fabs(predicted_mass - center_cal_masses[i]);
    if (error > max_error) max_error = error;
  }
  
  double linearity_percent = 0;
  if (center_cal_masses[center_cal_count-1] > 0) {
    linearity_percent = (max_error / center_cal_masses[center_cal_count-1]) * 100.0;
  }
  
  // Store calibration data using the existing calibration system
  // We'll use the existing temp arrays and compute_span_calibration function
  // First, copy our center calibration data to the temp arrays used by the existing system
  extern double temp_masses[20];
  extern int32_t temp_readings[4][20];
  extern uint8_t temp_data_count;
  
  // Copy our center calibration data to the existing temp arrays
  for (int i = 0; i < center_cal_count && i < 20; i++) {
    temp_masses[i] = center_cal_masses[i];
    temp_readings[channel][i] = center_cal_readings[channel][i];
  }
  temp_data_count = center_cal_count;
  
  // Use the existing compute_span_calibration function
  bool success = compute_span_calibration(channel);
  
  // Clear temp data for next channel
  temp_data_count = 0;
  
  if (success) {
    auto_cal_printf("[AUTO-CAL] Load Cell %d center calibration complete:\n", channel + 1);
    auto_cal_printf("[AUTO-CAL]   Span: %.2f counts/kg\n", counts_per_kg);
    auto_cal_printf("[AUTO-CAL]   Linearity error: %.3f%%\n", linearity_percent);
    auto_cal_printf("[AUTO-CAL]   Range: 0 - %.2f kg\n", center_cal_masses[center_cal_count-1]);
  }
  
  return success;
}

// ============================================================================
// STEP C: MATRIX CALIBRATION
// ============================================================================

void start_step_c_intro() {
  // Check if Step C is enabled
  if (!cal_config.step_c_enabled) {
    auto_cal_println("[AUTO-CAL] ==========================================");
    auto_cal_println("[AUTO-CAL] 📋 STEP C: MATRIX CALIBRATION (DISABLED)");
    auto_cal_println("[AUTO-CAL] ==========================================");
    auto_cal_println("[AUTO-CAL] ");
    auto_cal_println("[AUTO-CAL] ⏭️ Step C (Matrix Calibration) is DISABLED");
    auto_cal_println("[AUTO-CAL] Skipping to final save and verification...");
    auto_cal_println("[AUTO-CAL] ");
    auto_cal_println("[AUTO-CAL] ℹ️ Note: Without matrix calibration, you will have:");
    auto_cal_println("[AUTO-CAL]   ✓ Individual load cell force measurements");
    auto_cal_println("[AUTO-CAL]   ✗ No combined force (Fz) calculation");
    auto_cal_println("[AUTO-CAL]   ✗ No moment (Mx, My) calculations");
    auto_cal_println("[AUTO-CAL]   ✗ No center of pressure (COP) calculations");
    auto_cal_println("[AUTO-CAL] ");
    delay(3000);
    start_save_verify();
    return;
  }
  
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 📋 STEP C: MATRIX CALIBRATION (OPTIONAL)");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 🎯 PURPOSE:");
  auto_cal_println("[AUTO-CAL]   Create mapping from 4 load cell outputs to:");
  auto_cal_println("[AUTO-CAL]   • Total vertical force (Fz)");
  auto_cal_println("[AUTO-CAL]   • Moments about X and Y axes (Mx, My)");
  auto_cal_println("[AUTO-CAL]   • Center of pressure coordinates (COPx, COPy)");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] ⏱️  DURATION: ~20-30 minutes");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📋 WHAT WILL HAPPEN:");
  auto_cal_println("[AUTO-CAL]   • Place 10kg mass at 9 different positions");
  auto_cal_println("[AUTO-CAL]   • Record load cell responses at each position");
  auto_cal_println("[AUTO-CAL]   • Compute transformation matrix coefficients");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📦 EQUIPMENT NEEDED:");
  auto_cal_println("[AUTO-CAL]   • 10kg calibrated mass");
  auto_cal_println("[AUTO-CAL]   • Measuring tape for position verification");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📍 POSITIONS TO TEST:");
  for (int i = 0; i < num_matrix_positions; i++) {
    auto_cal_printf("[AUTO-CAL]   %d. %s (%.0f, %.0f mm)\n", 
                  i + 1, matrix_positions[i].description, 
                  matrix_positions[i].x, matrix_positions[i].y);
  }
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] ⚠️ This step is OPTIONAL:");
  auto_cal_println("[AUTO-CAL]   • Type 'CONTINUE' to perform matrix calibration");
  auto_cal_println("[AUTO-CAL]   • Type 'SKIP' to skip and complete calibration");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] Ready to start matrix calibration?");
  
  auto_cal_state = AUTO_CAL_STEP_C_INTRO;
  auto_cal_waiting_for_input = true;
  auto_cal_current_position = 0;
}

void start_step_c_collect() {
  start_matrix_calibration();
  auto_cal_state = AUTO_CAL_STEP_C_COLLECT;
  auto_cal_current_position = 0;
  
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 🔄 MATRIX CALIBRATION DATA COLLECTION");
  auto_cal_println("[AUTO-CAL] ==========================================");
  
  handle_step_c_collect();
}

void handle_step_c_collect() {
  if (auto_cal_current_position >= num_matrix_positions) {
    // All positions done, compute matrix
    auto_cal_state = AUTO_CAL_STEP_C_COMPUTE;
    handle_step_c_compute();
    return;
  }
  
  const CalPosition* pos = &matrix_positions[auto_cal_current_position];
  
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_printf("[AUTO-CAL] 📍 POSITION %d/%d: %s\n", 
                auto_cal_current_position + 1, num_matrix_positions, pos->description);
  auto_cal_printf("[AUTO-CAL] 📦 Place 10kg mass at coordinates: (%.0f, %.0f) mm\n", pos->x, pos->y);
  auto_cal_println("[AUTO-CAL] ");
  
  if (pos->x == 0.0 && pos->y == 0.0) {
    auto_cal_println("[AUTO-CAL] 🎯 CENTER: Place mass at the exact center of the plate");
  } else if (pos->x == 0.0) {
    if (pos->y > 0) {
      auto_cal_printf("[AUTO-CAL] 🎯 FRONT: Place mass %.0f mm toward the front\n", pos->y);
    } else {
      auto_cal_printf("[AUTO-CAL] 🎯 BACK: Place mass %.0f mm toward the back\n", -pos->y);
    }
  } else if (pos->y == 0.0) {
    if (pos->x > 0) {
      auto_cal_printf("[AUTO-CAL] 🎯 RIGHT: Place mass %.0f mm toward the right\n", pos->x);
    } else {
      auto_cal_printf("[AUTO-CAL] 🎯 LEFT: Place mass %.0f mm toward the left\n", -pos->x);
    }
  } else {
    auto_cal_printf("[AUTO-CAL] 🎯 CORNER: Place mass at corner position\n");
  }
  
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] Wait for reading to stabilize, then type 'CONTINUE'...");
  auto_cal_waiting_for_input = true;
}

void handle_step_c_collect_continue() {
  const CalPosition* pos = &matrix_positions[auto_cal_current_position];
  
  auto_cal_printf("[AUTO-CAL] 📊 Recording position: %s...\n", pos->description);
  
  if (add_matrix_calibration_point(10.0, pos->x, pos->y)) {
    auto_cal_printf("[AUTO-CAL] ✅ Position recorded successfully!\n");
    auto_cal_current_position++;
    
    delay(1000);
    handle_step_c_collect();
  } else {
    auto_cal_printf("[AUTO-CAL] ❌ Failed to record position!\n");
    auto_cal_println("[AUTO-CAL] Type 'CONTINUE' to retry or 'SKIP' to skip this position...");
    auto_cal_waiting_for_input = true;
  }
}

void handle_step_c_compute() {
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 🔄 STEP C: COMPUTING MATRIX COEFFICIENTS");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📊 Computing transformation matrix...");
  auto_cal_println("[AUTO-CAL] Calculating force and moment coefficients...");
  
  delay(2000);
  
  if (compute_matrix_calibration()) {
    auto_cal_println("[AUTO-CAL] ✅ Matrix calibration complete!");
  } else {
    auto_cal_println("[AUTO-CAL] ❌ Matrix calibration failed!");
  }
  
  auto_cal_println("[AUTO-CAL] ");
  delay(2000);
  
  start_save_verify();
}

// ============================================================================
// SAVE AND VERIFICATION
// ============================================================================

void start_save_verify() {
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 💾 SAVING AND VERIFICATION");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 🔄 Saving calibration data to EEPROM...");
  
  if (save_calibration_to_eeprom()) {
    auto_cal_println("[AUTO-CAL] ✅ Calibration data saved successfully!");
  } else {
    auto_cal_println("[AUTO-CAL] ❌ Failed to save calibration data!");
  }
  
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📊 CALIBRATION SUMMARY:");
  print_calibration_summary();
  
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 🧪 Testing calibrated measurements...");
  
  ForceData test_data = get_calibrated_force_data();
  if (test_data.valid) {
    auto_cal_println("[AUTO-CAL] ✅ Calibrated measurements working!");
    auto_cal_printf("[AUTO-CAL]   Force: %.2f N\n", test_data.fz);
    auto_cal_printf("[AUTO-CAL]   COP: (%.1f, %.1f) mm\n", test_data.cop_x, test_data.cop_y);
  } else {
    auto_cal_println("[AUTO-CAL] ⚠️ Calibrated measurements not available");
  }
  
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] Type 'CONTINUE' to complete calibration...");
  
  auto_cal_state = AUTO_CAL_SAVE_VERIFY;
  auto_cal_waiting_for_input = true;
}

void complete_calibration() {
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 🎉 CALIBRATION COMPLETE!");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] ✅ ALL CALIBRATION STEPS COMPLETED:");
  auto_cal_println("[AUTO-CAL]   ✓ Step A: ADC calibration");
  auto_cal_println("[AUTO-CAL]   ✓ Step B: Load cell calibration (4 cells)");
  auto_cal_println("[AUTO-CAL]   ✓ Step C: Matrix calibration");
  auto_cal_println("[AUTO-CAL]   ✓ Data saved to EEPROM");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 🎯 YOUR FORCE PLATE IS NOW READY FOR:");
  auto_cal_println("[AUTO-CAL]   • Jump tests");
  auto_cal_println("[AUTO-CAL]   • Balance tests");
  auto_cal_println("[AUTO-CAL]   • Research applications");
  auto_cal_println("[AUTO-CAL]   • Clinical assessments");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📋 NEXT STEPS:");
  auto_cal_println("[AUTO-CAL]   • Use 'CAL_FORCE_DATA' to see real-time measurements");
  auto_cal_println("[AUTO-CAL]   • Run validation tests with 'VAL_START'");
  auto_cal_println("[AUTO-CAL]   • Begin data acquisition with 'START'");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📄 DOCUMENTATION:");
  auto_cal_println("[AUTO-CAL]   • Record calibration date and conditions");
  auto_cal_println("[AUTO-CAL]   • Document masses and equipment used");
  auto_cal_println("[AUTO-CAL]   • Schedule next calibration (annually)");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] Thank you for using the automated calibration system!");
  auto_cal_println("[AUTO-CAL] ==========================================");
  
  auto_cal_state = AUTO_CAL_COMPLETE;
  auto_cal_active = false;
  auto_cal_waiting_for_input = false;
}

// ============================================================================
// UPDATE FUNCTION (CALL FROM MAIN LOOP)
// ============================================================================

void update_automated_calibration() {
  if (!auto_cal_active) return;
  
  // Handle timeouts or automatic progressions here if needed
  // For now, everything is user-driven with CONTINUE commands
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

bool is_auto_calibration_active() {
  return auto_cal_active;
}

void handle_auto_cal_continue() {
  if (auto_cal_state == AUTO_CAL_STEP_B_SPAN_COLLECT && !auto_cal_waiting_for_input) {
    handle_step_b_span_collect_continue();
  } else if (auto_cal_state == AUTO_CAL_STEP_C_COLLECT && !auto_cal_waiting_for_input) {
    handle_step_c_collect_continue();
  }
}
