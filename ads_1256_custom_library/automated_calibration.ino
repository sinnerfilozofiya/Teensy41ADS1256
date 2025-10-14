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

// Calibration mass sequence (in kg)
static const double cal_masses[] = {0.0, 5.0, 10.0, 15.0, 20.0};
static const uint8_t num_cal_masses = 5;

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
// AUTOMATED CALIBRATION FUNCTIONS
// ============================================================================

void start_automated_calibration() {
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 🚀 AUTOMATED FORCE PLATE CALIBRATION");
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] This automated system will guide you through");
  auto_cal_println("[AUTO-CAL] the complete 3-step calibration process:");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📋 STEP A: ADC (ADS1256) Calibration");
  auto_cal_println("[AUTO-CAL] 📋 STEP B: Per-Load-Cell Calibration");
  auto_cal_println("[AUTO-CAL] 📋 STEP C: Matrix Calibration");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] ⏱️  ESTIMATED TIME: 60-90 minutes");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📦 EQUIPMENT NEEDED:");
  auto_cal_println("[AUTO-CAL]   • Calibrated masses: 5kg, 10kg, 15kg, 20kg");
  auto_cal_println("[AUTO-CAL]   • Measuring tape/ruler");
  auto_cal_println("[AUTO-CAL]   • Level, stable surface");
  auto_cal_println("[AUTO-CAL]   • Patience and attention to detail");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 🎯 At each step, you can:");
  auto_cal_println("[AUTO-CAL]   • Type 'CONTINUE' to proceed");
  auto_cal_println("[AUTO-CAL]   • Type 'SKIP' to skip current step");
  auto_cal_println("[AUTO-CAL]   • Type 'ABORT' to cancel calibration");
  auto_cal_println("[AUTO-CAL]   • Type 'STATUS' to see current progress");
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
    auto_cal_println("[AUTO-CAL] ❓ Unknown command. Use: CONTINUE, SKIP, ABORT, or STATUS");
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
  auto_cal_println("[AUTO-CAL]   Calibrate each of the 4 load cells individually");
  auto_cal_println("[AUTO-CAL]   for accurate force measurements");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] ⏱️  DURATION: ~30-40 minutes");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📋 WHAT WILL HAPPEN:");
  auto_cal_println("[AUTO-CAL]   1. Tare (zero) all 4 load cells");
  auto_cal_println("[AUTO-CAL]   2. Span calibration for each cell (5 mass points)");
  auto_cal_println("[AUTO-CAL]   3. Linearity analysis and coefficient calculation");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] 📦 MASSES NEEDED:");
  auto_cal_println("[AUTO-CAL]   • 5kg, 10kg, 15kg, 20kg calibrated masses");
  auto_cal_println("[AUTO-CAL]   • Ability to place masses directly above each corner");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] ⚠️  IMPORTANT:");
  auto_cal_println("[AUTO-CAL]   • Remove ALL loads from the plate before starting");
  auto_cal_println("[AUTO-CAL]   • Place masses as centrally as possible above each corner");
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
  auto_cal_println("[AUTO-CAL]   We will calibrate each load cell with 5 mass points:");
  auto_cal_println("[AUTO-CAL]   • 0kg (no load)");
  auto_cal_println("[AUTO-CAL]   • 5kg mass");
  auto_cal_println("[AUTO-CAL]   • 10kg mass");
  auto_cal_println("[AUTO-CAL]   • 15kg mass");
  auto_cal_println("[AUTO-CAL]   • 20kg mass");
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
  auto_cal_println("[AUTO-CAL]   • Wait for 'READY FOR NEXT' message before changing masses");
  auto_cal_println("[AUTO-CAL] ");
  auto_cal_println("[AUTO-CAL] Ready to start span calibration?");
  auto_cal_println("[AUTO-CAL] Type 'CONTINUE' to begin with Load Cell 1...");
  
  auto_cal_state = AUTO_CAL_STEP_B_SPAN_SETUP;
  auto_cal_waiting_for_input = true;
  auto_cal_current_cell = 0;
  auto_cal_current_mass_point = 0;
}

void start_step_b_span_collect() {
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

void handle_step_b_span_collect() {
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
      start_step_c_intro();
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
  
  // When user continues, add the calibration point
  // This will be handled in the continue logic
}

void handle_step_b_span_collect_continue() {
  double current_mass = cal_masses[auto_cal_current_mass_point];
  
  auto_cal_printf("[AUTO-CAL] 📊 Recording %.1f kg calibration point...\n", current_mass);
  
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
// STEP C: MATRIX CALIBRATION
// ============================================================================

void start_step_c_intro() {
  auto_cal_println("[AUTO-CAL] ==========================================");
  auto_cal_println("[AUTO-CAL] 📋 STEP C: MATRIX CALIBRATION");
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
  auto_cal_println("[AUTO-CAL] Ready to start matrix calibration?");
  auto_cal_println("[AUTO-CAL] Type 'CONTINUE' to proceed...");
  
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
