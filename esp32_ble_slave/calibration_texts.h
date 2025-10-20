#ifndef CALIBRATION_TEXTS_H
#define CALIBRATION_TEXTS_H

// Auto-generated from calibration_texts.json
// Version: 1.0.0
// Schema Version: 1.0
// Description: Automated Force Plate Calibration Messages and Texts

// ============================================================================
// CALIBRATION MESSAGE STRUCTURE (from JSON "messages" section)
// ============================================================================
struct CalibrationMessage {
    const char* id;              // Message ID (e.g., "AC.INTRO.HEADER")
    int code;                    // Numeric code from JSON
    const char* type;            // Message type (header, body, prompt, etc.)
    const char* text_lines[15];  // Text lines array (max 15 lines)
    const char* placeholders[10]; // Placeholder names array (max 10 placeholders)
    const char* condition;       // Optional condition string
};

// ============================================================================
// CALIBRATION STATES ENUM (from JSON "states" section)
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
    AUTO_CAL_ERROR = 14,
};

// ============================================================================
// MATRIX POSITION STRUCTURE (from JSON "matrix_positions" section)
// ============================================================================
struct MatrixPosition {
    int id;                      // Position ID (0-8)
    float x;                     // X coordinate in mm
    float y;                     // Y coordinate in mm
    const char* description;     // Human-readable description
};

// ============================================================================
// PLACEHOLDER DOCUMENTATION STRUCTURE (from JSON "usage_notes.placeholders")
// ============================================================================
struct PlaceholderInfo {
    const char* name;            // Placeholder name (e.g., "mass_kg")
    const char* description;     // Description of the placeholder
};

// ============================================================================
// CONDITION DOCUMENTATION STRUCTURE (from JSON "usage_notes.conditions")
// ============================================================================
struct ConditionInfo {
    const char* condition;       // Condition string (e.g., "step_c_enabled")
    const char* description;     // Description of the condition
};

// ============================================================================
// MESSAGE TYPE DOCUMENTATION STRUCTURE (from JSON "usage_notes.message_types")
// ============================================================================
struct MessageTypeInfo {
    const char* type;            // Message type (e.g., "header")
    const char* description;     // Description of the message type
};

// ============================================================================
// CALIBRATION MESSAGES ARRAY
// ============================================================================

static const CalibrationMessage CALIBRATION_MESSAGES[] = {
    {
        "AC.INTRO.HEADER", 1, "header",
        {
            "==========================================",
            "🚀 AUTOMATED FORCE PLATE CALIBRATION",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.INTRO.OVERVIEW", 1, "body",
        {
            "",
            "This automated system will guide you through",
            "the flexible calibration process:",
            "",
            "📋 STEP A: ADC (ADS1256) Calibration",
            "📋 STEP B: Load Cell Calibration (CENTER placement)",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.INTRO.STEP_C_ENABLED", 1, "body",
        {
            "📋 STEP C: Matrix Calibration (OPTIONAL)",
            nullptr
        },
        {
            nullptr
        },
        "step_c_enabled == true"
    }
,
    {
        "AC.INTRO.STEP_C_DISABLED", 1, "body",
        {
            "📋 STEP C: Matrix Calibration (DISABLED)",
            nullptr
        },
        {
            nullptr
        },
        "step_c_enabled == false"
    }
,
    {
        "AC.INTRO.CONFIG_HEADER", 1, "body",
        {
            "",
            "📋 CURRENT CONFIGURATION:",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.INTRO.CONFIG_MASS_PLACEMENT", 1, "body",
        {
            "  Mass Placement: {mass_placement}",
            nullptr
        },
        {
            "mass_placement",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.INTRO.CONFIG_STEP_C", 1, "body",
        {
            "  Step C (Matrix): {step_c_status}",
            nullptr
        },
        {
            "step_c_status",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.INTRO.CONFIG_MASSES", 1, "body",
        {
            "  Calibration Masses ({num_masses}): {mass_list}",
            nullptr
        },
        {
            "num_masses",
            "mass_list",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.INTRO.ESTIMATED_TIME", 1, "body",
        {
            "",
            "⏱️  ESTIMATED TIME: {min_time}-{max_time} minutes",
            nullptr
        },
        {
            "min_time",
            "max_time",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.INTRO.EQUIPMENT_HEADER", 1, "body",
        {
            "",
            "📦 EQUIPMENT NEEDED:",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.INTRO.EQUIPMENT_MASSES", 1, "body",
        {
            "  • Calibrated masses: {mass_list}",
            nullptr
        },
        {
            "mass_list",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.INTRO.EQUIPMENT_MEASURING_TAPE", 1, "body",
        {
            "  • Measuring tape/ruler (for Step C)",
            nullptr
        },
        {
            nullptr
        },
        "step_c_enabled == true"
    }
,
    {
        "AC.INTRO.EQUIPMENT_GENERAL", 1, "body",
        {
            "  • Level, stable surface",
            "  • Patience and attention to detail",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.INTRO.ACTIONS", 1, "list",
        {
            "",
            "🎯 At each step, you can:",
            "  • Type 'CONTINUE' to proceed",
            "  • Type 'SKIP' to skip current step",
            "  • Type 'ABORT' to cancel calibration",
            "  • Type 'STATUS' to see current progress",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.INTRO.CONFIG_COMMANDS", 1, "list",
        {
            "",
            "💡 To change configuration, use:",
            "  • 'CAL_CONFIG_SHOW' - Show current settings",
            "  • 'CAL_CONFIG_MASSES 5,10,15' - Set custom masses",
            "  • 'CAL_CONFIG_DISABLE_STEP_C' - Skip matrix calibration",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.INTRO.READY", 1, "prompt",
        {
            "",
            "==========================================",
            "Ready to begin? Type 'CONTINUE' to start...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_A.HEADER", 2, "header",
        {
            "==========================================",
            "📋 STEP A: ADC (ADS1256) CALIBRATION",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_A.PURPOSE", 2, "body",
        {
            "",
            "🎯 PURPOSE:",
            "  Calibrate the ADS1256 ADC for accurate voltage",
            "  measurements at your current PGA and data rate",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_A.DURATION", 2, "body",
        {
            "",
            "⏱️  DURATION: ~30 seconds",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_A.WHAT_HAPPENS", 2, "body",
        {
            "",
            "📋 WHAT WILL HAPPEN:",
            "  • SELFOCAL: Offset calibration",
            "  • SELFGCAL: Gain calibration",
            "  • Store OFC and FSC register values",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_A.PREPARATION", 2, "body",
        {
            "",
            "✅ PREPARATION:",
            "  • Ensure stable power supply",
            "  • No external loads on ADC inputs",
            "  • System has been running for >2 minutes",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_A.READY", 2, "prompt",
        {
            "",
            "Ready to start ADC calibration?",
            "Type 'CONTINUE' to proceed...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_A.RUNNING", 3, "body",
        {
            "🔄 Starting ADC self-calibration...",
            "Please wait - this may take up to 30 seconds...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_A.SUCCESS", 3, "success",
        {
            "✅ ADC calibration completed successfully!",
            "",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_A.FAILURE", 3, "error",
        {
            "❌ ADC calibration failed!",
            "Check connections and power supply",
            "Type 'CONTINUE' to retry or 'SKIP' to continue anyway...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.HEADER", 4, "header",
        {
            "==========================================",
            "📋 STEP B: LOAD CELL CALIBRATION",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.PURPOSE_CENTER", 4, "body",
        {
            "",
            "🎯 PURPOSE:",
            "  Calibrate all 4 load cells simultaneously",
            "  using masses placed at the CENTER of the plate",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == true"
    }
,
    {
        "AC.STEP_B.PURPOSE_CORNER", 4, "body",
        {
            "",
            "🎯 PURPOSE:",
            "  Calibrate each of the 4 load cells individually",
            "  for accurate force measurements",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == false"
    }
,
    {
        "AC.STEP_B.DURATION_CENTER", 4, "body",
        {
            "",
            "⏱️  DURATION: ~15-20 minutes",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == true"
    }
,
    {
        "AC.STEP_B.DURATION_CORNER", 4, "body",
        {
            "",
            "⏱️  DURATION: ~30-40 minutes",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == false"
    }
,
    {
        "AC.STEP_B.WHAT_HAPPENS_HEADER", 4, "body",
        {
            "",
            "📋 WHAT WILL HAPPEN:",
            "  1. Tare (zero) all 4 load cells",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.WHAT_HAPPENS_CENTER", 4, "body",
        {
            "  2. Place masses in CENTER ({num_masses} mass points)",
            "  3. Record all 4 load cell responses simultaneously",
            "  4. Calculate calibration coefficients for all cells",
            nullptr
        },
        {
            "num_masses",
            nullptr
        },
        "center_placement == true"
    }
,
    {
        "AC.STEP_B.WHAT_HAPPENS_CORNER", 4, "body",
        {
            "  2. Span calibration for each cell ({num_masses} mass points)",
            "  3. Linearity analysis and coefficient calculation",
            nullptr
        },
        {
            "num_masses",
            nullptr
        },
        "center_placement == false"
    }
,
    {
        "AC.STEP_B.MASSES_HEADER", 4, "body",
        {
            "",
            "📦 MASSES NEEDED:",
            "  • {mass_list} calibrated masses",
            nullptr
        },
        {
            "mass_list",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.EQUIPMENT_CENTER", 4, "body",
        {
            "  • Ability to place masses at the CENTER of the plate",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == true"
    }
,
    {
        "AC.STEP_B.EQUIPMENT_CORNER", 4, "body",
        {
            "  • Ability to place masses directly above each corner",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == false"
    }
,
    {
        "AC.STEP_B.IMPORTANT_HEADER", 4, "body",
        {
            "",
            "⚠️  IMPORTANT:",
            "  • Remove ALL loads from the plate before starting",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.IMPORTANT_CENTER", 4, "body",
        {
            "  • Place masses at the EXACT CENTER of the plate",
            "  • All 4 load cells will share the load equally",
            "  • This method is faster and requires fewer masses",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == true"
    }
,
    {
        "AC.STEP_B.IMPORTANT_CORNER", 4, "body",
        {
            "  • Place masses as centrally as possible above each corner",
            "  • Avoid placing masses between load cells",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == false"
    }
,
    {
        "AC.STEP_B.IMPORTANT_GENERAL", 4, "body",
        {
            "  • Wait for stable readings between mass changes",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.READY", 4, "prompt",
        {
            "",
            "Ready to start load cell calibration?",
            "Type 'CONTINUE' to proceed...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.TARE_HEADER", 5, "header",
        {
            "==========================================",
            "🔄 STEP B1: TARE (ZERO) ALL LOAD CELLS",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.TARE_ENSURE", 5, "body",
        {
            "",
            "✅ ENSURE:",
            "  • NO masses or objects on the force plate",
            "  • Plate is level and stable",
            "  • No external forces applied",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.TARE_STARTING", 5, "body",
        {
            "",
            "🔄 Taring all 4 load cells...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.TARE_CELL", 5, "body",
        {
            "Taring Load Cell {cell_num}...",
            nullptr
        },
        {
            "cell_num",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.TARE_SUCCESS", 5, "success",
        {
            "✅ All load cells tared successfully!",
            "",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.TARE_FAILURE", 5, "error",
        {
            "❌ Some load cells failed to tare",
            "Type 'CONTINUE' to retry or 'SKIP' to continue anyway...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.SPAN_SETUP_HEADER", 6, "header",
        {
            "==========================================",
            "🔄 STEP B2: SPAN CALIBRATION SETUP",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.SPAN_SETUP_PROCESS_HEADER", 6, "body",
        {
            "",
            "📋 SPAN CALIBRATION PROCESS:",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.SPAN_SETUP_CENTER_MASSES", 6, "body",
        {
            "  We will calibrate all load cells with {num_masses} mass points:",
            nullptr
        },
        {
            "num_masses",
            nullptr
        },
        "center_placement == true"
    }
,
    {
        "AC.STEP_B.SPAN_SETUP_CORNER_MASSES", 6, "body",
        {
            "  We will calibrate each load cell with {num_masses} mass points:",
            nullptr
        },
        {
            "num_masses",
            nullptr
        },
        "center_placement == false"
    }
,
    {
        "AC.STEP_B.SPAN_SETUP_MASS_LIST", 6, "body",
        {
            "  • {mass_kg}kg{mass_note}",
            nullptr
        },
        {
            "mass_kg",
            "mass_note",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.SPAN_SETUP_CENTER_METHOD", 6, "body",
        {
            "",
            "🎯 CENTER PLACEMENT METHOD:",
            "  1. Place mass at the CENTER of the plate",
            "  2. Wait for reading to stabilize (~10 seconds)",
            "  3. System records ALL 4 load cell responses",
            "  4. Remove mass and prepare for next weight",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == true"
    }
,
    {
        "AC.STEP_B.SPAN_SETUP_CORNER_METHOD", 6, "body",
        {
            "",
            "🎯 FOR EACH LOAD CELL:",
            "  1. Place mass directly above the corner",
            "  2. Wait for reading to stabilize (~10 seconds)",
            "  3. System will automatically record the reading",
            "  4. Remove mass and prepare for next point",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == false"
    }
,
    {
        "AC.STEP_B.SPAN_SETUP_CRITICAL_CENTER", 6, "body",
        {
            "",
            "⚠️  CRITICAL:",
            "  • Place masses at the EXACT CENTER of the plate",
            "  • All 4 load cells will share the load equally",
            "  • This method is faster and requires fewer masses",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == true"
    }
,
    {
        "AC.STEP_B.SPAN_SETUP_CRITICAL_CORNER", 6, "body",
        {
            "",
            "⚠️  CRITICAL:",
            "  • Place masses as close to the load cell as possible",
            "  • Avoid placing masses between load cells",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == false"
    }
,
    {
        "AC.STEP_B.SPAN_SETUP_CRITICAL_GENERAL", 6, "body",
        {
            "  • Wait for 'READY FOR NEXT' message before changing masses",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.SPAN_SETUP_READY_CENTER", 6, "prompt",
        {
            "",
            "Ready to start CENTER calibration?",
            "Type 'CONTINUE' to begin...",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == true"
    }
,
    {
        "AC.STEP_B.SPAN_SETUP_READY_CORNER", 6, "prompt",
        {
            "",
            "Ready to start span calibration?",
            "Type 'CONTINUE' to begin with Load Cell 1...",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == false"
    }
,
    {
        "AC.STEP_B.SPAN_COLLECT_CENTER_HEADER", 7, "header",
        {
            "==========================================",
            "🔄 CENTER PLACEMENT CALIBRATION",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        "center_placement == true"
    }
,
    {
        "AC.STEP_B.SPAN_COLLECT_CORNER_HEADER", 7, "header",
        {
            "==========================================",
            "🔄 LOAD CELL {cell_num} SPAN CALIBRATION",
            "==========================================",
            nullptr
        },
        {
            "cell_num",
            nullptr
        },
        "center_placement == false"
    }
,
    {
        "AC.STEP_B.SPAN_COLLECT_MASS_POINT", 7, "body",
        {
            "",
            "📦 MASS POINT {current_point}/{total_points}: {mass_kg} kg",
            nullptr
        },
        {
            "current_point",
            "total_points",
            "mass_kg",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.SPAN_COLLECT_CENTER_INSTRUCTIONS", 7, "body",
        {
            "🔄 Remove all masses from the plate",
            "🔄 Place {mass_kg} kg at the CENTER of the plate",
            nullptr
        },
        {
            "mass_kg",
            nullptr
        },
        "center_placement == true"
    }
,
    {
        "AC.STEP_B.SPAN_COLLECT_CORNER_INSTRUCTIONS", 7, "body",
        {
            "🔄 Remove all masses from Load Cell {cell_num} area",
            "🔄 Place {mass_kg} kg directly above Load Cell {cell_num}",
            nullptr
        },
        {
            "cell_num",
            "mass_kg",
            nullptr
        },
        "center_placement == false"
    }
,
    {
        "AC.STEP_B.SPAN_COLLECT_WAIT", 7, "prompt",
        {
            "Wait for reading to stabilize, then type 'CONTINUE'...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.SPAN_COLLECT_RECORDING", 7, "body",
        {
            "📊 Recording {mass_kg} kg calibration point...",
            nullptr
        },
        {
            "mass_kg",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.SPAN_COLLECT_SUCCESS", 7, "success",
        {
            "✅ All 4 load cells recorded successfully!",
            "✅ Point recorded successfully!",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.SPAN_COLLECT_FAILURE", 7, "error",
        {
            "❌ Failed to record calibration point!",
            "Type 'CONTINUE' to retry or 'SKIP' to skip this point...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.SPAN_COMPUTE_HEADER", 8, "header",
        {
            "==========================================",
            "🔄 STEP B3: COMPUTING SPAN COEFFICIENTS",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.SPAN_COMPUTE_ANALYZING", 8, "body",
        {
            "",
            "📊 Analyzing calibration data for all load cells...",
            "Computing span coefficients and linearity errors...",
            "",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_B.SPAN_COMPUTE_SUCCESS", 8, "success",
        {
            "✅ Load cell calibration complete!",
            "",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_C.HEADER_DISABLED", 9, "header",
        {
            "==========================================",
            "📋 STEP C: MATRIX CALIBRATION (DISABLED)",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        "step_c_enabled == false"
    }
,
    {
        "AC.STEP_C.HEADER_ENABLED", 9, "header",
        {
            "==========================================",
            "📋 STEP C: MATRIX CALIBRATION (OPTIONAL)",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        "step_c_enabled == true"
    }
,
    {
        "AC.STEP_C.SKIP_MESSAGE", 9, "body",
        {
            "",
            "⏭️ Step C (Matrix Calibration) is DISABLED",
            "Skipping to final save and verification...",
            "",
            "ℹ️ Note: Without matrix calibration, you will have:",
            "  ✓ Individual load cell force measurements",
            "  ✗ No combined force (Fz) calculation",
            "  ✗ No moment (Mx, My) calculations",
            "  ✗ No center of pressure (COP) calculations",
            nullptr
        },
        {
            nullptr
        },
        "step_c_enabled == false"
    }
,
    {
        "AC.STEP_C.PURPOSE", 9, "body",
        {
            "",
            "🎯 PURPOSE:",
            "  Create mapping from 4 load cell outputs to:",
            "  • Total vertical force (Fz)",
            "  • Moments about X and Y axes (Mx, My)",
            "  • Center of pressure coordinates (COPx, COPy)",
            nullptr
        },
        {
            nullptr
        },
        "step_c_enabled == true"
    }
,
    {
        "AC.STEP_C.DURATION", 9, "body",
        {
            "",
            "⏱️  DURATION: ~20-30 minutes",
            nullptr
        },
        {
            nullptr
        },
        "step_c_enabled == true"
    }
,
    {
        "AC.STEP_C.WHAT_HAPPENS", 9, "body",
        {
            "",
            "📋 WHAT WILL HAPPEN:",
            "  • Place 10kg mass at 9 different positions",
            "  • Record load cell responses at each position",
            "  • Compute transformation matrix coefficients",
            nullptr
        },
        {
            nullptr
        },
        "step_c_enabled == true"
    }
,
    {
        "AC.STEP_C.EQUIPMENT", 9, "body",
        {
            "",
            "📦 EQUIPMENT NEEDED:",
            "  • 10kg calibrated mass",
            "  • Measuring tape for position verification",
            nullptr
        },
        {
            nullptr
        },
        "step_c_enabled == true"
    }
,
    {
        "AC.STEP_C.POSITIONS_HEADER", 9, "body",
        {
            "",
            "📍 POSITIONS TO TEST:",
            nullptr
        },
        {
            nullptr
        },
        "step_c_enabled == true"
    }
,
    {
        "AC.STEP_C.POSITION_ITEM", 9, "body",
        {
            "  {pos_num}. {pos_desc} ({x_mm}, {y_mm} mm)",
            nullptr
        },
        {
            "pos_num",
            "pos_desc",
            "x_mm",
            "y_mm",
            nullptr
        },
        "step_c_enabled == true"
    }
,
    {
        "AC.STEP_C.OPTIONAL_NOTE", 9, "body",
        {
            "",
            "⚠️ This step is OPTIONAL:",
            "  • Type 'CONTINUE' to perform matrix calibration",
            "  • Type 'SKIP' to skip and complete calibration",
            nullptr
        },
        {
            nullptr
        },
        "step_c_enabled == true"
    }
,
    {
        "AC.STEP_C.READY", 9, "prompt",
        {
            "",
            "Ready to start matrix calibration?",
            nullptr
        },
        {
            nullptr
        },
        "step_c_enabled == true"
    }
,
    {
        "AC.STEP_C.COLLECT_HEADER", 10, "header",
        {
            "==========================================",
            "🔄 MATRIX CALIBRATION DATA COLLECTION",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_C.COLLECT_POSITION", 10, "body",
        {
            "",
            "📍 POSITION {current_pos}/{total_pos}: {pos_desc}",
            "📦 Place 10kg mass at coordinates: ({x_mm}, {y_mm}) mm",
            nullptr
        },
        {
            "current_pos",
            "total_pos",
            "pos_desc",
            "x_mm",
            "y_mm",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_C.COLLECT_CENTER_INSTRUCTIONS", 10, "body",
        {
            "",
            "🎯 CENTER: Place mass at the exact center of the plate",
            nullptr
        },
        {
            nullptr
        },
        "x_mm == 0 && y_mm == 0"
    }
,
    {
        "AC.STEP_C.COLLECT_FRONT_INSTRUCTIONS", 10, "body",
        {
            "",
            "🎯 FRONT: Place mass {y_mm} mm toward the front",
            nullptr
        },
        {
            "y_mm",
            nullptr
        },
        "x_mm == 0 && y_mm > 0"
    }
,
    {
        "AC.STEP_C.COLLECT_BACK_INSTRUCTIONS", 10, "body",
        {
            "",
            "🎯 BACK: Place mass {y_mm_abs} mm toward the back",
            nullptr
        },
        {
            "y_mm_abs",
            nullptr
        },
        "x_mm == 0 && y_mm < 0"
    }
,
    {
        "AC.STEP_C.COLLECT_RIGHT_INSTRUCTIONS", 10, "body",
        {
            "",
            "🎯 RIGHT: Place mass {x_mm} mm toward the right",
            nullptr
        },
        {
            "x_mm",
            nullptr
        },
        "y_mm == 0 && x_mm > 0"
    }
,
    {
        "AC.STEP_C.COLLECT_LEFT_INSTRUCTIONS", 10, "body",
        {
            "",
            "🎯 LEFT: Place mass {x_mm_abs} mm toward the left",
            nullptr
        },
        {
            "x_mm_abs",
            nullptr
        },
        "y_mm == 0 && x_mm < 0"
    }
,
    {
        "AC.STEP_C.COLLECT_CORNER_INSTRUCTIONS", 10, "body",
        {
            "",
            "🎯 CORNER: Place mass at corner position",
            nullptr
        },
        {
            nullptr
        },
        "x_mm != 0 && y_mm != 0"
    }
,
    {
        "AC.STEP_C.COLLECT_WAIT", 10, "prompt",
        {
            "",
            "Wait for reading to stabilize, then type 'CONTINUE'...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_C.COLLECT_RECORDING", 10, "body",
        {
            "📊 Recording position: {pos_desc}...",
            nullptr
        },
        {
            "pos_desc",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_C.COLLECT_SUCCESS", 10, "success",
        {
            "✅ Position recorded successfully!",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_C.COLLECT_FAILURE", 10, "error",
        {
            "❌ Failed to record position!",
            "Type 'CONTINUE' to retry or 'SKIP' to skip this position...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_C.COMPUTE_HEADER", 11, "header",
        {
            "==========================================",
            "🔄 STEP C: COMPUTING MATRIX COEFFICIENTS",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_C.COMPUTE_ANALYZING", 11, "body",
        {
            "",
            "📊 Computing transformation matrix...",
            "Calculating force and moment coefficients...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_C.COMPUTE_SUCCESS", 11, "success",
        {
            "✅ Matrix calibration complete!",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STEP_C.COMPUTE_FAILURE", 11, "error",
        {
            "❌ Matrix calibration failed!",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.SAVE.HEADER", 12, "header",
        {
            "==========================================",
            "💾 SAVING AND VERIFICATION",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.SAVE.STARTING", 12, "body",
        {
            "",
            "🔄 Saving calibration data to EEPROM...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.SAVE.SUCCESS", 12, "success",
        {
            "✅ Calibration data saved successfully!",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.SAVE.FAILURE", 12, "error",
        {
            "❌ Failed to save calibration data!",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.SAVE.SUMMARY_HEADER", 12, "body",
        {
            "",
            "📊 CALIBRATION SUMMARY:",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.SAVE.TESTING", 12, "body",
        {
            "",
            "🧪 Testing calibrated measurements...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.SAVE.TEST_SUCCESS", 12, "success",
        {
            "✅ Calibrated measurements working!",
            "  Force: {force_n} N",
            "  COP: ({cop_x}, {cop_y}) mm",
            nullptr
        },
        {
            "force_n",
            "cop_x",
            "cop_y",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.SAVE.TEST_WARNING", 12, "warning",
        {
            "⚠️ Calibrated measurements not available",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.SAVE.CONTINUE", 12, "prompt",
        {
            "",
            "Type 'CONTINUE' to complete calibration...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.COMPLETE.HEADER", 13, "header",
        {
            "==========================================",
            "🎉 CALIBRATION COMPLETE!",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.COMPLETE.SUCCESS", 13, "success",
        {
            "",
            "✅ ALL CALIBRATION STEPS COMPLETED:",
            "  ✓ Step A: ADC calibration",
            "  ✓ Step B: Load cell calibration (4 cells)",
            "  ✓ Step C: Matrix calibration",
            "  ✓ Data saved to EEPROM",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.COMPLETE.READY_FOR", 13, "body",
        {
            "",
            "🎯 YOUR FORCE PLATE IS NOW READY FOR:",
            "  • Jump tests",
            "  • Balance tests",
            "  • Research applications",
            "  • Clinical assessments",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.COMPLETE.NEXT_STEPS", 13, "body",
        {
            "",
            "📋 NEXT STEPS:",
            "  • Use 'CAL_FORCE_DATA' to see real-time measurements",
            "  • Run validation tests with 'VAL_START'",
            "  • Begin data acquisition with 'START'",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.COMPLETE.DOCUMENTATION", 13, "body",
        {
            "",
            "📄 DOCUMENTATION:",
            "  • Record calibration date and conditions",
            "  • Document masses and equipment used",
            "  • Schedule next calibration (annually)",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.COMPLETE.FOOTER", 13, "body",
        {
            "",
            "==========================================",
            "Thank you for using the automated calibration system!",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STATUS.HEADER", 0, "header",
        {
            "==========================================",
            "📊 CALIBRATION PROGRESS STATUS",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STATUS.STATE", 0, "body",
        {
            "Current State: {state_name}",
            nullptr
        },
        {
            "state_name",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STATUS.LOAD_CELL", 0, "body",
        {
            "Current Load Cell: {current_cell}/4",
            nullptr
        },
        {
            "current_cell",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STATUS.MASS_POINT", 0, "body",
        {
            "Current Mass Point: {current_point}/{total_points}",
            nullptr
        },
        {
            "current_point",
            "total_points",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STATUS.POSITION", 0, "body",
        {
            "Current Position: {current_pos}/{total_positions}",
            nullptr
        },
        {
            "current_pos",
            "total_positions",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STATUS.WAITING", 0, "body",
        {
            "Waiting for input: {waiting_status}",
            nullptr
        },
        {
            "waiting_status",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.STATUS.FOOTER", 0, "body",
        {
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.HEADER", 0, "header",
        {
            "==========================================",
            "📋 CURRENT CALIBRATION CONFIGURATION",
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.STEP_C", 0, "body",
        {
            "Step C (Matrix Cal): {step_c_status}",
            nullptr
        },
        {
            "step_c_status",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.MASS_PLACEMENT", 0, "body",
        {
            "Mass Placement: {mass_placement}",
            nullptr
        },
        {
            "mass_placement",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.NUM_MASSES", 0, "body",
        {
            "Number of Masses: {num_masses}",
            nullptr
        },
        {
            "num_masses",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.MASSES_HEADER", 0, "body",
        {
            "Calibration Masses:",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.MASS_ITEM", 0, "body",
        {
            "  {mass_num}. {mass_kg} kg",
            nullptr
        },
        {
            "mass_num",
            "mass_kg",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.USING", 0, "body",
        {
            "Using: {mass_type} masses",
            nullptr
        },
        {
            "mass_type",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.FOOTER", 0, "body",
        {
            "==========================================",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.ERROR.ABORT", 0, "error",
        {
            "❌ Calibration aborted by user",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.ERROR.WAIT", 0, "warning",
        {
            "⚠️ Please wait for the current step to complete",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.ERROR.SKIP", 0, "body",
        {
            "⏭️ Skipping current step...",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.ERROR.UNKNOWN_COMMAND", 0, "error",
        {
            "❓ Unknown command. Use: CONTINUE, SKIP, ABORT, STATUS, or CAL_CONFIG_*",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.ERROR.INVALID_STATE", 0, "error",
        {
            "❌ Invalid state",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.ERROR.SKIP_PHASE", 0, "body",
        {
            "⏭️ Step skipped - moving to next phase",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.SUCCESS.MASSES_SET", 0, "success",
        {
            "✓ Custom calibration masses set ({num_masses} masses)",
            nullptr
        },
        {
            "num_masses",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.SUCCESS.MASSES_LIST", 0, "body",
        {
            "Masses: ",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.SUCCESS.MASS_ITEM", 0, "body",
        {
            "  {mass_num}. {mass_kg} kg",
            nullptr
        },
        {
            "mass_num",
            "mass_kg",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.SUCCESS.RESET_MASSES", 0, "success",
        {
            "✓ Reset to default calibration masses (0, 5, 10, 15, 20 kg)",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.SUCCESS.STEP_C", 0, "success",
        {
            "✓ Step C (Matrix Calibration) {status}",
            nullptr
        },
        {
            "status",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.SUCCESS.MASS_PLACEMENT", 0, "success",
        {
            "✓ Mass placement: {placement}",
            nullptr
        },
        {
            "placement",
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.ERROR.INVALID_MASS_FORMAT", 0, "error",
        {
            "❌ Invalid mass format. Use: CAL_CONFIG_MASSES 5.0,10.0,15.0,20.0",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
,
    {
        "AC.CONFIG.ERROR.UNKNOWN_COMMAND", 0, "error",
        {
            "❌ Unknown config command",
            "Available commands:",
            "  CAL_CONFIG_MASSES <mass1,mass2,mass3,...>",
            "  CAL_CONFIG_RESET_MASSES",
            "  CAL_CONFIG_ENABLE_STEP_C",
            "  CAL_CONFIG_DISABLE_STEP_C",
            "  CAL_CONFIG_CENTER_PLACEMENT",
            "  CAL_CONFIG_CORNER_PLACEMENT",
            "  CAL_CONFIG_SHOW",
            nullptr
        },
        {
            nullptr
        },
        nullptr
    }
};

// ============================================================================
// MATRIX POSITIONS ARRAY (from JSON "matrix_positions" section)
// ============================================================================

static const MatrixPosition MATRIX_POSITIONS[] = {
    {0, 0.0, 0.0, "Center"}
,
    {1, 0.0, 150.0, "Front Center"}
,
    {2, 0.0, -150.0, "Back Center"}
,
    {3, 150.0, 0.0, "Right Center"}
,
    {4, -150.0, 0.0, "Left Center"}
,
    {5, 150.0, 150.0, "Front-Right Corner"}
,
    {6, -150.0, 150.0, "Front-Left Corner"}
,
    {7, 150.0, -150.0, "Back-Right Corner"}
,
    {8, -150.0, -150.0, "Back-Left Corner"}
};

// ============================================================================
// STATE NAMES ARRAY (from JSON "state_names" section)
// ============================================================================

static const char* STATE_NAMES[] = {
    "Idle"
,
    "Introduction"
,
    "Step A Intro"
,
    "Step A Running"
,
    "Step B Intro"
,
    "Step B Tare"
,
    "Step B Span Setup"
,
    "Step B Span Collect"
,
    "Step B Span Compute"
,
    "Step C Intro"
,
    "Step C Collect"
,
    "Step C Compute"
,
    "Save & Verify"
,
    "Complete"
,
    "Error"
};

// ============================================================================
// PLACEHOLDER DOCUMENTATION ARRAY (from JSON "usage_notes.placeholders")
// ============================================================================

static const PlaceholderInfo PLACEHOLDER_DOCS[] = {
    {"mass_kg", "Mass value in kg (e.g., 5.0, 10.0)"}
,
    {"mass_note", "Additional note for mass (e.g., ' (no load)' for 0kg)"}
,
    {"num_masses", "Number of calibration masses"}
,
    {"mass_list", "Comma-separated list of masses (e.g., '5.0kg, 10.0kg, 15.0kg')"}
,
    {"current_point", "Current mass point number (1-based)"}
,
    {"total_points", "Total number of mass points"}
,
    {"current_cell", "Current load cell number (1-based)"}
,
    {"current_pos", "Current position number (1-based)"}
,
    {"total_positions", "Total number of positions"}
,
    {"pos_desc", "Position description (e.g., 'Center', 'Front Center')"}
,
    {"x_mm", "X coordinate in mm"}
,
    {"y_mm", "Y coordinate in mm"}
,
    {"x_mm_abs", "Absolute value of X coordinate in mm"}
,
    {"y_mm_abs", "Absolute value of Y coordinate in mm"}
,
    {"pos_num", "Position number (1-based)"}
,
    {"cell_num", "Load cell number (1-based)"}
,
    {"state_name", "Current state name"}
,
    {"waiting_status", "YES or NO"}
,
    {"step_c_status", "ENABLED or DISABLED"}
,
    {"mass_placement", "CENTER of plate or Individual corners"}
,
    {"mass_type", "DEFAULT or CUSTOM"}
,
    {"mass_num", "Mass number in list (1-based)"}
,
    {"placement", "CENTER of plate or INDIVIDUAL load cell corners"}
,
    {"status", "ENABLED or DISABLED"}
,
    {"force_n", "Force value in Newtons"}
,
    {"cop_x", "Center of pressure X coordinate in mm"}
,
    {"cop_y", "Center of pressure Y coordinate in mm"}
};

// ============================================================================
// CONDITION DOCUMENTATION ARRAY (from JSON "usage_notes.conditions")
// ============================================================================

static const ConditionInfo CONDITION_DOCS[] = {
    {"step_c_enabled", "Step C (Matrix Calibration) is enabled"}
,
    {"center_placement", "Masses are placed at center of plate (vs individual corners)"}
,
    {"x_mm == 0 && y_mm == 0", "Position is at center (0,0)"}
,
    {"x_mm == 0 && y_mm > 0", "Position is on front edge"}
,
    {"x_mm == 0 && y_mm < 0", "Position is on back edge"}
,
    {"y_mm == 0 && x_mm > 0", "Position is on right edge"}
,
    {"y_mm == 0 && x_mm < 0", "Position is on left edge"}
,
    {"x_mm != 0 && y_mm != 0", "Position is at a corner"}
};

// ============================================================================
// MESSAGE TYPE DOCUMENTATION ARRAY (from JSON "usage_notes.message_types")
// ============================================================================

static const MessageTypeInfo MESSAGE_TYPE_DOCS[] = {
    {"header", "Section header with separator lines"}
,
    {"body", "Regular informational text"}
,
    {"prompt", "User input prompt"}
,
    {"list", "Bulleted or numbered list"}
,
    {"success", "Success confirmation message"}
,
    {"error", "Error or failure message"}
,
    {"warning", "Warning or caution message"}
};

// ============================================================================
// CONSTANTS AND VERSION INFO
// ============================================================================

// Array size constants (automatically calculated)
#define CALIBRATION_MESSAGES_COUNT (sizeof(CALIBRATION_MESSAGES) / sizeof(CalibrationMessage))
#define MATRIX_POSITIONS_COUNT (sizeof(MATRIX_POSITIONS) / sizeof(MatrixPosition))
#define STATE_NAMES_COUNT (sizeof(STATE_NAMES) / sizeof(char*))
#define PLACEHOLDER_DOCS_COUNT (sizeof(PLACEHOLDER_DOCS) / sizeof(PlaceholderInfo))
#define CONDITION_DOCS_COUNT (sizeof(CONDITION_DOCS) / sizeof(ConditionInfo))
#define MESSAGE_TYPE_DOCS_COUNT (sizeof(MESSAGE_TYPE_DOCS) / sizeof(MessageTypeInfo))

// Version info (from JSON file header)
#define CALIBRATION_TEXTS_VERSION "1.0.0"
#define CALIBRATION_TEXTS_SCHEMA_VERSION "1.0"
#define CALIBRATION_TEXTS_DESCRIPTION "Automated Force Plate Calibration Messages and Texts"

#endif // CALIBRATION_TEXTS_H