# Calibration Step ID System Integration

## Overview

The calibration system has been enhanced with a step ID system that sends structured identifiers to the ESP32 during automated calibration. This allows client applications to track the exact calibration step and provide appropriate user interfaces and feedback.

## How It Works

### 1. Step ID Functions

The system includes several functions for sending different types of step IDs:

- `send_calibration_step_id(step_id)` - Sends a general step ID
- `send_calibration_status_id(step_id)` - Sends a status update ID
- `send_calibration_error_id(step_id)` - Sends an error ID
- `send_calibration_success_id(step_id)` - Sends a success ID
- `send_calibration_prompt_id(step_id)` - Sends a prompt ID (when waiting for user input)
- `send_contextual_step_id(step_id, context)` - Sends a step ID with additional context

### 2. Communication Format

Step IDs are sent to the ESP32 via Serial3 in the following formats:

```
CAL_STEP_ID:AC.INTRO.HEADER
CAL_STATUS_ID:AC.STATUS.HEADER
CAL_ERROR_ID:AC.ERROR.ABORT
CAL_SUCCESS_ID:AC.STEP_A.SUCCESS
CAL_PROMPT_ID:AC.STEP_A.READY
CAL_STEP_ID:AC.STEP_B.SPAN_COLLECT.MASS_POINT:2/5:10.0kg
```

### 3. Step ID Mapping

The system automatically maps calibration states to step IDs:

| State | Step ID |
|-------|---------|
| `AUTO_CAL_INTRO` | `AC.INTRO.HEADER` |
| `AUTO_CAL_STEP_A_INTRO` | `AC.STEP_A.HEADER` |
| `AUTO_CAL_STEP_A_RUNNING` | `AC.STEP_A.RUNNING` |
| `AUTO_CAL_STEP_B_INTRO` | `AC.STEP_B.HEADER` |
| `AUTO_CAL_STEP_B_TARE` | `AC.STEP_B.TARE_HEADER` |
| `AUTO_CAL_STEP_B_SPAN_SETUP` | `AC.STEP_B.SPAN_SETUP_HEADER` |
| `AUTO_CAL_STEP_B_SPAN_COLLECT` | `AC.STEP_B.SPAN_COLLECT_CENTER_HEADER` or `AC.STEP_B.SPAN_COLLECT_CORNER_HEADER` |
| `AUTO_CAL_STEP_B_SPAN_COMPUTE` | `AC.STEP_B.SPAN_COMPUTE_HEADER` |
| `AUTO_CAL_STEP_C_INTRO` | `AC.STEP_C.HEADER_ENABLED` or `AC.STEP_C.HEADER_DISABLED` |
| `AUTO_CAL_STEP_C_COLLECT` | `AC.STEP_C.COLLECT_HEADER` |
| `AUTO_CAL_STEP_C_COMPUTE` | `AC.STEP_C.COMPUTE_HEADER` |
| `AUTO_CAL_SAVE_VERIFY` | `AC.SAVE.HEADER` |
| `AUTO_CAL_COMPLETE` | `AC.COMPLETE.HEADER` |
| `AUTO_CAL_ERROR` | `AC.ERROR.INVALID_STATE` |

### 4. Contextual Information

For detailed tracking, the system sends contextual information:

- **Mass Points**: `AC.STEP_B.SPAN_COLLECT.MASS_POINT:2/5:10.0kg`
- **Matrix Positions**: `AC.STEP_C.COLLECT_POSITION:3/9:Front Center`
- **Load Cell Progress**: `AC.STEP_B.SPAN_COLLECT_CORNER_HEADER:2/4`

## Integration Points

### 1. Calibration Start

When `AUTO_CAL_START` or `AUTOMATED_CALIBRATION` is received:

```cpp
start_automated_calibration();
send_status_response("AUTO_CAL_START", "OK");
// Send initial step ID
Serial3.println("CAL_STEP_ID:AC.INTRO.HEADER");
Serial3.flush();
```

### 2. State Changes

Each major calibration phase sends its step ID:

- **Step A Intro**: `AC.STEP_A.HEADER`
- **Step A Running**: `AC.STEP_A.RUNNING`
- **Step A Success**: `AC.STEP_A.SUCCESS`
- **Step A Failure**: `AC.STEP_A.FAILURE`
- **Step B Intro**: `AC.STEP_B.HEADER`
- **Step B Tare**: `AC.STEP_B.TARE_HEADER`
- **Step B Success**: `AC.STEP_B.TARE_SUCCESS`
- **Step B Failure**: `AC.STEP_B.TARE_FAILURE`
- **Completion**: `AC.COMPLETE.HEADER`

### 3. User Prompts

When waiting for user input:

- **Continue Prompts**: `AC.STEP_A.READY`
- **Status Requests**: `AC.STATUS.HEADER`
- **Error Handling**: `AC.ERROR.ABORT`

## ESP32 Client Implementation

### 1. Parsing Step IDs

```cpp
void parseCalibrationStepId(String message) {
  if (message.startsWith("CAL_STEP_ID:")) {
    String stepId = message.substring(12);
    handleCalibrationStep(stepId);
  } else if (message.startsWith("CAL_STATUS_ID:")) {
    String statusId = message.substring(14);
    handleCalibrationStatus(statusId);
  } else if (message.startsWith("CAL_ERROR_ID:")) {
    String errorId = message.substring(13);
    handleCalibrationError(errorId);
  } else if (message.startsWith("CAL_SUCCESS_ID:")) {
    String successId = message.substring(15);
    handleCalibrationSuccess(successId);
  } else if (message.startsWith("CAL_PROMPT_ID:")) {
    String promptId = message.substring(14);
    handleCalibrationPrompt(promptId);
  }
}
```

### 2. Step ID Lookup

Use the `calibration_texts.json` file to look up messages by step ID:

```cpp
String getCalibrationMessage(String stepId) {
  // Look up stepId in calibration_texts.json
  // Return the corresponding message text
  // Handle placeholders and conditions
}
```

### 3. Context Parsing

For contextual step IDs, parse the additional information:

```cpp
void handleContextualStepId(String stepId, String context) {
  if (stepId == "AC.STEP_B.SPAN_COLLECT.MASS_POINT") {
    // Parse context: "2/5:10.0kg"
    // Extract current point, total points, and mass
  } else if (stepId == "AC.STEP_C.COLLECT_POSITION") {
    // Parse context: "3/9:Front Center"
    // Extract position number, total positions, and description
  }
}
```

## Benefits

1. **Structured Communication**: ESP32 receives clear, structured step identifiers
2. **Client Synchronization**: Client applications can track exact calibration progress
3. **User Interface Updates**: Clients can update UI based on current step
4. **Error Handling**: Clear error and success states for robust error handling
5. **Progress Tracking**: Detailed progress information for user feedback
6. **Contextual Information**: Additional context for complex steps like mass collection

## Usage Example

When a calibration starts:

1. ESP32 receives: `CAL_STEP_ID:AC.INTRO.HEADER`
2. Client looks up `AC.INTRO.HEADER` in `calibration_texts.json`
3. Client displays the corresponding message to user
4. As calibration progresses, client receives new step IDs
5. Client updates UI and provides appropriate feedback

This system provides a robust, structured way for ESP32 clients to track and respond to calibration progress in real-time.
