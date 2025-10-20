# Calibration Texts Header File Usage

## Overview

The `calibration_texts.h` file is auto-generated from `calibration_texts.json` and provides a C++ header that can be imported into ESP32 and Arduino projects.

## File Structure

```
calibration_texts.h          # Generated header file (import this)
calibration_texts.json       # Source JSON file (edit this)
CALIBRATION_TEXTS_USAGE.md   # This documentation
```

## Usage in ESP32/Arduino Code

### 1. Include the Header

```cpp
#include "calibration_texts.h"
// or if in subdirectory:
#include "../calibration_texts.h"
```

### 2. Access Calibration Messages

```cpp
// Find a message by ID
const CalibrationMessage* find_calibration_message(const String& id) {
    for (int i = 0; i < CALIBRATION_MESSAGES_COUNT; i++) {
        if (String(CALIBRATION_MESSAGES[i].id) == id) {
            return &CALIBRATION_MESSAGES[i];
        }
    }
    return nullptr;
}

// Example usage
const CalibrationMessage* msg = find_calibration_message("AC.INTRO.HEADER");
if (msg != nullptr) {
    Serial.printf("Type: %s\n", msg->type);
    for (int i = 0; i < 15 && msg->text_lines[i] != nullptr; i++) {
        Serial.printf("Line %d: %s\n", i, msg->text_lines[i]);
    }
}
```

### 3. Available Data Structures

#### CalibrationMessage
```cpp
struct CalibrationMessage {
    const char* id;              // Message ID (e.g., "AC.INTRO.HEADER")
    const char* type;            // Message type (e.g., "header", "body", "prompt")
    const char* text_lines[15];  // Array of text lines (null-terminated)
    bool has_placeholders;       // True if message contains {placeholder} variables
    const char* condition;       // Optional condition for conditional messages
};
```

#### MatrixPosition
```cpp
struct MatrixPosition {
    int id;                      // Position ID (0-8)
    float x;                     // X coordinate in mm
    float y;                     // Y coordinate in mm
    const char* description;     // Human-readable description
};
```

#### AutoCalState Enum
```cpp
enum AutoCalState {
    AUTO_CAL_IDLE = 0,
    AUTO_CAL_INTRO = 1,
    AUTO_CAL_STEP_A_INTRO = 2,
    // ... etc
};
```

### 4. Available Constants

```cpp
#define CALIBRATION_MESSAGES_COUNT    // Number of messages in array
#define MATRIX_POSITIONS_COUNT        // Number of matrix positions
#define STATE_NAMES_COUNT            // Number of state names

#define CALIBRATION_TEXTS_VERSION "1.0.0"
#define CALIBRATION_TEXTS_SCHEMA_VERSION "1.0"
```

### 5. Available Arrays

```cpp
CALIBRATION_MESSAGES[]    // All calibration messages
MATRIX_POSITIONS[]        // Matrix calibration positions
STATE_NAMES[]            // State name strings
```

## Regenerating the Header File

When you update `calibration_texts.json`, you need to regenerate the header file:

1. **Manual Method**: Update the header file manually to match JSON changes
2. **Script Method**: Create a conversion script (Python/Node.js) to auto-generate
3. **Build Integration**: Add generation step to your build process

## Example: Complete Message Display

```cpp
void display_calibration_message(const String& message_id) {
    const CalibrationMessage* msg = find_calibration_message(message_id);
    
    if (msg != nullptr) {
        Serial.println("╔════════════════════════════════════════╗");
        Serial.printf("║ ID: %s\n", msg->id);
        Serial.printf("║ Type: %s\n", msg->type);
        Serial.println("╠════════════════════════════════════════╣");
        
        for (int i = 0; i < 15 && msg->text_lines[i] != nullptr; i++) {
            String line = String(msg->text_lines[i]);
            
            if (msg->has_placeholders && line.indexOf('{') >= 0) {
                Serial.printf("║ %s [*has placeholders*]\n", line.c_str());
            } else {
                Serial.printf("║ %s\n", line.c_str());
            }
        }
        
        Serial.println("╚════════════════════════════════════════╝");
    } else {
        Serial.printf("Message ID '%s' not found!\n", message_id.c_str());
    }
}
```

## Benefits

1. **Single Source of Truth**: All calibration texts come from `calibration_texts.json`
2. **Type Safety**: C++ structs provide compile-time checking
3. **Memory Efficient**: Stored in program memory (PROGMEM compatible)
4. **Easy Integration**: Simple `#include` statement
5. **Maintainable**: Update JSON, regenerate header, recompile

## Integration Points

- **ESP32 BLE Slave**: Uses header for message lookup and display
- **ESP32 SPI Slave**: Can use for local message handling
- **Teensy Code**: Can use for consistent message IDs
- **Client Applications**: Can use same JSON for message interpretation

## Version Compatibility

The header includes version constants to ensure compatibility:
- `CALIBRATION_TEXTS_VERSION`: Data version (e.g., "1.0.0")
- `CALIBRATION_TEXTS_SCHEMA_VERSION`: Structure version (e.g., "1.0")

Check these constants in your code to ensure compatibility with expected data format.
