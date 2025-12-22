// ============================================================================
// MOCK DATA GENERATOR
// ============================================================================
// This file provides mock data generation for testing data rates and reliability
// without requiring physical load cells. Can be easily removed by deleting this file.
//
// Wave types per channel:
//   Channel 0: Square wave
//   Channel 1: Triangle wave
//   Channel 2: Sawtooth wave
//   Channel 3: Sine wave
// ============================================================================

#ifndef MOCK_DATA_GENERATOR_DEFINED
#define MOCK_DATA_GENERATOR_DEFINED

#include <math.h>

// Mock data enable flag
static bool mock_data_enabled = false;

// Wave generation parameters
static const float MOCK_FREQUENCY = 1.0f;  // 1 Hz base frequency
static const int32_t MOCK_AMPLITUDE = 100000;  // Amplitude in counts (adjustable)
static const int32_t MOCK_OFFSET = 0;  // DC offset

// Time tracking for wave generation
static uint32_t mock_start_time = 0;

// ============================================================================
// WAVE GENERATION FUNCTIONS
// ============================================================================

// Square wave: y = sign(sin(2πft))
// Range: {-1, +1} or {0, 1}
static inline int32_t generate_square_wave(float t, float frequency) {
  float phase = 2.0f * 3.14159265359f * frequency * t;
  float sine_val = sin(phase);
  return (sine_val >= 0.0f) ? MOCK_AMPLITUDE : -MOCK_AMPLITUDE;
}

// Triangle wave: y = (2/π) * arcsin(sin(2πft))
// Range: [-1, 1]
static inline int32_t generate_triangle_wave(float t, float frequency) {
  float phase = 2.0f * 3.14159265359f * frequency * t;
  float sine_val = sin(phase);
  float triangle_val = (2.0f / 3.14159265359f) * asin(sine_val);
  return (int32_t)(triangle_val * MOCK_AMPLITUDE);
}

// Sawtooth wave: y = 2(tf - floor(tf + 0.5))
// Range: [-1, 1]
static inline int32_t generate_sawtooth_wave(float t, float frequency) {
  float phase = t * frequency;
  float sawtooth_val = 2.0f * (phase - floor(phase + 0.5f));
  return (int32_t)(sawtooth_val * MOCK_AMPLITUDE);
}

// Sine wave: y = sin(2πft)
// Range: [-1, 1]
static inline int32_t generate_sine_wave(float t, float frequency) {
  float phase = 2.0f * 3.14159265359f * frequency * t;
  float sine_val = sin(phase);
  return (int32_t)(sine_val * MOCK_AMPLITUDE);
}

// ============================================================================
// PUBLIC API
// ============================================================================

// Enable/disable mock data generation
void mock_data_set_enabled(bool enabled) {
  mock_data_enabled = enabled;
  if (enabled) {
    mock_start_time = millis();
    Serial.println("[MOCK] Mock data generation ENABLED");
    Serial.println("[MOCK] Ch0: Square, Ch1: Triangle, Ch2: Sawtooth, Ch3: Sine");
  } else {
    Serial.println("[MOCK] Mock data generation DISABLED");
  }
}

bool mock_data_is_enabled() {
  return mock_data_enabled;
}

// Generate mock data for a specific channel
int32_t mock_data_generate(uint8_t channel) {
  if (!mock_data_enabled || channel >= 4) {
    return 0;
  }
  
  // Calculate time in seconds since mock data was enabled
  float t = (millis() - mock_start_time) / 1000.0f;
  
  // Generate different wave types for each channel
  int32_t value = 0;
  switch (channel) {
    case 0:
      // Square wave
      value = generate_square_wave(t, MOCK_FREQUENCY);
      break;
      
    case 1:
      // Triangle wave
      value = generate_triangle_wave(t, MOCK_FREQUENCY);
      break;
      
    case 2:
      // Sawtooth wave
      value = generate_sawtooth_wave(t, MOCK_FREQUENCY);
      break;
      
    case 3:
      // Sine wave
      value = generate_sine_wave(t, MOCK_FREQUENCY);
      break;
      
    default:
      value = 0;
      break;
  }
  
  // Add offset
  return value + MOCK_OFFSET;
}

#endif // MOCK_DATA_GENERATOR_DEFINED
