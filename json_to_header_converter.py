#!/usr/bin/env python3
"""
JSON to C++ Header Converter for Calibration Texts
Converts calibration_texts.json to calibration_texts.h
"""

import json
import sys
from pathlib import Path

def escape_c_string(text):
    """Escape special characters for C++ string literals"""
    if text is None:
        return '""'
    return '"' + text.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n').replace('\r', '\\r') + '"'

def convert_json_to_header(json_file_path, output_file_path):
    """Convert JSON file to C++ header file"""
    
    # Read JSON file
    try:
        with open(json_file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except Exception as e:
        print(f"Error reading JSON file: {e}")
        return False
    
    # Start building header content
    header_content = []
    
    # Header guard and includes
    header_content.extend([
        '#ifndef CALIBRATION_TEXTS_H',
        '#define CALIBRATION_TEXTS_H',
        '',
        '// Auto-generated from calibration_texts.json',
        f'// Version: {data.get("version", "1.0.0")}',
        f'// Schema Version: {data.get("schema_version", "1.0")}',
        f'// Description: {data.get("description", "Automated Force Plate Calibration Messages and Texts")}',
        '',
        '// ============================================================================',
        '// CALIBRATION MESSAGE STRUCTURE (from JSON "messages" section)',
        '// ============================================================================',
        'struct CalibrationMessage {',
        '    const char* id;              // Message ID (e.g., "AC.INTRO.HEADER")',
        '    int code;                    // Numeric code from JSON',
        '    const char* type;            // Message type (header, body, prompt, etc.)',
        '    const char* text_lines[15];  // Text lines array (max 15 lines)',
        '    const char* placeholders[10]; // Placeholder names array (max 10 placeholders)',
        '    const char* condition;       // Optional condition string',
        '};',
        '',
        '// ============================================================================',
        '// CALIBRATION STATES ENUM (from JSON "states" section)',
        '// ============================================================================',
        'enum AutoCalState {'
    ])
    
    # Add states enum
    if 'states' in data:
        for state_name, state_id in data['states'].items():
            header_content.append(f'    {state_name} = {state_id},')
    
    header_content.extend([
        '};',
        '',
        '// ============================================================================',
        '// MATRIX POSITION STRUCTURE (from JSON "matrix_positions" section)',
        '// ============================================================================',
        'struct MatrixPosition {',
        '    int id;                      // Position ID (0-8)',
        '    float x;                     // X coordinate in mm',
        '    float y;                     // Y coordinate in mm',
        '    const char* description;     // Human-readable description',
        '};',
        '',
        '// ============================================================================',
        '// PLACEHOLDER DOCUMENTATION STRUCTURE (from JSON "usage_notes.placeholders")',
        '// ============================================================================',
        'struct PlaceholderInfo {',
        '    const char* name;            // Placeholder name (e.g., "mass_kg")',
        '    const char* description;     // Description of the placeholder',
        '};',
        '',
        '// ============================================================================',
        '// CONDITION DOCUMENTATION STRUCTURE (from JSON "usage_notes.conditions")',
        '// ============================================================================',
        'struct ConditionInfo {',
        '    const char* condition;       // Condition string (e.g., "step_c_enabled")',
        '    const char* description;     // Description of the condition',
        '};',
        '',
        '// ============================================================================',
        '// MESSAGE TYPE DOCUMENTATION STRUCTURE (from JSON "usage_notes.message_types")',
        '// ============================================================================',
        'struct MessageTypeInfo {',
        '    const char* type;            // Message type (e.g., "header")',
        '    const char* description;     // Description of the message type',
        '};',
        '',
        '// ============================================================================',
        '// CALIBRATION MESSAGES ARRAY',
        '// ============================================================================',
        '',
        'static const CalibrationMessage CALIBRATION_MESSAGES[] = {'
    ])
    
    # Add messages
    if 'messages' in data:
        message_count = 0
        for msg_id, msg_data in data['messages'].items():
            if message_count > 0:
                header_content.append(',')
            
            header_content.append('    {')
            header_content.append(f'        {escape_c_string(msg_id)}, {msg_data.get("code", 0)}, {escape_c_string(msg_data.get("type", "body"))},')
            
            # Text lines
            header_content.append('        {')
            text_lines = msg_data.get('text', [])
            for i, line in enumerate(text_lines):
                # Remove [AUTO-CAL] prefix if present
                clean_line = line.replace('[AUTO-CAL] ', '').replace('[AUTO-CAL]', '')
                header_content.append(f'            {escape_c_string(clean_line)},')
            header_content.append('            nullptr')
            header_content.append('        },')
            
            # Placeholders
            header_content.append('        {')
            placeholders = msg_data.get('placeholders', [])
            for placeholder in placeholders:
                header_content.append(f'            {escape_c_string(placeholder)},')
            header_content.append('            nullptr')
            header_content.append('        },')
            
            # Condition
            condition = msg_data.get('condition')
            header_content.append(f'        {escape_c_string(condition) if condition else "nullptr"}')
            header_content.append('    }')
            
            message_count += 1
    
    header_content.extend([
        '};',
        '',
        '// ============================================================================',
        '// MATRIX POSITIONS ARRAY (from JSON "matrix_positions" section)',
        '// ============================================================================',
        '',
        'static const MatrixPosition MATRIX_POSITIONS[] = {'
    ])
    
    # Add matrix positions
    if 'matrix_positions' in data:
        for i, pos in enumerate(data['matrix_positions']):
            if i > 0:
                header_content.append(',')
            header_content.append(f'    {{{pos["id"]}, {pos["x"]}, {pos["y"]}, {escape_c_string(pos["description"])}}}')
    
    header_content.extend([
        '};',
        '',
        '// ============================================================================',
        '// STATE NAMES ARRAY (from JSON "state_names" section)',
        '// ============================================================================',
        '',
        'static const char* STATE_NAMES[] = {'
    ])
    
    # Add state names
    if 'state_names' in data:
        for i, state_name in enumerate(data['state_names']):
            if i > 0:
                header_content.append(',')
            header_content.append(f'    {escape_c_string(state_name)}')
    
    header_content.extend([
        '};',
        '',
        '// ============================================================================',
        '// PLACEHOLDER DOCUMENTATION ARRAY (from JSON "usage_notes.placeholders")',
        '// ============================================================================',
        '',
        'static const PlaceholderInfo PLACEHOLDER_DOCS[] = {'
    ])
    
    # Add placeholder docs
    if 'usage_notes' in data and 'placeholders' in data['usage_notes']:
        placeholder_count = 0
        for name, desc in data['usage_notes']['placeholders'].items():
            if placeholder_count > 0:
                header_content.append(',')
            header_content.append(f'    {{{escape_c_string(name)}, {escape_c_string(desc)}}}')
            placeholder_count += 1
    
    header_content.extend([
        '};',
        '',
        '// ============================================================================',
        '// CONDITION DOCUMENTATION ARRAY (from JSON "usage_notes.conditions")',
        '// ============================================================================',
        '',
        'static const ConditionInfo CONDITION_DOCS[] = {'
    ])
    
    # Add condition docs
    if 'usage_notes' in data and 'conditions' in data['usage_notes']:
        condition_count = 0
        for condition, desc in data['usage_notes']['conditions'].items():
            if condition_count > 0:
                header_content.append(',')
            header_content.append(f'    {{{escape_c_string(condition)}, {escape_c_string(desc)}}}')
            condition_count += 1
    
    header_content.extend([
        '};',
        '',
        '// ============================================================================',
        '// MESSAGE TYPE DOCUMENTATION ARRAY (from JSON "usage_notes.message_types")',
        '// ============================================================================',
        '',
        'static const MessageTypeInfo MESSAGE_TYPE_DOCS[] = {'
    ])
    
    # Add message type docs
    if 'usage_notes' in data and 'message_types' in data['usage_notes']:
        msg_type_count = 0
        for msg_type, desc in data['usage_notes']['message_types'].items():
            if msg_type_count > 0:
                header_content.append(',')
            header_content.append(f'    {{{escape_c_string(msg_type)}, {escape_c_string(desc)}}}')
            msg_type_count += 1
    
    header_content.extend([
        '};',
        '',
        '// ============================================================================',
        '// CONSTANTS AND VERSION INFO',
        '// ============================================================================',
        '',
        '// Array size constants (automatically calculated)',
        '#define CALIBRATION_MESSAGES_COUNT (sizeof(CALIBRATION_MESSAGES) / sizeof(CalibrationMessage))',
        '#define MATRIX_POSITIONS_COUNT (sizeof(MATRIX_POSITIONS) / sizeof(MatrixPosition))',
        '#define STATE_NAMES_COUNT (sizeof(STATE_NAMES) / sizeof(char*))',
        '#define PLACEHOLDER_DOCS_COUNT (sizeof(PLACEHOLDER_DOCS) / sizeof(PlaceholderInfo))',
        '#define CONDITION_DOCS_COUNT (sizeof(CONDITION_DOCS) / sizeof(ConditionInfo))',
        '#define MESSAGE_TYPE_DOCS_COUNT (sizeof(MESSAGE_TYPE_DOCS) / sizeof(MessageTypeInfo))',
        '',
        '// Version info (from JSON file header)',
        f'#define CALIBRATION_TEXTS_VERSION "{data.get("version", "1.0.0")}"',
        f'#define CALIBRATION_TEXTS_SCHEMA_VERSION "{data.get("schema_version", "1.0")}"',
        f'#define CALIBRATION_TEXTS_DESCRIPTION "{data.get("description", "Automated Force Plate Calibration Messages and Texts")}"',
        '',
        '#endif // CALIBRATION_TEXTS_H'
    ])
    
    # Write header file
    try:
        with open(output_file_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(header_content))
        print(f"✅ Successfully generated {output_file_path}")
        return True
    except Exception as e:
        print(f"Error writing header file: {e}")
        return False

def main():
    """Main function"""
    # Default file paths
    json_file = "calibration_texts.json"
    header_file = "calibration_texts.h"
    
    # Check if JSON file exists
    if not Path(json_file).exists():
        print(f"❌ Error: {json_file} not found!")
        return 1
    
    # Convert JSON to header
    print(f"🔄 Converting {json_file} to {header_file}...")
    
    if convert_json_to_header(json_file, header_file):
        print("✅ Conversion completed successfully!")
        
        # Also create copy in esp32_ble_slave directory
        esp32_header = "esp32_ble_slave/calibration_texts.h"
        try:
            with open(header_file, 'r', encoding='utf-8') as src:
                content = src.read()
            with open(esp32_header, 'w', encoding='utf-8') as dst:
                dst.write(content)
            print(f"✅ Also created {esp32_header}")
        except Exception as e:
            print(f"⚠️  Warning: Could not copy to {esp32_header}: {e}")
        
        return 0
    else:
        print("❌ Conversion failed!")
        return 1

if __name__ == "__main__":
    sys.exit(main())
