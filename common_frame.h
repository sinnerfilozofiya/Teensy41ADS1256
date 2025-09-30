#ifndef COMMON_FRAME_H
#define COMMON_FRAME_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>

// Radio header magic number
#define RADIO_MAGIC 0xB66B

// Frame sizes
#define INNER_FRAME_SIZE 144
#define RADIO_HEADER_SIZE 6
#define UDP_PACKET_SIZE (RADIO_HEADER_SIZE + INNER_FRAME_SIZE)

// Radio header for UDP packets (6 bytes)
struct __attribute__((packed)) RadioHdr {
    uint16_t magic;     // 0xB66B (little-endian)
    uint16_t seq;       // Sequence number (wraps at 65535)
    uint16_t len;       // Payload length (always 144)
};

// ESP-NOW command packet (16 bytes)
struct __attribute__((packed)) ESPNowCommand {
    uint8_t magic[2];   // 0xCM, 0xD1 (Command magic)
    uint8_t command[8]; // Command string (null-terminated)
    uint32_t timestamp; // Timestamp for deduplication
    uint16_t crc16;     // CRC16 of packet
};

// Inner frame structure (from Teensy, 144 bytes total)
struct __attribute__((packed)) InnerFrame {
    uint8_t  sync[2];       // 0xA5, 0x5A
    uint8_t  plate_id;
    uint8_t  proto_ver;
    uint16_t frame_idx;     // Inner sequence number
    uint32_t t0_us;         // Timestamp
    uint8_t  samples[120];  // ADC samples (10 samples × 4 channels × 3 bytes)
    uint16_t crc16;         // CRC of frame up to this point
    uint8_t  pad[12];       // Padding to 144 bytes
};

// Configuration structure
struct RadioConfig {
    // Wi-Fi settings
    char ssid[32];
    char password[64];
    char dest_ip[16];       // For TX: destination IP
    uint16_t port;          // UDP port (default 30303)
    
    // ESP-NOW settings
    uint8_t peer_mac[6];    // Peer MAC address
    uint8_t channel;        // Wi-Fi channel (1-13)
    
    // Custom MAC settings
    bool use_custom_mac;
    uint8_t custom_sta_mac[6];
    uint8_t custom_ap_mac[6];
    
    // Transport selection
    bool use_wifi_udp;      // true = Wi-Fi UDP, false = ESP-NOW
};

// Default configuration
static const RadioConfig DEFAULT_CONFIG = {
    .ssid = "YourWiFiSSID",
    .password = "YourWiFiPassword",
    .dest_ip = "192.168.1.100",
    .port = 30303,
    .peer_mac = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x02},  // Default RX MAC
    .channel = 1,
    .use_custom_mac = true,
    .custom_sta_mac = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x01},  // TX MAC
    .custom_ap_mac = {0x02, 0xAA, 0xBB, 0x00, 0x01, 0x01},
    .use_wifi_udp = true
};

// CRC16 CCITT-FALSE implementation
static inline uint16_t crc16_ccitt_false(const uint8_t* data, uint32_t length) {
    uint16_t crc = 0xFFFF;
    while (length--) {
        crc ^= ((uint16_t)*data++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

// Validate inner frame (SYNC bytes and CRC)
static inline bool validate_inner_frame(const InnerFrame* frame) {
    if (frame->sync[0] != 0xA5 || frame->sync[1] != 0x5A) {
        return false;
    }
    
    uint16_t expected_crc = crc16_ccitt_false((const uint8_t*)frame, 
                                              offsetof(InnerFrame, crc16));
    return frame->crc16 == expected_crc;
}

// MAC address helper functions
static inline bool parse_mac(const char* str, uint8_t mac[6]) {
    int values[6];
    if (sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
               &values[0], &values[1], &values[2],
               &values[3], &values[4], &values[5]) == 6) {
        for (int i = 0; i < 6; i++) {
            mac[i] = (uint8_t)values[i];
        }
        return true;
    }
    return false;
}

static inline void make_locally_administered(uint8_t mac[6]) {
    mac[0] |= 0x02;  // Set locally administered bit
    mac[0] &= 0xFE;  // Clear multicast bit (ensure unicast)
}

static inline bool is_locally_administered_unicast(const uint8_t mac[6]) {
    return (mac[0] & 0x02) && !(mac[0] & 0x01);
}

static inline void print_mac(const char* label, const uint8_t mac[6]) {
    printf("%s: %02X:%02X:%02X:%02X:%02X:%02X\n", 
           label, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Extract load cell data from inner frame samples
static inline void extract_load_cell_sample(const uint8_t* sample_data, int sample_idx, 
                                           int32_t* lc1, int32_t* lc2, int32_t* lc3, int32_t* lc4) {
    const uint8_t* s = sample_data + (sample_idx * 12);  // 4 channels × 3 bytes each
    
    // Unpack 24-bit little-endian signed values
    auto unpack_int24_le = [](const uint8_t* p) -> int32_t {
        uint32_t u = p[0] | (p[1] << 8) | (p[2] << 16);
        if (u & 0x800000) {
            u |= 0xFF000000;  // Sign extend
        }
        return (int32_t)u;
    };
    
    *lc1 = unpack_int24_le(s + 0);
    *lc2 = unpack_int24_le(s + 3);
    *lc3 = unpack_int24_le(s + 6);
    *lc4 = unpack_int24_le(s + 9);
}

// ESP-NOW Command helper functions
static inline bool create_espnow_command(ESPNowCommand* cmd, const char* command_str) {
    if (strlen(command_str) >= 8) return false; // Command too long
    
    memset(cmd, 0, sizeof(ESPNowCommand));
    cmd->magic[0] = 0xCM;
    cmd->magic[1] = 0xD1;
    strncpy((char*)cmd->command, command_str, 7);
    cmd->command[7] = '\0';
    cmd->timestamp = millis();
    
    // Calculate CRC16 of everything except CRC field
    cmd->crc16 = crc16_ccitt_false((const uint8_t*)cmd, sizeof(ESPNowCommand) - 2);
    return true;
}

static inline bool validate_espnow_command(const ESPNowCommand* cmd) {
    if (cmd->magic[0] != 0xCM || cmd->magic[1] != 0xD1) {
        return false;
    }
    
    uint16_t expected_crc = crc16_ccitt_false((const uint8_t*)cmd, sizeof(ESPNowCommand) - 2);
    return cmd->crc16 == expected_crc;
}

#endif // COMMON_FRAME_H
