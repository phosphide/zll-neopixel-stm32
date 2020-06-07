//
//  uart_protocol.h
//  light_serial_protocol
//
//  Created by Jakub Tomczyński on 04/05/2020.
//  Copyright © 2020 Jakub Tomczyński. All rights reserved.
//

#ifndef uart_protocol_h
#define uart_protocol_h

#ifdef JENNIC_CHIP
#include "colour_light.h"
#include "zps_gen.h"
#include "AppHardwareApi_JN5168.h"
#else
#include <stdbool.h>
#include <stdint.h>

typedef bool zbool;

typedef struct {
    zbool bOnOff;
} tsCLD_OnOff;

typedef struct {
    uint8_t u8CurrentLevel;
} tsCLD_LevelControl;

typedef struct {
    uint16_t u16CurrentX;
    uint16_t u16CurrentY;
    uint8_t u8CurrentHue;
    uint8_t u8CurrentSaturation;
    uint16_t u16EnhancedCurrentHue;
} tsCLD_ColourControl;

typedef struct {
    tsCLD_OnOff sOnOffServerCluster;
    tsCLD_LevelControl sLevelControlServerCluster;
    tsCLD_ColourControl sColourControlServerCluster;
} tsZLL_ColourLightDevice;
#endif

extern const uint8_t digits[];
extern const uint8_t hex_to_dec[];

static inline void sprint_x16(uint8_t *str, uint16_t val) {
    str[3] = digits[val & 0x000F];
    val >>= 4;
    str[2] = digits[val & 0x000F];
    val >>= 4;
    str[1] = digits[val & 0x000F];
    val >>= 4;
    str[0] = digits[val & 0x000F];
}

static inline void sprint_x8(uint8_t *str, uint8_t val) {
    str[1] = digits[val & 0x0F];
    str[0] = digits[val >> 4];
}
static inline void sprint_b(uint8_t *str, zbool val) {
    str[0] = val ? '1' : '0';
}
static inline void sscan_x16(const uint8_t *str, uint16_t *val) {
    *val = hex_to_dec[str[0]-'0'] << 12 | hex_to_dec[str[1]-'0'] << 8 |
    hex_to_dec[str[2]-'0'] << 4 | hex_to_dec[str[3]-'0'];
}
static inline void sscan_x8(const uint8_t *str, uint8_t *val) {
    *val = hex_to_dec[str[0]-'0'] << 4 | hex_to_dec[str[1]-'0'];
}
static inline void sscan_b(const uint8_t *str, zbool *val) {
    *val = *str == '1' ? 1 : 0;
}

uint8_t is_get(const uint8_t *msg, uint16_t length);
uint16_t serialize(const tsZLL_ColourLightDevice *light, uint8_t **buffer_ptr);
uint8_t deserialize(tsZLL_ColourLightDevice *light, const uint8_t *msg, uint16_t length);

#endif /* uart_protocol_h */
