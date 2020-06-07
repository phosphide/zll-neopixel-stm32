//
//  uart_protocol.c
//  light_serial_protocol
//
//  Created by Jakub Tomczyński on 04/05/2020.
//  Copyright © 2020 Jakub Tomczyński. All rights reserved.
//

#include "uart_protocol.h"
#include <string.h>

const uint8_t digits[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
const uint8_t hex_to_dec[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0, 0, 10, 11, 12, 13, 14, 15};

uint8_t msg_[] = "RGB0,on=0,level=00,x=0000,y=0000,hue=00,saturation=00,enhancedhue=0000\n";
//                01234567890123456789012345678901234567890123456789012345678901234567890
//                0         1         2         3         4         5         6         7

uint8_t is_get(const uint8_t *msg, uint16_t length) {
	if (length == 3 && memcmp("GET", msg, 3) == 0) {
		return 1;
	}
	return 0;
}


uint16_t serialize(const tsZLL_ColourLightDevice *light, uint8_t **buffer_ptr) {
    sprint_b(msg_+8, light->sOnOffServerCluster.bOnOff);
    sprint_x8(msg_+16, light->sLevelControlServerCluster.u8CurrentLevel);
    sprint_x16(msg_+21, light->sColourControlServerCluster.u16CurrentX);
    sprint_x16(msg_+28, light->sColourControlServerCluster.u16CurrentY);
    sprint_x8(msg_+37, light->sColourControlServerCluster.u8CurrentHue);
    sprint_x8(msg_+51, light->sColourControlServerCluster.u8CurrentSaturation);
    sprint_x16(msg_+66, light->sColourControlServerCluster.u16EnhancedCurrentHue);
    *buffer_ptr = msg_;
    return sizeof(msg_)-1;
}

uint8_t deserialize(tsZLL_ColourLightDevice *light, const uint8_t *msg, uint16_t length) {
    if (length != sizeof(msg_)-2) {
        return 0;
    }
    sscan_b(msg+8, &light->sOnOffServerCluster.bOnOff);
    sscan_x8(msg+16, &light->sLevelControlServerCluster.u8CurrentLevel);
    sscan_x16(msg+21, &light->sColourControlServerCluster.u16CurrentX);
    sscan_x16(msg+28, &light->sColourControlServerCluster.u16CurrentY);
    sscan_x8(msg+37, &light->sColourControlServerCluster.u8CurrentHue);
    sscan_x8(msg+51, &light->sColourControlServerCluster.u8CurrentSaturation);
    sscan_x16(msg+66, &light->sColourControlServerCluster.u16EnhancedCurrentHue);
    return 1;
}
