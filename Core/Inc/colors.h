//
//  colorspace.h
//  light_serial_protocol
//
//  Created by Jakub Tomczyński on 12/05/2020.
//  Copyright © 2020 Jakub Tomczyński. All rights reserved.
//

#ifndef colors_h
#define colors_h

#include <stdint.h>

typedef struct {
    float r;       // a fraction between 0 and 1
    float g;       // a fraction between 0 and 1
    float b;       // a fraction between 0 and 1
} rgb32f_t;

typedef struct {
    float h;       // angle in degrees
    float s;       // a fraction between 0 and 1
    float v;       // a fraction between 0 and 1
} hsv32f_t;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb888_t;

typedef struct hsv888_t
{
    unsigned char h;
    unsigned char s;
    unsigned char v;
} hsv888_t;

hsv32f_t rgb32f_to_hsv32f(rgb32f_t in); // David H
rgb32f_t hsv32f_to_rgb32f(hsv32f_t in); // David H

hsv888_t rgb888_to_hsv888(rgb888_t rgb); // Leszek Szary
rgb888_t hsv888_to_rgb888(hsv888_t hsv); // Leszek Szary

#endif /* colors_h */
