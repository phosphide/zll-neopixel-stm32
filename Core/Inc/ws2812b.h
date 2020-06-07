/*
 * ws2812b.h
 *
 *  Created on: Apr 30, 2020
 *      Author: phosphide
 */

#ifndef INC_WS2812B_H_
#define INC_WS2812B_H_

#include "stm32l0xx.h"
#include "colors.h"

void ws2812_DMA_TC_callback();
void ws2812_send(rgb888_t *values, uint16_t count);

#endif /* INC_WS2812B_H_ */
