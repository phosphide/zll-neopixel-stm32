/*
 * ws2812b.c
 *
 *  Created on: Apr 30, 2020
 *      Author: phosphide
 */

#include "ws2812b.h"
#include <stdbool.h>

__IO bool WS2812_DMA_busy;
static uint8_t tx_buf_a[13] = {0};
static uint8_t tx_buf_b[13] = {0};
static uint8_t *next_tx_buf = tx_buf_a;

void ws2812_DMA_TC_callback() {
	WS2812_DMA_busy = false;
}

void ws2812_send(rgb888_t *values, uint16_t count) {
	WRITE_REG(DMA1_Channel3->CPAR, (uint32_t)&SPI1->DR);
	//LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)tx_buf, (uint32_t)&SPI1->DR, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

	for (uint16_t i = 0; i < count; i++) {
		uint32_t color;
		((uint8_t*)&color)[0] = values[i].b;
		((uint8_t*)&color)[1] = values[i].r;
		((uint8_t*)&color)[2] = values[i].g;

		for (int b = 0; b < 12; b++) {
			next_tx_buf[11-b] = (color & (1 << (b*2+1)) ? 0xE0 : 0x80) | (color & (1 << (b*2)) ? 0x0E : 0x08);
		}
		while(WS2812_DMA_busy);
		WS2812_DMA_busy = 1;
		CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN); // LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
		WRITE_REG(DMA1_Channel3->CMAR, (uint32_t)next_tx_buf);
		MODIFY_REG(DMA1_Channel3->CNDTR, DMA_CNDTR_NDT, 13); // LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 12);
		SET_BIT(DMA1_Channel3->CCR, DMA_CCR_EN); // LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
		if (next_tx_buf == tx_buf_a) {
			next_tx_buf = tx_buf_b;
		} else {
			next_tx_buf = tx_buf_a;
		}
	}
}
