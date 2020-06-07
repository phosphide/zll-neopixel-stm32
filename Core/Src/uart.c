/*
 * uart.c
 *
 *  Created on: Apr 29, 2020
 *      Author: phosphide
 */

#include "uart.h"
#include <stdbool.h>
#include "stm32l0xx_ll_dma.h"
#include "stm32l0xx_ll_usart.h"

__IO bool UART_DMA_busy;

void UART_DMA_TC_callback() {
	UART_DMA_busy = false;
}

void UART_send(uint8_t *data, uint16_t length) {
	while (UART_DMA_busy);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)data);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, length);
	LL_USART_EnableDMAReq_TX(USART2);
	UART_DMA_busy = true;
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}
