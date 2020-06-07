/*
 * uart.h
 *
 *  Created on: Apr 29, 2020
 *      Author: phosphide
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32l0xx.h"

void UART_DMA_TC_callback();

void UART_send(uint8_t *data, uint16_t length);

#endif /* INC_UART_H_ */
