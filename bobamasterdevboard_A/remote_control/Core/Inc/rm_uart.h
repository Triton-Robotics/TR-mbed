/*
 * rm_uart.h
 *
 *  Created on: Jan 27, 2020
 *      Author: WYT
 */

#ifndef INC_RM_UART_H_
#define INC_RM_UART_H_

#include "main.h"

#define UART_RX_DMA_SIZE (1024)
#define DBUS_MAX_LEN     (36u)
#define DBUS_BUFLEN      (18u)
#define DBUS_HUART       huart1 /* for dji remote controler reciever */

/**
  * @brief  remote control information
  */
typedef struct
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  int16_t wheel;

  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;

	struct {
		int16_t x;
		int16_t y;
		int16_t z;

		uint8_t l;
		uint8_t r;
	}mouse;

	union {
		uint16_t key_code;
	/**********************************************************************************
	   * Channel: 15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
	   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
	************************************************************************************/
		struct
		    {
		      uint16_t W : 1;
		      uint16_t S : 1;
		      uint16_t A : 1;
		      uint16_t D : 1;
		      uint16_t SHIFT : 1;
		      uint16_t CTRL : 1;
		      uint16_t Q : 1;
		      uint16_t E : 1;
		      uint16_t R : 1;
		      uint16_t F : 1;
		      uint16_t G : 1;
		      uint16_t Z : 1;
		      uint16_t X : 1;
		      uint16_t C : 1;
		      uint16_t V : 1;
		      uint16_t B : 1;
		    } bit;
	} kb;
} rc_info_t;

extern rc_info_t rc;
extern uint8_t dbus_buf[DBUS_MAX_LEN];

void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);

#endif /* INC_RM_UART_H_ */
