/* Copyright 2016, Fernandez Martin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef UART_DRIVER_MF_H_
#define UART_DRIVER_MF_H_

/*==========================[inclusions]=================================*/

#include "board.h"
#include "chip.h"

/*==================[macros and definitions]=================================*/

/* -------------- USART0 Port definition and configuration -------------- */

#define U0_TXD_PORT   9
#define U0_TXD_PIN    5
#define U0_TXD_FUNC   (SCU_MODE_INACT | SCU_MODE_FUNC7) /* [UM:Table 190] */

#define U0_RXD_PORT   9
#define U0_RXD_PIN    6
#define U0_RXD_FUNC   (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC7)

#define U0_BAUD_RATE  115200
#define U0_FIFO_CONF  (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV0)

#define U0_SETUP      (UART_LCR_WLEN8 | UART_LCR_PARITY_DIS | UART_LCR_SBS_1BIT)


/* -------------- USART2 Port definition and configuration -------------- */

#define U2_TXD_PORT   2
#define U2_TXD_PIN    10
#define U2_TXD_FUNC   (SCU_MODE_INACT | SCU_MODE_FUNC2)  /* [UM:Table 190] */

/* -------- CIAA USB UART ---------- */
//#define U2_TXD_PORT   7
//#define U2_TXD_PIN    1
//#define U2_TXD_FUNC   (SCU_MODE_INACT | SCU_MODE_FUNC6)  /* [UM:Table 190] */

#define U2_RXD_PORT   2
#define U2_RXD_PIN    11
#define U2_RXD_FUNC   (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC2)

/* -------- CIAA USB UART ---------- */
//#define U2_RXD_PORT   7
//#define U2_RXD_PIN    2
//#define U2_RXD_FUNC   (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC6)

#define U2_BAUD_RATE  38400 // TODO: remove resistor Pin13 (GIO) and change value back to 115200 [datasheet page 8]
#define U2_FIFO_CONF  (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV3)

#define U2_SETUP      (UART_LCR_WLEN8 | UART_LCR_PARITY_DIS | UART_LCR_SBS_1BIT) /* NMEA Standard:
																						8-bit word length
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 		Parity disabled
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 		1 stop bit */

/* -------------- USART3 Port definition and configuration -------------- */

#define U3_TXD_PORT   4
#define U3_TXD_PIN    1
#define U3_TXD_FUNC   (SCU_MODE_INACT | SCU_MODE_FUNC6)  /* [UM:Table 190] */

#define U3_RXD_PORT   4
#define U3_RXD_PIN    2
#define U3_RXD_FUNC   (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC6)

#define U3_BAUD_RATE  115200
#define U3_FIFO_CONF  (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV0)

#define U3_SETUP      (UART_LCR_WLEN8 | UART_LCR_PARITY_DIS | UART_LCR_SBS_1BIT)


/* -------------------- General definitions -------------------- */

#define RS485_CONF (UART_RS485CTRL_DCTRL_EN | UART_RS485CTRL_OINV_1)		/* [UM:40.6.16] */

/*=======================[data declaration]===========================*/

typedef enum _uarts
{
	UART0 = 0,
	UART2 = 1,
	UART3 = 2
}uarts_e;

/*======================[functions declaration]=========================*/

/**
 * @brief	Initializes target UART in a ring buffer configuration
 * @param	n			: UART number, should be 0, 2 or 3
 * @return	Nothing
 * @note	Initializes UART peripheral to use its fifo and also
 * work with ring buffers
 */
void UART_Init(uarts_e n);

/**
 * @brief	Deinitializes target UART
 * @param	n			: UART number, should be 0, 2 or 3
 * @return	Nothing
 */
void UART_DeInit(uarts_e n);

/**
 * @brief	Sets the target UART's baud rate
 * @param	n			: UART number, should be 0, 2 or 3
 * @param	bauds		: desired baud rate
 * @return	Nothing
 */
void UART_BaudsConfig(uarts_e n, uint32_t bauds);

/**
 * @brief	Enables target UART's RS485 mode
 * @param	n			: UART number, should be 0, 2 or 3
 * @return	Nothing
 */
void UART_RS485Enable(uarts_e n);

/**
 * @brief	Writes N bytes on tx ring buffer of target UART
 * @param	n			: UART number, should be 0, 2 or 3
 * @param	buffer		: pointer to data
 * @param	nbytes		: number of bytes to be written
 * @return	retval		: number of bytes successfully written
 */
uint32_t UART_WriteNBytes(uarts_e n, uint8_t *buffer, uint32_t nbytes);

/**
 * @brief	Checks number of bytes stored in target UART's rx ring buffer
 * @param	n			: UART number, should be 0, 2 or 3
 * @return	retval		: number of bytes being held in the ring buffer
 */
uint32_t UART_DataCount(uarts_e n);

/**
 * @brief	Reads all available data from target UART's rx ring buffer
 * @param	n			: UART number, should be 0, 2 or 3
 * @param	buffer		: pointer to where available data will be written
 * @return	retval		: number of bytes read from target UART
 */
uint32_t UART_GetData(uarts_e n, uint8_t * buffer);

/**
 * @brief	Reads N bytes from target UART's rx ring buffer
 * @param	n			: UART number, should be 0, 2 or 3
 * @param	buffer		: pointer to where available data will be written
 * @param	nbytes		: number of bytes to be read
 * @return	retval		: number of bytes read from target UART
 */
uint32_t UART_GetNBytes(uarts_e n, uint8_t * buffer, uint32_t nbytes);

/**
 * @brief	Checks if the UART rx is busy
 * @param	n			: UART number, should be 0, 2 or 3
 * @return	TRUE if the UART Rx FIFO isn't busy
 * @return	FALSE if the RBR holds an unread character
 */
Bool UART_CheckRxEmpty(uarts_e n);

#endif /* UART_DRIVER_MF_H_ */
