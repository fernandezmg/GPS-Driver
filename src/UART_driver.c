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

/*==================[inclusions]=============================================*/

#include "../../Drivers/inc/UART_driver.h"

#include <string.h>

/*==================[macros and definitions]=================================*/

/* ---------------------- Size definitions ---------------------- */

#define UART_RB_SIZE 1024

/*==================[internal data declaration]==============================*/

static uint8_t rxbuf[3][UART_RB_SIZE];
static uint8_t txbuf[3][UART_RB_SIZE];

static RINGBUFF_T rrb[3];
static RINGBUFF_T trb[3];

typedef struct _uartData
{
	LPC_USART_T * uart;
	RINGBUFF_T * rrb;
	RINGBUFF_T * trb;
	uint8_t	rxport;
	uint8_t rxpin;
	uint16_t rxmodefunc;
	uint8_t	txport;
	uint8_t txpin;
	uint16_t txmodefunc;
	uint32_t baudrate;
	uint32_t fifoconf;
	uint32_t setup;
	IRQn_Type uartirq;

}uartData_t;

static uartData_t uarts[3] =
{
	{ LPC_USART0, &(rrb[0]), &(trb[0]), U0_RXD_PORT, U0_RXD_PIN, U0_RXD_FUNC,
			U0_TXD_PORT, U0_TXD_PIN, U0_TXD_FUNC, U0_BAUD_RATE, U0_FIFO_CONF, U0_SETUP, USART0_IRQn },
	{ LPC_USART2, &(rrb[1]), &(trb[1]), U2_RXD_PORT, U2_RXD_PIN, U2_RXD_FUNC,
			U2_TXD_PORT, U2_TXD_PIN, U2_TXD_FUNC, U2_BAUD_RATE, U2_FIFO_CONF, U2_SETUP, USART2_IRQn },
	{ LPC_USART3, &(rrb[2]), &(trb[2]), U3_RXD_PORT, U3_RXD_PIN, U3_RXD_FUNC,
			U3_TXD_PORT, U3_TXD_PIN, U3_TXD_FUNC, U3_BAUD_RATE, U3_FIFO_CONF, U3_SETUP, USART3_IRQn },
};

/*==================[internal functions declaration]=========================*/

/**
 * @brief	Initializes target UART
 * @param	n			: UART number, should be 0, 2 or 3
 * @return	Nothing
 * @note	Initializes UART peripheral to use its fifo and also
 * work with ring buffers
 */
void UART_HardwareInit(uarts_e n);

/**
 * 	@brief general UART interrupt handler
 *	@param	n			: UART number, should be 0, 2 or 3
 *	@return none
 */
void uart_irq(uarts_e n);

/*==================[internal functions definition]=========================*/

void UART_HardwareInit(uarts_e n)
{
	/* USART configuration */

	// Set Rx and Tx pin modes and functions
	Chip_SCU_PinMuxSet(uarts[n].rxport, uarts[n].rxpin, uarts[n].rxmodefunc);
	Chip_SCU_PinMuxSet(uarts[n].txport, uarts[n].txpin, uarts[n].txmodefunc); /* [UM:17.4.1] */

	// USARTn peripheral configuration
	Chip_UART_Init(uarts[n].uart);
	Chip_UART_SetBaud(uarts[n].uart, uarts[n].baudrate); /* [UM:40.6.3] [UM:40.6.12] */
	Chip_UART_SetupFIFOS(uarts[n].uart, uarts[n].fifoconf); /* [UM:40.6.6] */
	Chip_UART_ConfigData(uarts[n].uart, uarts[n].setup); /* [UM:40.6.7] */
	Chip_UART_TXEnable(uarts[n].uart); /* [UM:40.6.20] */

	// USARTn RS485 configuration
	//Chip_UART_SetRS485Flags(uarts[n].uart, RS485_CONF); /* [UM:40.6.16] */

	// Enable USARTn RBR and THRE interrupt
	Chip_UART_IntEnable(uarts[n].uart, (UART_IER_RBRINT | UART_IER_THREINT)); /* [UM:40.6.4] */

	// Enable USARTn interrupt in the NVIC. Should be off if DMA mode is being used
	NVIC_ClearPendingIRQ(uarts[n].uartirq); /* [UM:9.7] */
	NVIC_EnableIRQ(uarts[n].uartirq);
}

Bool UART_CheckRxEmpty(uarts_e n)
{
	LPC_USART_T *pUART = uarts[n].uart;
	if (pUART->LSR & UART_LSR_RDR) { /* [UM 40.6.8] */
			return FALSE;
		}
		else {
			return TRUE;
		}
}

/*==================[external functions definition]==========================*/

void UART_Init(uarts_e n)
{
	UART_HardwareInit(n);

	// Initialize receive and transmit buffers
	memset(rxbuf[n], 0, UART_RB_SIZE);
	memset(txbuf[n], 0, UART_RB_SIZE);

	// Initialize the ring buffers
	RingBuffer_Init(uarts[n].rrb, rxbuf[n], 1, UART_RB_SIZE);
	RingBuffer_Init(uarts[n].trb, txbuf[n], 1, UART_RB_SIZE);
}

void UART_DeInit(uarts_e n)
{
	Chip_UART_DeInit(uarts[n].uart);
}

void UART_BaudsConfig(uarts_e n, uint32_t bauds)
{
	Chip_UART_SetBaud(uarts[n].uart, bauds); /* [UM:40.6.3] [UM:40.6.12] */
}

void UART_RS485Enable(uarts_e n)
{
	Chip_UART_SetRS485Flags(uarts[n].uart, RS485_CONF); /* [UM:40.6.16] */
}

/* -------------- user functions -------------- */

uint32_t UART_DataCount(uarts_e n)
{
	uint32_t nbytes = RingBuffer_GetCount(uarts[n].rrb);

	return nbytes;
}

uint32_t UART_GetData(uarts_e n, uint8_t *buffer)
{
	uint32_t nbytes, retval;
	nbytes = RingBuffer_GetCount(uarts[n].rrb);
	retval = Chip_UART_ReadRB(uarts[n].uart, uarts[n].rrb, buffer, nbytes);

	return retval;
}

uint32_t UART_GetNBytes(uarts_e n, uint8_t *buffer, uint32_t nbytes)
{
	uint32_t retval;
	retval = Chip_UART_ReadRB(uarts[n].uart, uarts[n].rrb, buffer, nbytes);

	return retval;
}

uint32_t UART_WriteNBytes(uarts_e n, uint8_t *buffer, uint32_t nbytes)
{
	uint32_t retval;
	retval = Chip_UART_SendRB(uarts[n].uart, uarts[n].trb, buffer, nbytes);

	return retval;
}


/*==================[interrupt handling]==========================*/

void UART0_IRQHandler(void)
{
	uart_irq(UART0);
}

void UART2_IRQHandler(void)
{
	uart_irq(UART2);
}

void UART3_IRQHandler(void)
{
	uart_irq(UART3);
}

void uart_irq(uarts_e n)
{
	Chip_UART_IRQRBHandler(uarts[n].uart, uarts[n].rrb, uarts[n].trb);
}
