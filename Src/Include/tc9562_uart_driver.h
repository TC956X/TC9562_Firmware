/* ============================================================================
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Toshiba Electronic Devices & Storage Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ========================================================================= */

/*! History:
 *  30 Sep 2019 : Base lined
 *  VERSION     : 1.0.0
 */

/*
 *********************************************************************************************************
 *
 * Target        : TC9562
 * Filename      : tc9562_uart_driver.h
 *
 *********************************************************************************************************
 */

#ifndef TC9562_UART_DRIVER_H_
#define TC9562_UART_DRIVER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "Driver_USART.h"
#include "tc9562_regacc.h"
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

#define UART0_REG_BASE 0x40006000U
#define UART1_REG_BASE 0x40007000U

#define TC9562_UART0_ENABLE  0x11U
#define TC9562_UART1_ENABLE  0x1100U

#define UART0 0U
#define UART1 1U

#define UART_DR    0x000U
#define UART_RSR   0x004U
#define UART_ECR   0x004U
#define UART_FR    0x018U
#define UART_IBRD  0x024U
#define UART_FBRD  0x028U
#define UART_LCR_H 0x02cU
#define UART_CR    0x030U
#define UART_IFLS  0x034U
#define UART_IMSC  0x038U
#define UART_RIS   0x03cU
#define UART_ICR   0x044U

#define RXRIS  0x010U
#define TXRIS  0x020U
#define RTRIS  0x040U
#define FERIS  0x080U
#define PERIS  0x100U
#define BERIS  0x200U
#define OERIS  0x400U

/* Bit positions of UART Error clear register (0x004) */
#define UART_FE_ECR_BIT_POS  0
#define UART_PE_ECR_BIT_POS  1
#define UART_BE_ECR_BIT_POS  2
#define UART_OE_ECR_BIT_POS  3

/* Bit positions of UART Line Control register (0x02c) */
#define UART_LCR_H_BRK_BIT_POS  0
#define UART_LCR_H_PEN_BIT_POS  1
#define UART_LCR_H_EPS_BIT_POS  2
#define UART_LCR_H_STP2_BIT_POS 3
#define UART_LCR_H_FEN_BIT_POS  4
#define UART_LCR_H_WLEN_BIT_POS 5
#define UART_LCR_H_SPS_BIT_POS  7

/* Bit positions of UART Control register (0x030) */
#define UART_CR_UART_EN_BIT_POS  0
#define UART_CR_SIR_EN_BIT_POS  1
#define UART_CR_SIR_LP_BIT_POS  2
#define UART_CR_LBE_BIT_POS  7
#define UART_CR_TXE_BIT_POS  8
#define UART_CR_RXE_BIT_POS  9
#define UART_CR_DTR_BIT_POS  10
#define UART_CR_RTS_BIT_POS  11

/* FIFO Level enable bit*/
#define TC9562_USART_FIFO_ENABLE (1<<20)
#define TC9562_USART_FIFO_DISABLE (1<<27)

/* Bit positions of UART FIFO level selection register (0x034) */
#define UART_TX_FIFO_LVL_SEL_BIT_POS  0
#define UART_RX_FIFO_LVL_SEL_BIT_POS  3

#define UART_FIFO_DEFAULT 2
/* Bit positions for TX-FIFO */
#define TC9562_USART_TXIFLSEL 21
#define TC9562_USART_TXIFLSEL_1_2  (5<<TC9562_USART_TXIFLSEL)
#define TC9562_USART_TXIFLSEL_1_4  (1<<TC9562_USART_TXIFLSEL)
#define TC9562_USART_TXIFLSEL_1_8  (2<<TC9562_USART_TXIFLSEL)
#define TC9562_USART_TXIFLSEL_3_4  (3<<TC9562_USART_TXIFLSEL)
#define TC9562_USART_TXIFLSEL_7_8  (4<<TC9562_USART_TXIFLSEL)

/* Bit positions for RX-FIFO */
#define TC9562_USART_RXIFLSEL 24
#define TC9562_USART_RXIFLSEL_1_2  (5<<TC9562_USART_RXIFLSEL)
#define TC9562_USART_RXIFLSEL_1_4  (1<<TC9562_USART_RXIFLSEL)
#define TC9562_USART_RXIFLSEL_1_8  (2<<TC9562_USART_RXIFLSEL)
#define TC9562_USART_RXIFLSEL_3_4  (3<<TC9562_USART_RXIFLSEL)
#define TC9562_USART_RXIFLSEL_7_8  (4<<TC9562_USART_RXIFLSEL)


/* Bit positions of UART Interrupt related register (0x034) */
#define UART_INTR_RI_BIT_POS  0
#define UART_INTR_CTS_BIT_POS 1
#define UART_INTR_DCD_BIT_POS 2
#define UART_INTR_DSR_BIT_POS 3
#define UART_INTR_RX_BIT_POS  4
#define UART_INTR_TX_BIT_POS  5
#define UART_INTR_RT_BIT_POS  6
#define UART_INTR_FE_BIT_POS  7
#define UART_INTR_PE_BIT_POS  8
#define UART_INTR_BE_BIT_POS  9
#define UART_INTR_OE_BIT_POS  10

/* Bit positions of UART Flag Register (0x018) */
#define UART_FR_CTS_BIT_POS  0
#define UART_FR_DSR_BIT_POS  1
#define UART_FR_DCD_BIT_POS  2
#define UART_FR_BUSY_BIT_POS 3
#define UART_FR_RXFE_BIT_POS 4
#define UART_FR_TXFF_BIT_POS 5
#define UART_FR_RXFF_BIT_POS 6
#define UART_FR_TXFE_BIT_POS 7
#define UART_FR_RI_BIT_POS   8

#define MCLK_FREQ_VAL   62500000U
#define CLR_UART_INT    0x07ffU
#ifdef TC9562
#define UART_BAUDRATE   3000000U
#else
#define UART_BAUDRATE   115200U
#endif
#define ARM_USART_DRV_VERSION       ARM_DRIVER_VERSION_MAJOR_MINOR(1, 3)  /* driver version */

#define TX_FIFO 32

#ifndef UART1DEBUG
#define UART_DEGUG 0
#else
#define UART_DEGUG 1
#endif

typedef struct _UART_RESOURCE_ {
  uint32_t txe:1;
  uint32_t rxe:1;
  uint8_t *rx_buf;
  const uint8_t *tx_buf;
  uint32_t rx_num;
  uint32_t rx_cnt;
  uint32_t tx_num;
  uint32_t tx_cnt;
} UART_RESOURCE;

typedef struct _UART_CONFIG_ {
  uint32_t UART_id:1;
  uint32_t baudrate;
  uint32_t txen:1;
  uint32_t rxen:1;
  uint32_t fen:1;
  uint32_t rxiflsel:3;
  uint32_t txiflsel:3;
  uint32_t oeim:1;
  uint32_t beim:1;
  uint32_t peim:1;
  uint32_t feim:1;
  uint32_t rtim:1;
  uint32_t txim:1;
  uint32_t rxim:1;
} UART_CONFIG;

/*Function declaration*/
int32_t  TC9562_Ser_Printf (char *format, ...);
int32_t tc9562_UART_InterSend (uint8_t UART_id, const uint8_t *data, uint32_t length);
int32_t tc9562_UART_InterReceive (uint8_t UART_id, uint8_t *data, uint32_t length);
int32_t tc9562_UART_Send (const void *data, uint32_t num, uint8_t UART_id);
int32_t tc9562_UART_Receive (void *data, uint32_t num, uint8_t UART_id);
int32_t tc9562_UART_Control (uint32_t UART_id, uint32_t control, uint32_t arg );
ARM_USART_STATUS tc9562_UART_GetStatus  (uint8_t UART_id);
int32_t tc9562_UART_Initialize(ARM_USART_SignalEvent_t cb_event, UART_CONFIG uart);
int32_t tc9562_UART_Uninitialize (uint32_t UART_id);
uint32_t tc9562_UART_GetTxCount (uint32_t UART_id);
uint32_t tc9562_UART_GetRxCount (uint32_t UART_id);


ARM_DRIVER_VERSION USARTX_GetVersion (void);
ARM_USART_CAPABILITIES USART_GetCapabilities (int8_t UART_id);
ARM_USART_CAPABILITIES USART0_GetCapabilities (void);
int32_t USART0_Initialize (ARM_USART_SignalEvent_t cb_event);
int32_t USART0_Uninitialize (void);
int32_t USART0_Send (const void *data, uint32_t num);
int32_t USART0_Receive (void *data, uint32_t num);
int32_t USART0_Control (uint32_t control, uint32_t arg);
ARM_USART_STATUS USART0_GetStatus (void);
uint32_t USART0_GetTxCount (void);
uint32_t USART0_GetRxCount (void);


ARM_USART_CAPABILITIES USART1_GetCapabilities (void);
int32_t USART1_Initialize (ARM_USART_SignalEvent_t cb_event);
int32_t USART1_Uninitialize (void);
int32_t USART1_Send (const void *data, uint32_t num);
int32_t USART1_Receive (void *data, uint32_t num);
int32_t USART1_Control (uint32_t control, uint32_t arg);
ARM_USART_STATUS USART1_GetStatus (void);
int32_t USARTX_PowerControl (ARM_POWER_STATE state);
int32_t USARTX_Transfer (const void *data_out, void *data_in, uint32_t num);
int32_t USARTX_SetModemControl (ARM_USART_MODEM_CONTROL control);
ARM_USART_MODEM_STATUS USARTX_GetModemStatus (void);
uint32_t USART1_GetTxCount (void);
uint32_t USART1_GetRxCount (void);

/*Added for QAC IRQ Handler declaration */
void UART0_IRQHandler (void);
void UART1_IRQHandler (void);

extern ARM_DRIVER_USART Driver_USART1;
extern ARM_DRIVER_USART Driver_USART0;

#ifdef __cplusplus
}
#endif
#endif
