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
 * Filename      : tc9562_intc_driver.h
 *
 *********************************************************************************************************
 */


#ifndef TC9562_INTC_DRIVER_H_
#define TC9562_INTC_DRIVER_H_


#include "Driver_Common.h"
#include "tc9562_regacc.h"

#define TC9562_BASE_REG 0x40000000U
#define MCU_INTR_FLAG 0x8054U

#define PCIE_BASE_REG 		0x40020000U
#define PINT0_INIT_MSK 		0xA004U
#define PINT1_INIT_MSK 		0xA014U
#define MAC_INFORM_INT_MSK 	0xA10CU
#define SYS_FREQ 			0x6018U
#define PHY_CFGOUT 			0x6020U
#define PIPE_RESET_SH 		0x6000U
#define PINT0_INT 			0xA000U
#define BAR_RANGE_SET 		0x200CU
#define BAR_0_CFG 			0x2010U
#define BAR_1_CFG 			0x2014U
#define BAR_2_CFG 			0x2018U
#define BAR_3_CFG 			0x201CU
#define BAR_4_CFG 			0x2020U
#define BAR_5_CFG 			0x2024U
#define MAC_INFORM_INT 		0xA108U
#define PORT_INT1 			0xA010U
#define FUNC_READY 			0x2FC0U
#define VENDOR_ID 			0x2000U
#define SUBSYSTEM_ID 		0x202CU
#define MSI_CAPA_NXT_POINT 	0x2090U

#define PCLK_SYNC_RST 		0x5000U

#define INTMCUMASK2 0x8028U
#define INTEXTMASK2 0x8038U
#define INTINTXMASK2 0x8048U
#define NCLKCTRL 0x1004
#define NRSTCTRL 0x1008


#define NPCIEBOOT 0x0018

#if 1
/** Interrupt Number Definition */
typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ***************************************************************/
  NonMaskableInt_IRQn          			= -14,       /*!< 2 Non Maskable Interrupt                                    */
  HardFault_IRQn               			= -13,       /*!< 3 Cortex-M3 Hard Fault Interrupt                            */
  MemoryManagement_IRQn        			= -12,       /*!< 4 Cortex-M3 Memory Management Interrupt                     */
  BusFault_IRQn                			= -11,       /*!< 5 Cortex-M3 Bus Fault Interrupt                             */
  UsageFault_IRQn              			= -10,       /*!< 6 Cortex-M3 Usage Fault Interrupt                           */
  SVCall_IRQn                  			= -5,        /*!< 11 Cortex-M3 SV Call Interrupt                              */
  DebugMonitor_IRQn            			= -4,        /*!< 12 Cortex-M3 Debug Monitor Interrupt                        */
  PendSV_IRQn                  			= -2,        /*!< 14 Cortex-M3 Pend SV Interrupt                              */
  SysTick_IRQn                 			= -1,        /*!< 15 Cortex-M3 System Tick Interrupt                          */

/******  TC9560 Specific Interrupt Numbers *******************************************************************/
INT_SRC_NBR_INTI01			= 0,         /*!<  External interrupt input (INTI01)                      		*/
INT_SRC_NBR_INTI02			= 1,         /*!<  External interrupt input (INIT02)                      		*/
INT_SRC_NBR_INTI03			= 2,         /*!<  External interrupt input (INTI03)                      		*/
INT_SRC_NBR_EXT_INT			= 3,         /*!<  External interrupt input (INT_i)Ti                    		*/
INT_SRC_NBR_I2C_SLAVE		= 4,         /*!<  I2C slave interrupt                      					*/
INT_SRC_NBR_I2C_Master		= 5,         /*!<  I2C master interrupt                      					*/
INT_SRC_NBR_SPI_SLAVE		= 6,         /*!<  SPI slave interrupt                      					*/
INT_SRC_NBR_QSPI			= 7,         /*!<  qSPI interrupt                      							*/	
INT_SRC_NBR_MAC_LPI_EXIT	= 8,         /*!<  MAC LPI exit interrupt                      					*/
INT_SRC_NBR_MAC_POWER		= 9,         /*!<  MAC Power management interrupt                      			*/
INT_SRC_NBR_MAC_EVENTS		= 10,        /*!<  MAC event interruptgement counter...     					*/
INT_SRC_NBR_EMACTXDMA0		= 11,        /*!<  MAC interrupt from eMAC Tx DMA channel 0raffic               */
INT_SRC_NBR_EMACTXDMA1		= 12,        /*!<  MAC interrupt from eMAC Tx DMA channel 1raffic               */
INT_SRC_NBR_EMACTXDMA2		= 13,        /*!<  MAC interrupt from eMAC Tx DMA channel 2raffic               */
INT_SRC_NBR_EMACTXDMA3		= 14,        /*!<  MAC interrupt from eMAC Tx DMA channel 3raffic               */
INT_SRC_NBR_EMACTXDMA4		= 15,        /*!<  MAC interrupt from eMAC Tx DMA channel 4raffic               */
INT_SRC_NBR_EMACTXDMA5		= 16,        /*!<  MAC interrupt from eMAC Tx DMA channel 5ll the other         */
INT_SRC_NBR_EMACTXDMA6		= 17,        /*!<  MAC interrupt from eMAC Tx DMA channel 6atch M3 DA?          */
INT_SRC_NBR_EMACTXDMA7		= 18,        /*!<  MAC interrupt from eMAC Tx DMA channel 7atch host DA?        */
INT_SRC_NBR_EMACRXDMA0		= 19,        /*!<  MAC interrupt from eMAC Rx DMA channel 0ayer 2 gPTP          */
INT_SRC_NBR_EMACRXDMA1		= 20,        /*!<  MAC interrupt from eMAC Rx DMA channel 1ntagged AV Control   */
INT_SRC_NBR_EMACRXDMA2		= 21,        /*!<  MAC interrupt from eMAC Rx DMA channel 2LAN tagged           */
INT_SRC_NBR_EMACRXDMA3		= 22,        /*!<  MAC interrupt from eMAC Rx DMA channel 3                     */
INT_SRC_NBR_EMACRXDMA4		= 23,        /*!<  MAC interrupt from eMAC Rx DMA channel 4                     */
INT_SRC_NBR_EMACRXDMA5		= 24,        /*!<  MAC interrupt from eMAC Rx DMA channel 5                     */
INT_SRC_NBR_EMACRXDMA6		= 25,        /*!<  MAC interrupt from eMAC Rx DMA channel 6                     */
INT_SRC_NBR_EMACRXDMA7		= 26,        /*!<  MAC interrupt from eMAC Rx DMA channel 7                     */
INT_SRC_NBR_EMACPPO			= 27,        /*!<  MAC PPO                      								*/
INT_SRC_NBR_GDMA0			= 30,        /*!<  GDMA Channel 0 Interrupt.                     				*/
INT_SRC_NBR_GDMA1			= 31,        /*!<  GDMA Channel 1 Interrupt                      				*/
INT_SRC_NBR_GDMA2			= 32,        /*!<  GDMA Channel 2 Interrupt                      				*/
INT_SRC_NBR_GDMA3			= 33,        /*!<  GDMA Channel 3 Interrupt                      				*/
INT_SRC_NBR_GDMA4			= 34,        /*!<  GDMA Channel 4 Interrupt                      				*/
INT_SRC_NBR_GDMA5			= 35,        /*!<  GDMA Channel 5 Interrupt                     				*/
INT_SRC_NBR_GDMAGEN			= 36,        /*!<  GDMA general interrupt                      					*/
INT_SRC_NBR_SHA				= 37,        /*!<  SHA interrupt                      							*/
INT_SRC_NBR_UART0			= 38,        /*!<  UART0 interrupt                      						*/
INT_SRC_NBR_UART1			= 39,        /*!<  UART1 interrupt                      						*/
INT_SRC_NBR_MSIGEN			= 40,        /*!<  MSIGEN interrupt                      						*/
INT_SRC_NBR_PCIEC0			= 41,        /*!<  PCIe controller interrupt 0                      			*/
INT_SRC_NBR_PCIEC1			= 42,        /*!<  PCIe controller interrupt 1                      			*/
INT_SRC_NBR_PCIEC2			= 43,        /*!<  PCIe controller interrupt 2                      			*/
INT_SRC_NBR_PCIE_L12		= 44,        /*!<  PCIe L12 change interrupt                      				*/
INT_SRC_NBR_MCU				= 45,        /*!<  MCU Flag interrupt                      						*/
INT_SRC_NBR_WATCHDOG		= 48,        /*!<  Watchdog-timer interrupt                      				*/
INT_SRC_NBR_TDM0			= 49,        /*!<  TDM stream 0 input error (overflow/underflow)                */
INT_SRC_NBR_TDM1			= 50,        /*!<  TDM stream 1 input error (overflow/underflow)                */
INT_SRC_NBR_TDM2			= 51,        /*!<  TDM stream 2 input error (overflow/underflow)                */
INT_SRC_NBR_TDM3			= 52,        /*!<  TDM stream 3 input error (overflow/underflow)        		*/
INT_SRC_NBR_TDMOUT			= 53,        /*!<  TDM output error (overflow/underflow)             			*/
} IRQn_Type;
#endif

void pcie_hw_sequencier (void);
void pcie_reset_de_assertion (void);
void pcie_reset_assertion (void);
void pcie_init_config (void) ;
void pcie_hw_sequencier (void) ;

/* Configuration of the Cortex-M3 Processor and Core Peripherals */
#define __CM3_REV               0x0200  /* Cortex-M3 Core Revision */
#define __MPU_PRESENT           0       /* MPU present or not      */
#define __NVIC_PRIO_BITS        3       /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig  0       /* Set to 1 if different SysTick Config is used */

#include <core_cm3.h>                   /* Cortex-M3 processor and core peripherals */


#endif
