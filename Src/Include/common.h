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
 
#ifndef _COMMON_H__
#define _COMMON_H__

/*
 *********************************************************************************************************
 *
 * Target        : TC9562
 * Filename      : common.h
 *
 *********************************************************************************************************
 */

/*********************************************************************
*                      INCLUDE FILES
********************************************************************* */
#include "tc9562.h"  /* Register address, Interrupt numbers, 
                          Core peripheral section */

/*=====================================================================
                    MACRO DEFINITION
==================================================================== */ 
#define DEF_DISABLED                0
#define DEF_ENABLED                 1
#define NULL                        0

/* DEF_DISABLED ==> Use hardware POR sequence */
#define TC9562_M3POR_4PCIE_ENABLE       DEF_DISABLED 
#define TC9562_DISABLE_MAC_EVENTS_INT	DEF_ENABLED 
#define TC9562_SWITCH_HW_TO_SW_SEQ

#define TC9562_M3_MASK_INT_IN_MSIGEN    1
//#define TC9562_SW_MSI_TEST            1

#define TC9562_COMMON_ONE              0x00000001U
#define TC9562_COMMON_TWO              0x00000002U
#define TC9562_COMMON_FOUR             0x00000004U

#define TC9562_MAX                     255
#define TC9562_THOUSAND                1000U
#define TRUE                        	1
#define FALSE                       	0

typedef unsigned long long uint64_t ;
typedef unsigned int uint32_t ;
typedef int int32_t ;
typedef unsigned char uint8_t ;
typedef char int08_t ;

extern volatile uint64_t guiM3Ticks ;        /* System ticks */

/*=====================================================================
FUNCTION PROTOTYPES
==================================================================== */
void SysTick_Handler( void ) ;
void SysInit( void ) ;
void TC9562_MSIGEN_ISR_Handler( void ) ;
void Hw_Reg_Write32 ( uint32_t uiAddr_base, uint32_t uiOffset,
                      uint32_t uiVal ) ;
uint32_t Hw_Reg_Read32 ( uint32_t uiAddr_base,
                         uint32_t uiOffset ) ;
extern void MAC_LPI_EXIT_IRQHandler ( void ) ;
extern void MAC_POWER_IRQHandler ( void ) ;
extern void MAC_EVENTS_IRQHandler ( void ) ;
extern void EMACTXDMA0_IRQHandler ( void ) ;
extern void EMACTXDMA1_IRQHandler ( void ) ;
extern void EMACTXDMA2_IRQHandler ( void ) ;
extern void EMACTXDMA3_IRQHandler ( void ) ;
extern void EMACTXDMA4_IRQHandler ( void ) ;
extern void EMACRXDMA0_IRQHandler ( void ) ;
extern void EMACRXDMA1_IRQHandler ( void ) ;
extern void EMACRXDMA2_IRQHandler ( void ) ;
extern void EMACRXDMA3_IRQHandler ( void ) ;
extern void EMACRXDMA4_IRQHandler ( void ) ;
extern void EMACRXDMA5_IRQHandler ( void ) ;
extern void WDT_IRQHandler ( void ) ;
extern void NVIC_EnableIRQ(IRQn_Type IRQn);
extern void NVIC_DisableIRQ(IRQn_Type IRQn);
extern void CPU_IntEn( void ) ;
/* Interrupt callbacks prototypes */
typedef void (*INT_FUNC_CB)(void);
extern INT_FUNC_CB cb_pcie_turn_off;
extern INT_FUNC_CB cb_pcie_enter_low_states;
extern void TC9562_emac_power_dwn_seq(void);
extern void TC9562_PCIE_Init_SWSeq( void );
/* pcie Reset Deassert and Assert */
void TC9562_PCIE_Reset_Deassert( void );
void TC9562_PCIE_Reset_Assert( void );
void TC9562_PCIE_Ctrl_Init( void );
#endif /* _COMMON_H__ */
