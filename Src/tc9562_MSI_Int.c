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
 * =========================================================================== */

/*! History:
 *  30 Sep 2019 : Base lined
 *  VERSION     : 1.0.0
 */

/*
 *********************************************************************************************************
 *
 * Target        : TC9562
 * Filename      : tc9562_MSI_Int.c
 *
 *********************************************************************************************************
 */

/*=====================================================================
INCLUDE FILES
=====================================================================*/
#include "common.h"

/*=====================================================================
FUNCTION DEFINITION
======================================================================*/

/**
*   Function    :   TC9562_MSIGEN_ISR_Handler( void )
*   Purpose     :   Interrupt handler for handling MSI message
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
**/
void TC9562_MSIGEN_ISR_Handler( void )
{

#ifdef TC9562_M3_MASK_INT_IN_MSIGEN
  uint32_t uiMask ;       /* reading register values */

/********************************************************
*  Mask all interrupts except reserved and unused bits *
********************************************************/

  uiMask = Hw_Reg_Read32( TC9562_INTC_REG_BASE, INTMCUMASK1_OFFS ) ;
  uiMask |= INTMCUMASK1_MSI ;
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTMCUMASK1_OFFS, uiMask ) ;
  
  uiMask = Hw_Reg_Read32( TC9562_INTC_REG_BASE, INTMCUMASK2_OFFS ) ;
  uiMask |= INTMCUMASK2_MSI ;
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTMCUMASK2_OFFS, uiMask ) ;
#endif
    
    /* MSI GEN */
#ifdef TC9562_SW_MSI_TEST	
  Hw_Reg_Write32( TC9562_MSI_REG_BASE, 0x50,TC9562_ONE ) ;	
#else
  Hw_Reg_Write32( TC9562_PCIE_REG_BASE, MSI_SEND_TRIGGER_OFFS,TC9562_ZERO ) ;
#endif


}
/* End of TC9562_MSIGEN_ISR_Handler */

/* ********************************************************************
*               eMAC
* ****************************************************************** */

/*
*   Function    :   MAC_LPI_EXIT_IRQHandle( void )
*   Purpose     :   Interrupt handler for Handling MAC LPI interrupt
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
*/
void MAC_LPI_EXIT_IRQHandler ( void )
{
  TC9562_MSIGEN_ISR_Handler( ) ;
  *( uint32_t *)( TC9562_M3_DBG_CNT_START + ( TC9562_THIRTEEN * TC9562_FOUR ) ) += TC9562_ONE ;
}
/* End of MAC_LPI_EXIT_IRQHandler */

/*
*   Function     :   MAC_EVENTS_IRQHandler( void )
*   Purpose      :   Interrupt handler for handling MAC EVENTS interrupt
*   Inputs       :   None
*   Outputs      :   None
*   Return  Value:   None
*   Limitations  :   None
*/
void MAC_EVENTS_IRQHandler ( void )
{
  TC9562_MSIGEN_ISR_Handler( ) ;
  *( uint32_t *)( TC9562_M3_DBG_CNT_START + ( TC9562_THIRTEEN * TC9562_FOUR ) ) += TC9562_ONE ;
}
/* End of MAC_EVENTS_IRQHandler */

/* ********************************************************************
*               DMATX CH
* ****************************************************************** */

/*
*   Function    :   EMACTXDMA0_IRQHandler( void )
*   Purpose     :   Interrupt handler for handling EMAC Transmitting DMA 
*                   Channel 0 interrupts.
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
*/
void EMACTXDMA0_IRQHandler ( void )
{
  TC9562_MSIGEN_ISR_Handler( ) ;
  *( uint32_t*)( TC9562_M3_DBG_CNT_START + ( TC9562_ZERO * TC9562_FOUR ) ) += TC9562_ONE ;
}
/* End of EMACTXDMA0_IRQHandler */

/*
*   Function    :   EMACTXDMA1_IRQHandler( void )
*   Purpose     :   Interrupt handler for handling EMAC Transmitting 
*                   DMA Channel 1 interrupts.
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
*/
void EMACTXDMA1_IRQHandler ( void )
{
  TC9562_MSIGEN_ISR_Handler( ) ;
  *( uint32_t*)( TC9562_M3_DBG_CNT_START + ( TC9562_ONE * TC9562_FOUR ) ) += TC9562_ONE ;
}
/* End of EMACTXDMA1_IRQHandler */

/*
*   Function    :   EMACTXDMA2_IRQHandler( void )
*   Purpose     :   Interrupt handler for handling EMAC Transmitting 
*                   DMA Channel 2 interrupts.
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
*/
void EMACTXDMA2_IRQHandler ( void )
{
  TC9562_MSIGEN_ISR_Handler( ) ;
  *( uint32_t*)( TC9562_M3_DBG_CNT_START + ( TC9562_TWO * TC9562_FOUR ) ) += TC9562_ONE ;
}
/* End of EMACTXDMA2_IRQHandler */

/*
*   Function    :   EMACTXDMA3_IRQHandler( void )
*   Purpose     :   Interrupt handler for handling EMAC Transmitting 
                   DMA Channel 3 interrupts.
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
*/
void EMACTXDMA3_IRQHandler ( void )
{
  TC9562_MSIGEN_ISR_Handler( ) ;
  *( uint32_t*)( TC9562_M3_DBG_CNT_START + ( TC9562_THREE * TC9562_FOUR ) ) += TC9562_ONE ;
}
/* End of EMACTXDMA3_IRQHandler */

/*
*   Function    :   EMACTXDMA4_IRQHandler( void )
*   Purpose     :   Interrupt handler for handling EMAC Transmitting
                    DMA Channel 4 interrupts.
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
*/
void EMACTXDMA4_IRQHandler ( void )
{
  TC9562_MSIGEN_ISR_Handler( ) ;
  *( uint32_t*)( TC9562_M3_DBG_CNT_START + ( TC9562_FOUR * TC9562_FOUR ) ) += TC9562_ONE ;
}
/* End of EMACTXDMA4_IRQHandler */

/*
*   Function    :   EMACTXDMA5_IRQHandler( void )
*   Purpose     :   Interrupt handler for handling EMAC Transmitting
                    DMA Channel 5 interrupts.
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
*/
void EMACTXDMA5_IRQHandler ( void )
{
  TC9562_MSIGEN_ISR_Handler( ) ;
  *( uint32_t*)( TC9562_M3_DBG_CNT_START + ( TC9562_FIVE * TC9562_FOUR ) ) += TC9562_ONE ;
}
/* End of EMACTXDMA5_IRQHandler */

/* ********************************************************************
*               DMARX CH
* ******************************************************************* */

/*
*   Function    :   EMACRXDMA0_IRQHandler( void )
*   Purpose     :   Interrupt handler for handling EMAC Receiving 
*                   DMA Channel 0 interrupts.
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
*/
void EMACRXDMA0_IRQHandler ( void )
{
  TC9562_MSIGEN_ISR_Handler( ) ;
  *( uint32_t*)( TC9562_M3_DBG_CNT_START + ( TC9562_SIX * TC9562_FOUR ) ) += TC9562_ONE ;
}
/* End of EMACRXDMA0_IRQHandler */

/*
*   Function    :   EMACRXDMA1_IRQHandler( void )
*   Purpose     :   Interrupt handler for handling EMAC Receiving 
                    DMA Channel 1 interrupts.
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
*/
void EMACRXDMA1_IRQHandler ( void )
{
  TC9562_MSIGEN_ISR_Handler( ) ;
  *( uint32_t*)( TC9562_M3_DBG_CNT_START + ( TC9562_SEVEN * TC9562_FOUR ) ) += TC9562_ONE ;
}
/* End of EMACRXDMA1_IRQHandler */

/*
*   Function    :   EMACRXDMA2_IRQHandler( void )
*   Purpose     :   Interrupt handler for handling EMAC Receiving 
*                   DMA Channel 2 interrupts.
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
*/
void EMACRXDMA2_IRQHandler ( void )
{
  TC9562_MSIGEN_ISR_Handler( ) ;
  *( uint32_t*)( TC9562_M3_DBG_CNT_START + ( TC9562_EIGHT * TC9562_FOUR ) ) += TC9562_ONE ;
}
/* End of EMACRXDMA2_IRQHandler */

/*
*   Function    :   EMACRXDMA3_IRQHandler( void )
*   Purpose     :   Interrupt handler for handling EMAC Receiving 
*                   DMA Channel 3 interrupts.
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
*/
void EMACRXDMA3_IRQHandler ( void )
{
  TC9562_MSIGEN_ISR_Handler( ) ;
  *( uint32_t*)( TC9562_M3_DBG_CNT_START +   ( TC9562_NINE * TC9562_FOUR ) ) += TC9562_ONE ;
}
/* End of EMACRXDMA3_IRQHandler */

/*
*   Function    :   EMACRXDMA4_IRQHandler( void )
*   Purpose     :   Interrupt handler for handling EMAC Receiving 
*                   DMA Channel 4 interrupts.
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
*/
void EMACRXDMA4_IRQHandler ( void )
{
  TC9562_MSIGEN_ISR_Handler( ) ;
  *( uint32_t*)( TC9562_M3_DBG_CNT_START + ( TC9562_TEN * TC9562_FOUR ) ) += TC9562_ONE ;
}
/* End of EMACRXDMA4_IRQHandler */

/*
*   Function    :   WDT_IRQHandler( void )
*   Purpose     :   Interrupt handler for handling WatchDog Timer interrupts
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None
*/
void WDT_IRQHandler ( void )
{
  uint32_t uiData ;           /* reading register values */

  *( uint32_t*)( TC9562_M3_DBG_CNT_START + ( TC9562_TWELVE * TC9562_FOUR ) ) += TC9562_ONE ;
  uiData = Hw_Reg_Read32( TC9562_INTC_REG_BASE, INTINTWDCTL_OFFS ) ;
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTINTWDCTL_OFFS, 
                ( uiData | ( WDCTL_WDRestart_MASK ) ) ) ;
}
/* end WDT_IRQHandler */
