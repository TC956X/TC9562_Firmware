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
 * Filename      : tc9562_pcie.c
 *
 *********************************************************************************************************
 */

/**********************************************************************
*                        INCLUDE FILES
**********************************************************************/
#include "common.h"
#include "tc9562_uart_driver.h"

/*====================================================================
MACRO DEFINITION
==================================================================== */
/* PINT0 Interrupts */
#define PINT0_ST_LINK_STATE_DOWN            0x00000E00U

/* PINT1 interrupts */
#define PINT1_ST_RC_INTERNAL_INT_RES        0x00000008U
#define PINT1_ST_VC0_INTG_ERR_RES           0x00000100U

#if ( DEF_ENABLED == TC9562_M3POR_4PCIE_ENABLE )
#define TC9560_SYS_FREQ                     0x000000bcU
#define ML_PA_HOTRESET_VAL                  0x00000004U
#define FUNC_READY_VAL                      0x00000001U
#define PIPE_RESET_SH_VAL                   0x00000001U
#define BAR_RANGE_SET_VAL                   0x6C6D6B1CU
#define REV_ID_VAL                          0x01080200U
#define BAR_0_CFG_VAL                       0x40000004U
#define BAR_2_CFG_VAL                       0x0000000CU
#define BAR_4_CFG_VAL                       0x1000000CU
#define VENDOR_ID_VAL                       0x021A1179U
#define SUB_SYS_ID_VAL                      0x00011179U
#define RANGE00_UP_OFFSET_VAL               0x00000000U
#define RANGE00_ENABLE_VAL                  0x60000001U
#define RANGE00_WIDTH_VAL                   0x20000023U
#define RANGE00_UP_RPLC_VAL                 0x00000002U
#define GEN1_LINK_SPEED                     0x00000002U
#endif


#define TC9560_GPIO_ISR                     0x804CU
#define TC9560_PCIE_REG                     0xA200U
#define VC0_INTEGRITY_ERR                   0xA200U
#define PME_TO_ACK_TRGR                     0x00020000U
#define MAC_FD_RE_EN                        0x2001U
#define MAC_SLEEP_CNT_EN                    0x3U

/* PCIe callback functions */
typedef void (*PCIE_FUNC_CB)(void);
PCIE_FUNC_CB cb_pcie_enter_low_states = NULL;
PCIE_FUNC_CB cb_pcie_turn_off = NULL;


/*====================================================================
FUNCTION DEFINITION
==================================================================== */

/*
*   Function    :   PCIEC1_IRQHandler( )
*   Purpose     :   Handle the PME Turn Off event which is part of PCIe Controller 1 Interrupt
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None 
*/
void PCIEC1_IRQHandler( void )
{
  uint32_t data, status, mask;
  uint32_t power_inform;
  uint32_t ret = TC9562_ZERO;
    
  status = Hw_Reg_Read32(TC9562_PCIE_REG_BASE, PINT1_INT_REG);
  mask = data = Hw_Reg_Read32(TC9562_PCIE_REG_BASE, PINT1_INT_MSK_REG);
  data = status & (~mask);

  if ( (data & PINT1_ST_POWER_INFORM_INT) != TC9562_ZERO )/* POWER_INFORM_INT */
  {
    /* Power events */
    power_inform = Hw_Reg_Read32(TC9562_PCIE_REG_BASE, POWER_INFORM_INT_REG);
    if ( (power_inform & PME_TURN_OFF_RCV_ST) != TC9562_ZERO )
    {
      /* Here initiate WAKE ON LAN if going to Low power states. */
      if (cb_pcie_enter_low_states != NULL)
      {
        cb_pcie_enter_low_states();
      }

      /* Received PME_TURN_OFF
        Assert PM_CTRL2.PME_TO_ACK_TRG to send PME_TO_ACK */
      Hw_Reg_Write32(TC9562_PCIE_REG_BASE, PM_CTRL2_REG, PME_TO_ACK_TRGR);

      if (cb_pcie_turn_off != NULL)
      {
        /* handle things before shutdown */
        cb_pcie_turn_off();
      } 

      /* Clear POWER_INFORM_INT.PME_TURN_OFF_RCV_ST */
      Hw_Reg_Write32(TC9562_PCIE_REG_BASE, POWER_INFORM_INT_REG, power_inform);

      /* Clear PINT1 */
      Hw_Reg_Write32(TC9562_PCIE_REG_BASE, PINT1_INT_REG, status);

      /* Enable Full Duplex and Receiver Enable */
      data = Hw_Reg_Read32(TC9562_EMAC_REG_BASE, NTC_EMAC_MAC_CONFIG_OFFS);
      data |= MAC_FD_RE_EN;
      Hw_Reg_Write32(TC9562_EMAC_REG_BASE, NTC_EMAC_MAC_CONFIG_OFFS, data);

      /* Put in sleep mode */
      data = Hw_Reg_Read32(TC9562_REG_BASE, TC9562_NSLEEPCTR_OFFS);
      data |= MAC_SLEEP_CNT_EN;
      Hw_Reg_Write32(TC9562_REG_BASE, TC9562_NSLEEPCTR_OFFS, data);

      ret = TC9562_ONE; /*  PCIe will be off */
    }
    if( ret == TC9562_ZERO )
    {
      /* Clear POWER_INFORM_INT */
      Hw_Reg_Write32(TC9562_PCIE_REG_BASE, POWER_INFORM_INT_REG, power_inform);
    }
  }
        
  if( ret == TC9562_ZERO )
  {
    /* Clear PINT1 */
    Hw_Reg_Write32(TC9562_PCIE_REG_BASE, PINT1_INT_REG, status);
  }
}

/* End of PCIEC1_IRQHandler */
#ifdef TC9562_SWITCH_HW_TO_SW_SEQ
/*
*   Function    :   TC9562_PCIE_Reset_Assert( )
*   Purpose     :   Update the system after PCIe reset assertion
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None 
*/
void TC9562_PCIE_Reset_Assert( void )
{
  uint32_t uiData;
  
  /* Mask interrupts for PCIe Controller and MSI Generator */
  uiData = Hw_Reg_Read32(TC9562_INTC_REG_BASE, INTMCUMASK2_OFFS);
  uiData = uiData | TC9562_INTMCUMASK2_PCIE_MSI;
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTMCUMASK2_OFFS, uiData);
  
  uiData = Hw_Reg_Read32(TC9562_INTC_REG_BASE, INTEXTMASK2_OFFS);
  uiData = uiData | TC9562_INTEXTMASK2_PCIE_MSI;
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTEXTMASK2_OFFS, uiData);
  
  uiData = Hw_Reg_Read32(TC9562_INTC_REG_BASE, INTINTXMASK2_OFFS);
  uiData = uiData | TC9562_INTINTXMASK2_PCIE_MSI;
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTINTXMASK2_OFFS, uiData);
  
  /* Reset assert PCIE and MSI Generator */
  uiData = Hw_Reg_Read32 ( TC9562_REG_BASE, TC9562_NRSTCTRL_OFFS ) ;
  uiData |= (1 << TC9562_NRSTCTRL_PCIRST_LPOS);
  Hw_Reg_Write32(TC9562_REG_BASE, TC9562_NRSTCTRL_OFFS, uiData);

  uiData = Hw_Reg_Read32 ( TC9562_REG_BASE, TC9562_NRSTCTRL_OFFS ) ;
  uiData |= (1 << TC9562_NRSTCTRL_MSIGENRST_LPOS);
  Hw_Reg_Write32(TC9562_REG_BASE, TC9562_NRSTCTRL_OFFS, uiData);
}
/*End of TC9562_PCIE_Reset_Assert */

/*
*   Function    :   TC9562_PCIE_Ctrl_Init( )
*   Purpose     :   Procedure to initialize PCIe registers when switching to SW sequencer
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None 
*/
void TC9562_PCIE_Ctrl_Init( void )
{
  volatile uint32_t uiData;
  
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, PINT0_INT_MSK_REG, PINT0_INT_MSK_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, PINT1_INT_MSK_REG, PINT_INTI_MSK_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, MAC_INFORM_INT_MSK_REG, MAC_INFORM_INT_MSK_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, SYS_FREQ_REG, SYS_FREQ_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, PHY_CFGOUT_REG, PHY_CFGOUT_VAL);
  
  uiData = Hw_Reg_Read32(TC9562_PCIE_REG_BASE, PINT0_INT_REG);
  while((uiData & PINT0_ST_PCLK_VALKD_CHG) != PINT0_ST_PCLK_VALKD_CHG)
  {
    uiData = Hw_Reg_Read32(TC9562_PCIE_REG_BASE, PINT0_INT_REG);
  }
  
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, PIPE_RESET_SH_REG, PIPE_RESET_SH_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, PINT0_INT_REG, PINT0_INT_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, BAR_RANGE_SET_REG, BAR_RANGE_SET_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, BAR_0_CFG_REG, BAR_0_CFG_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, BAR_1_CFG_REG, BAR_1_CFG_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, BAR_2_CFG_REG, BAR_2_CFG_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, BAR_3_CFG_REG, BAR_3_CFG_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, BAR_4_CFG_REG, BAR_4_CFG_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, BAR_5_CFG_REG, BAR_5_CFG_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, MAC_INFORM_INT_REG, MAC_INFORM_INT_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, PINT1_INT_REG, PORT_INT1_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, FUNC_READY_REG, FUNC_READY_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, VENDOR_ID_REG, VENDOR_ID_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, SUBSYSTEM_ID_REG, SUBSYSTEM_ID_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, MSI_CAPA_NXT_POINT_REG, MSI_CAPA_NXT_POINT_VAL);
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, PCLK_SYNC_RST, PCLK_SYNC_VAL);
}
/* End of TC9562_PCIE_Ctrl_Init */

/*
*   Function    :   TC9562_PCIE_Reset_Deassert( )
*   Purpose     :   Update the system after PCIe Reset de-assertion
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None 
*/
void TC9562_PCIE_Reset_Deassert( void )
{
  uint32_t uiData;
  
  uiData = Hw_Reg_Read32 ( TC9562_REG_BASE, TC9562_NRSTCTRL_OFFS ) ;
  uiData &= (~(1 << TC9562_NRSTCTRL_MSIGENRST_LPOS));
  Hw_Reg_Write32(TC9562_REG_BASE, TC9562_NRSTCTRL_OFFS, uiData);
  
  /* Unmask interrupts for PCIe Controller and MSI Generator */
  uiData = Hw_Reg_Read32(TC9562_INTC_REG_BASE, INTMCUMASK2_OFFS);
  uiData = uiData & (~TC9562_INTMCUMASK2_PCIE_MSI);
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTMCUMASK2_OFFS, uiData);
  
  uiData = Hw_Reg_Read32(TC9562_INTC_REG_BASE, INTEXTMASK2_OFFS);
  uiData = uiData & (~TC9562_INTEXTMASK2_PCIE_MSI);
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTEXTMASK2_OFFS, uiData);
  
  uiData = Hw_Reg_Read32(TC9562_INTC_REG_BASE, INTINTXMASK2_OFFS);
  uiData = uiData & (~TC9562_INTINTXMASK2_PCIE_MSI);
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTINTXMASK2_OFFS, uiData);
}
/* End of TC9562_PCIE_Reset_Deassert */

/*
*   Function    :   TC9562_PCIE_Init_SWSeq( )
*   Purpose     :   Procedure to switch from HW sequencer to SW Sequencer
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None 
*/
void TC9562_PCIE_Init_SWSeq( void )
{
  uint32_t uiData;
  uint32_t uiPCIe_rst, uiPCIe_bt_st;
  
  /* Clear PCIe reset interrupts */
  uiData = (1 << TC9562_MCUFLAG_PR_RE) | (1 << TC9562_MCUFLAG_PR_FE);
  Hw_Reg_Write32(TC9562_INTC_REG_BASE, MCUFLG_OFFS, uiData);
  
  /* Read PCIe reset status */
  uiPCIe_rst = Hw_Reg_Read32(TC9562_INTC_REG_BASE, MCUFLG_OFFS);
  
  if((uiPCIe_rst & (1 << TC9562_MCUFLAG_PR_RST_MON)) == 0) 
  {
    /* Clear PCIe-reset falling edge interrupt */
    uiData = (1 << TC9562_MCUFLAG_PR_FE);
    Hw_Reg_Write32(TC9562_INTC_REG_BASE, MCUFLG_OFFS, uiData);
    
    TC9562_PCIE_Reset_Assert();
  }
  else
  {
    /* Clear PCIe-reset rising  edge interrupt */
    uiData = (1 << TC9562_MCUFLAG_PR_RE);
    Hw_Reg_Write32(TC9562_INTC_REG_BASE, MCUFLG_OFFS, uiData);
    
    uiPCIe_bt_st = Hw_Reg_Read32 ( TC9562_REG_BASE, TC9562_NPCIEBOOT_OFFS ) ;
    uiPCIe_bt_st &= TC9562_NPCIEBOOT_PCIE_HW_BT_ST_MASK;
    uiPCIe_bt_st = uiPCIe_bt_st >> TC9562_NPCIEBOOT_PCIE_HW_BT_ST_LPOS;
    
    if(uiPCIe_bt_st == 0x0) 
    {
      TC9562_PCIE_Ctrl_Init();
    }

    TC9562_PCIE_Reset_Deassert();
  }
  
  NVIC_EnableIRQ(INT_SRC_NBR_MCU_FLAG);
}
/* End of TC9562_PCIE_Init_SWSeq */

/*
*   Function    :   MCU_IRQHandler( )
*   Purpose     :   Interrupt handler for MCU_FLAG interrupts
*   Inputs      :   None
*   Outputs     :   None
*   Return Value:   None
*   Limitations :   None 
*/
void MCU_IRQHandler(void)
{
  uint32_t uiData;
  uint32_t uiIntRst, uiItr = 0;
  
  uiData = Hw_Reg_Read32(TC9562_INTC_REG_BASE, MCUFLG_OFFS);
  if((uiData & (1 << TC9562_MCUFLAG_PR_FE)) == TC9562_MCUFLAG_PR_FE_VAL)
  {
    uiIntRst = (1 << TC9562_MCUFLAG_PR_FE);
    Hw_Reg_Write32(TC9562_INTC_REG_BASE, MCUFLG_OFFS, uiIntRst);
    TC9562_PCIE_Reset_Assert();
  }
  else if((uiData & (1 << TC9562_MCUFLAG_PR_RE)) == TC9562_MCUFLAG_PR_RE_VAL)
  {
    uiIntRst = (1 << TC9562_MCUFLAG_PR_RE);
    Hw_Reg_Write32(TC9562_INTC_REG_BASE, MCUFLG_OFFS, uiIntRst);
    
    uiData = Hw_Reg_Read32 ( TC9562_REG_BASE, TC9562_NRSTCTRL_OFFS ) ;
    uiData &= (~(1 << TC9562_NRSTCTRL_PCIRST_LPOS));
    Hw_Reg_Write32(TC9562_REG_BASE, TC9562_NRSTCTRL_OFFS, uiData);

    while(uiItr < 10)
    {
      /* Dummy read */
      Hw_Reg_Read32 ( TC9562_REG_BASE, TC9562_NRSTCTRL_OFFS ) ;
      uiItr++;
    }
    
    TC9562_PCIE_Ctrl_Init();
    TC9562_PCIE_Reset_Deassert();
  }
  else
  {
    Hw_Reg_Write32(TC9562_INTC_REG_BASE, MCUFLG_OFFS, uiData & 0x7FFF);
  }
}
/* End of MCU_IRQHandler */

#endif /* End of TC9562_SWITCH_HW_TO_SW_SEQ */
