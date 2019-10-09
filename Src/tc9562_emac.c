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
 * Filename      : tc9562_emac.c
 * Programmer(s) : 
 *                 
 *********************************************************************************************************
 */

#include "common.h"

/*====================================================================
MACRO DEFINITION
==================================================================== */
#define DMA_CH0_Tx_ST                   0x01U
#define MAC_DBG_IDLE_STATE_MASK         0x70000U
#define MTL_TX_QUEUES                   0x6U
#define MTL_RX_QUEUES                   0x4
#define MTL_Tx_QSTS_MASK                0x10
#define TC9562_EMAC_MAC_TE_RE_MASK      0x03U
#define MTL_QREG_OFF                    0x40
#define MTL_Rx_QPACKET_MASK             0x3FFF0000U
#define MAC_PMT_MPKT_RPKT_EN            0x06U
#define MAC_ARP_EN_MASK                 0x200U
#define MAC_CONFIG_ARP_EN               0x80000000U
#define MAC_Rx_EN                       0x01U
#define MAC_PMT_INT_EN                  0x10U
#define MAC_PMT_PWRDWN                  0x01U
#define TC9562_SLEEPCNTR_MASK           0x3U
#define MAC_PMT_MAGIC_PKT_RX_MASK       0x20U
#define MAC_PMT_PWRDWN_MAGIC_PKT_MASK   0x07U

/**
 * @brief Perform Power Down Sequence after receiving PME Turn Off message
 *
 * @param  None
 * @return None
 */
void TC9562_emac_power_dwn_seq(void)
{
  uint32_t data, i;

  /**
    * 1. Disable the Transmit DMA by clearing the ST bit of DMA_CH0_Tx_Control
   */
  data = Hw_Reg_Read32(TC9562_EMAC_REG_BASE, TC9562_EMAC_DMA0_CH0_Tx_OFFS);
  data &= ~(DMA_CH0_Tx_ST);
  Hw_Reg_Write32(TC9562_EMAC_REG_BASE, TC9562_EMAC_DMA0_CH0_Tx_OFFS, data);
  
  /**
   * 2. Wait for any previous frame transmissions to complete
   */
  /* check for idle state */
  data = Hw_Reg_Read32(TC9562_EMAC_REG_BASE, TC9562_EMAC_MAC_DEBUG_OFFS);
  while ( (data & MAC_DBG_IDLE_STATE_MASK) != TC9562_ZERO) 
  {
    data = Hw_Reg_Read32(TC9562_EMAC_REG_BASE, TC9562_EMAC_MAC_DEBUG_OFFS);
  }        
  
  /* Check MTL Tx Queue Not Empty Status */
  for (i = 0; i <= MTL_TX_QUEUES; i++) 
  {
    do {
        data = Hw_Reg_Read32(TC9562_EMAC_REG_BASE, TC9562_EMAC_MTL_TxQ0_OFFS + (i * MTL_QREG_OFF));
    } while ( (data & MTL_Tx_QSTS_MASK) != TC9562_ZERO);
  }

  /**
   * 3. Disable the MAC transmitter and MAC receiver by clearing 
   * Bit 1 (TE) and Bit 0 (RE) in MAC_Configuration register
   */
  data = Hw_Reg_Read32(TC9562_EMAC_REG_BASE, NTC_EMAC_MAC_CONFIG_OFFS);
  data &= ~(TC9562_EMAC_MAC_TE_RE_MASK);
  Hw_Reg_Write32(TC9562_EMAC_REG_BASE, NTC_EMAC_MAC_CONFIG_OFFS, data);
    
  /**
   * 4. Wait till the Receive DMA empties all frames from the Rx FIFO
   */
  for (i = 0; i <= MTL_RX_QUEUES; i++) 
  {
    do {
        data = Hw_Reg_Read32(TC9562_EMAC_REG_BASE, TC9562_EMAC_MTL_RxQ0_OFFS + (i * MTL_QREG_OFF));
    } while ( (data & MTL_Rx_QPACKET_MASK) != TC9562_ZERO);  
  }
  
  /**
   * 5. Configure the magic packet (bit 2)/ remote wakeup packet (bit 3)
   * detection in the MAC_PMT_Control_Status register (offset 0x00C0).
   */
  data = Hw_Reg_Read32(TC9562_EMAC_REG_BASE, TC9562_EMAC_MAC_PMT_Control_Sts);
  data |= MAC_PMT_MPKT_RPKT_EN;
  Hw_Reg_Write32(TC9562_EMAC_REG_BASE, TC9562_EMAC_MAC_PMT_Control_Sts, data);
  
  /**
   * 6. If DWC_EQOS_ARP_EN is set to enable ARP Offload during Power Down,
   * set Bit 31 (ARPEN) in the MAC Configuration Register
   */
  data = Hw_Reg_Read32(TC9562_EMAC_REG_BASE, TC9562_EMAC_MAC_HW_Feature0_OFFS);
  if ( (data & MAC_ARP_EN_MASK) != TC9562_ZERO) 
  {
    data = Hw_Reg_Read32(TC9562_EMAC_REG_BASE, NTC_EMAC_MAC_CONFIG_OFFS);
    data |= MAC_CONFIG_ARP_EN;
    Hw_Reg_Write32(TC9562_EMAC_REG_BASE, NTC_EMAC_MAC_CONFIG_OFFS, data);
  }
  
  /**
   * 7.1 Enable the MAC Receiver by setting Bit 0 (RE)
   */
  data = Hw_Reg_Read32(TC9562_EMAC_REG_BASE, NTC_EMAC_MAC_CONFIG_OFFS);
  data |= MAC_Rx_EN;
  Hw_Reg_Write32(TC9562_EMAC_REG_BASE, NTC_EMAC_MAC_CONFIG_OFFS, data);

  /* 7.2 MAC Interrupt Enable regsiter for PMT Interrupt Enable */
  data = Hw_Reg_Read32(TC9562_EMAC_REG_BASE, TC9562_EMAC_MAC_Interrupt_Enable_OFFS);
  data |= MAC_PMT_INT_EN;
  Hw_Reg_Write32(TC9562_EMAC_REG_BASE, TC9562_EMAC_MAC_Interrupt_Enable_OFFS, data);

  /**
   * 7.3 Set Bit 0 (PWRDWN) in the PMT Control and Status register to initiate
   * the power-down sequence in MAC
   */
  data = Hw_Reg_Read32(TC9562_EMAC_REG_BASE, TC9562_EMAC_MAC_PMT_Control_Sts);
  data |= MAC_PMT_PWRDWN;
  Hw_Reg_Write32(TC9562_EMAC_REG_BASE, TC9562_EMAC_MAC_PMT_Control_Sts, data);    
}
/* End of TC9562_emac_power_dwn_seq*/
