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
 * Target      : TC9562
 * Filename    : tc9562_regacc.h
 *
 *********************************************************************************************************
 */

#ifndef _TC9562_REGACC_H_
#define _TC9562_REGACC_H_

#include<stdint.h>

/* ****************************** Macro definition **************************************************/
#define TC9562_REG_BASE           0x40000000

#define  TC9562_REG_BASE_ADRS     (0x40000000U)

#define  MAC_OFFSET               (0xA000U)

#define  MAC_BASE_ADDRESS         (TC9562_REG_BASE_ADRS + MAC_OFFSET)

#define CM3_MAC_ADDR_INDEX         2U  /* Ofset of CM3 MAC Address between 1 to 15*/

#define MII_ADDR_C45              (1U <<30U )

/* Macro for register PHY address */

#define DWC_ETH_QOS_PHY_CL45_ST_REG   0x00010003U  //MRVL88Q1010 ; changes for MRVL88Q2110

#define MII_BMSR      0x01U

#define MAC_GMIIAR_RgOffAddr    ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x200U))

#define MAC_GMIIDR_RgOffAddr    ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x204U))

#define MAC_GMIIDR_RA_Wr_Mask    (uint32_t)(0xffffU)

#define MAC_GMIIDR_RA_Mask       (uint32_t)(0xffffU)

#define MAC_GMIIDR_GD_Wr_Mask    (uint32_t)(0xffff0000U)

#define MAC_GMIIDR_GD_Mask       (uint32_t)(0xffffU)

# define MAC_GMIIAR_CR_CLK_2035  (0x00000200U)

# define MAC_GMIIAR_GOC_RD       (0x0000000CU)

# define MAC_GMIIAR_C45E_EN      (0x00000002U)

# define MAC_GMIIAR_GB_EN        (0x00000001U)

# define MAC_GMIIAR_GOC_WR       (0x00000004U)

#define MAC_GMIIDR_RA_LPOS       16U

#define MAC_GMIIDR_RA_HPOS       31U

#define MAC_GMIIDR_GD_LPOS       0U

#define MAC_GMIIDR_GD_HPOS       15U

#define MAC_GMIIAR_GB_LPOS       0U

#define MAC_GMIIAR_GB_HPOS       0U

#define Y_SUCCESS                1

#define Y_FAILURE               -1

#define GET_VALUE(data, lbit, hbit)   ((data >> (lbit)) & (~((uint32_t)(~0U) << ( ((hbit) - (lbit)) + 1))))

#define SUCCESS 1
#define FAIL    0

#define hw_reg_write32(addr_base, offset, val) *((volatile unsigned int*)((addr_base) + (offset)))=(val)
#define hw_reg_read32(addr_base, offset) (*((volatile unsigned int*)((addr_base) + (offset))))


#define hw_reg_read08(addr_base, offset) (*((volatile unsigned char*)((addr_base) + (offset))))
#define hw_reg_write08(addr_base, offset, val) *((volatile unsigned char *)(addr_base + offset)) = (val & 0xFF)


#define TC9562_NFUNCEN0_OFFS 0x0008U
#define TC9562_NFUNCEN1_OFFS 0x1514U
#define TC9562_NFUNCEN2_OFFS 0x151CU
#define TC9562_NFUNCEN3_OFFS 0x1524U
#define TC9562_NFUNCEN4_OFFS 0x1528U
#define TC9562_NFUNCEN5_OFFS 0x152CU
#define TC9562_NFUNCEN6_OFFS 0x1530U
#define TC9562_NFUNCEN7_OFFS 0x153CU
#define TC9562_NMODESTS_OFFS 0x0004U


#define TC9562_ZERO          0U
#define TC9562_ONE           1U
#define TC9562_TWO           2U
#define TC9562_THREE         3U
#define TC9562_FOUR          4U
#define TC9562_FIVE          5U
#define TC9562_SIX           6U
#define TC9562_SEVEN         7U
#define TC9562_EIGHT         8U
#define TC9562_NINE          9U
#define TC9562_TEN           10U
#define TC9562_ELEVEN        11U
#define TC9562_TWELVE        12U
#define TC9562_THIRTEEN      13U
#define TC9562_FOURTEEN      14U
#define TC9562_FIFTEEN       15U
#define TC9562_SIXTEEN       16U
#define TC9562_SEVENTEEN     17U
#define TC9562_EIGHTEEN      18U
#define TC9562_NINETEEN      19U
#define TC9562_TWENTY        20U
#define TC9562_TWENTYONE     21U
#define TC9562_TWENTYTWO     22U
#define TC9562_TWENTYTHREE   23U
#define TC9562_TWENTYFOUR    24U
#define TC9562_TWENTYFIVE    25U
#define TC9562_TWENTYSIX     26U
#define TC9562_TWENTYSEVEN   27U
#define TC9562_TWENTYEIGHT   28U
#define TC9562_TWENTYNINE    29U
#define TC9562_THIRTY        30U
#define TC9562_THIRTYONE     31U


#define TC9562_MASK_0_7bits        (0x000000FFU)
#define TC9562_MASK_8_15bits       (0x0000FF00U)
#define TC9562_MASK_16_23bits      (0x00FF0000U)
#define TC9562_MASK_24_31bits      (0xFF000000U)

#define TC9562_SET_Bit0_Zero       (0xFFFFFFFEU)

#define TC9562_NCLKCTRL_OFFS       0x1004U
#define TC9562_NRSTCTRL_OFFS       0x1008U
#define TC9562_VAL_ZERO            0U
#define TC9562_CLK_CTRL_ENABLE     0x0770FAB5U
/*************************** END *******************************************************************/


__inline static void iowrite32(uint32_t val, volatile uint32_t *addr)
{
  *addr = val;
}

__inline static unsigned int ioread32(volatile uint32_t *addr)
{
  volatile uint32_t val = *addr;
  return val;
}

__inline static void mdelay(uint32_t ms)
{
  unsigned int i, cnt = 0;

  cnt = 187500*ms;  /* 187.5MHz */
  for (i = 0; i < cnt; i++) {
  ;
  }
  return;
}

__inline static void udelay(uint32_t us)
{
  unsigned int i, cnt = 0;

  cnt = 187*us;  /* 187.5MHz: Its not an accurate usec */
  for (i = 0; i < cnt; i++) {
  ;
  }
  return;
}

#define MAC_GMIIDR_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_GMIIDR_RgOffAddr);\
} while(0)

#define MAC_GMIIDR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_GMIIDR_RgOffAddr);\
} while(0)

#define MAC_GMIIAR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_GMIIAR_RgOffAddr);\
} while(0)

#define MAC_GMIIDR_RA_UdfWr(data) do{\
  uint32_t v;\
  MAC_GMIIDR_RgRd(v);\
  v = ((v & MAC_GMIIDR_RA_Wr_Mask) | ((data & MAC_GMIIDR_RA_Mask) << MAC_GMIIDR_RA_LPOS));\
  MAC_GMIIDR_RgWr(v);\
} while(0)

#define MAC_GMIIAR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_GMIIAR_RgOffAddr);\
} while(0)

#define MAC_GMIIAR_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_GMIIAR_RgOffAddr);\
} while(0)

#define MAC_GMIIDR_GD_UdfWr(data) do{\
  uint32_t v;\
  MAC_GMIIDR_RgRd(v);\
  v = ((v & MAC_GMIIDR_GD_Wr_Mask) | ((data & MAC_GMIIDR_GD_Mask) << MAC_GMIIDR_GD_LPOS));\
  MAC_GMIIDR_RgWr(v);\
} while(0)

#define MAC_Addr1_15_HR_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x300 + (0x008*(i))))

#define MAC_Addr1_15_HR_RgWr(i, data) do {\
  iowrite32((data), (volatile void *)MAC_Addr1_15_HR_RgOffAddr(i));\
} while(0)

#define MAC_Addr1_15_HR_RgRd(i, data) do {\
  (data) = ioread32((volatile void *)MAC_Addr1_15_HR_RgOffAddr(i));\
} while(0)

#define MAC_Addr1_15_HR_ADDRHI_Mask (uint32_t)(0xffff)

#define MAC_Addr1_15_HR_ADDRHI_Wr_Mask (uint32_t)(0xffff0000)

#define MAC_Addr1_15_HR_RES_Wr_Mask_19 (uint32_t)(0xff07ffff)

#define MAC_Addr1_15_HR_Mask_19 (uint32_t)(0x1f)

#define MAC_Addr1_15_HR_ADDRHI_LPOS    0U

#define MAC_Addr1_15_HR_ADDRHI_HPOS    15U

#define MAC_Addr1_15_HR_RES_LPOS       19U

#define MAC_Addr1_15_HR_AE_LPOS        31U

#define MAC_Addr1_15_HR_AE_HPOS        31U

#define MAC_Addr1_15_HR_DCS_LPOS       16U

#define MAC_Addr1_15_HR_DCS_HPOS       18U

#define MAC_Addr1_15_HR_ADDRHI_UdfWr(i, data) do {\
  uint32_t v;\
  MAC_Addr1_15_HR_RgRd(i, v);\
  v = (v & (MAC_Addr1_15_HR_RES_Wr_Mask_19));\
  v = ((v & MAC_Addr1_15_HR_ADDRHI_Mask) | ((data & MAC_Addr1_15_HR_ADDRHI_Mask) << MAC_Addr1_15_HR_ADDRHI_LPOS));\
  MAC_Addr1_15_HR_RgWr(i, v);\
} while(0)

#define MAC_Addr1_15_HR_ADDRHI_UdfRd(i, data) do {\
  MAC_Addr1_15_HR_RgRd(i, data);\
  data = ((data >> MAC_Addr1_15_HR_ADDRHI_LPOS) & MAC_Addr1_15_HR_ADDRHI_Mask);\
} while(0)

#define MAC_Addr1_15_HR_AE_Mask (uint32_t)(0x1)

#define MAC_Addr1_15_HR_AE_Wr_Mask (uint32_t)(0x7fffffff)

#define MAC_Addr1_15_HR_AE_UdfWr(i, data) do {\
  uint32_t v;\
  MAC_Addr1_15_HR_RgRd(i, v);\
  v = (v & (MAC_Addr1_15_HR_RES_Wr_Mask_19));\
  v = ((v & MAC_Addr1_15_HR_AE_Wr_Mask) | ((data & MAC_Addr1_15_HR_AE_Mask) << MAC_Addr1_15_HR_AE_LPOS));\
  MAC_Addr1_15_HR_RgWr(i, v);\
} while(0)

#define MAC_Addr1_15_HR_AE_UdfRd(i, data) do {\
  MAC_Addr1_15_HR_RgRd(i, data);\
  data = ((data >> MAC_Addr1_15_HR_AE_LPOS) & MAC_Addr1_15_HR_AE_Mask);\
} while(0)


#define MAC_Addr1_15_LR_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x304 + (0x008*(i))))

#define MAC_Addr1_15_LR_RgWr(i, data) do {\
  iowrite32(data, (volatile void *)MAC_Addr1_15_LR_RgOffAddr(i));\
} while(0)

#define MAC_Addr1_15_LR_RgRd(i, data) do {\
  (data) = ioread32((volatile void *)MAC_Addr1_15_LR_RgOffAddr(i));\
} while(0)

#define MAC_Addr1_15_LR_ADDRLO_UdfWr(i, data) do {\
  MAC_Addr1_15_LR_RgWr(i, data);\
} while(0)

#define MAC_Addr1_15_LR_ADDRLO_UdfRd(i, data) do {\
  MAC_Addr1_15_LR_RgRd(i, data);\
} while(0)

#define NCLKCTRL_RgOffAddr      (0x1004U)
#define NCLKCTRL_POEPLLCEN_LPOS (24U)
#define NCLKCTRL_MACRXCEN_LPOS  (14U)
#define NCLKCTRL_MACTXCEN_LPOS  (7U)
#define NCLKCTRL_MACRMCEN_LPOS  (15U)
#define NCLKCTRL_MACINTCEN_LPOS (4U)

#define TC9562_NCLKCTRL_RgRd(data) do {\
  (data) = ioread32((volatile void *)(TC9562_REG_BASE_ADRS + NCLKCTRL_RgOffAddr));\
} while(0)

#define TC9562_NCLKCTRL_RgWr(data) do {\
  iowrite32((data), (volatile void *)(TC9562_REG_BASE_ADRS + NCLKCTRL_RgOffAddr));\
} while(0)

#define TC9562_NFUNCEN3_RgRd(data) do {\
  (data) = ioread32((volatile void *)(TC9562_REG_BASE_ADRS + TC9562_NFUNCEN3_OFFS));\
} while(0)

#define TC9562_NFUNCEN3_RgWr(data) do {\
  iowrite32((data), (volatile void *)(TC9562_REG_BASE_ADRS + TC9562_NFUNCEN3_OFFS));\
} while(0)
#define NRSTCTRL_RgOffAddr   (0x1008U)
#define NRSTCTRL_MACRST_LPOS (7U)
#define NRSTCTRL_INTRST_LPOS (4U)

#define TC9562_NRSTCTRL_RgRd(data) do {\
  (data) = ioread32((volatile void *)(TC9562_REG_BASE_ADRS + NRSTCTRL_RgOffAddr));\
} while(0)

#define TC9562_NRSTCTRL_RgWr(data) do {\
  iowrite32((data), (volatile void *)(TC9562_REG_BASE_ADRS + NRSTCTRL_RgOffAddr));\
} while(0)

#define DMA_Mode_RgOffAddr (0x1000U)

#define DMA_Mode_RgRd(data) do {\
  (data) = ioread32((volatile void *)(MAC_BASE_ADDRESS + DMA_Mode_RgOffAddr));\
} while(0)

#define DMA_Mode_RgWr(data) do {\
  iowrite32((data), (volatile void *)(MAC_BASE_ADDRESS + DMA_Mode_RgOffAddr));\
} while(0)

#define DMA_Mode_SWR_LPOS    (0U)
#define DMA_Mode_SWR_Mask    0x1U
#define DMA_Mode_SWR_Wr_Mask 0xFFFFFFFEU

#define DMA_Mode_SWR_UdfWr(data) do {\
  uint32_t v;\
  DMA_Mode_RgRd(v);\
  v = (v & DMA_Mode_SWR_Wr_Mask) | ( ((data) & DMA_Mode_SWR_Mask) << DMA_Mode_SWR_LPOS);\
  DMA_Mode_RgWr(v);\
} while(0)

#define DMA_Mode_INTM_Mask    0x3U
#define DMA_Mode_INTM_Wr_Mask 0xffffff3fU
#define DMA_Mode_INTM_LPOS    16U
#define DMA_Mode_INTM_HPOS    17U

#define DMA_Mode_INTM_UdfWr(data) do {\
  uint32_t v;\
  DMA_Mode_RgRd(v);\
  v = (v & DMA_Mode_INTM_Wr_Mask) | ( ((data) & DMA_Mode_INTM_Mask) << DMA_Mode_INTM_LPOS);\
  DMA_Mode_RgWr(v);\
} while(0)

#define DMA_Mode_PR_Mask    0x7U
#define DMA_Mode_PR_Wr_Mask 0xffff8fffU
#define DMA_Mode_PR_LPOS    12U
#define DMA_Mode_PR_HPOS    14U

#define DMA_Mode_PR_UdfWr(data) do {\
  uint32_t v;\
  DMA_Mode_RgRd(v);\
  v = (v & DMA_Mode_PR_Wr_Mask) | ( ((data) & DMA_Mode_PR_Mask) << DMA_Mode_PR_LPOS);\
  DMA_Mode_RgWr(v);\
} while(0)

#define DMA_Mode_TXPR_Mask    0x1U
#define DMA_Mode_TXPR_Wr_Mask 0xfffff7ffU
#define DMA_Mode_TXPR_LPOS    11U
#define DMA_Mode_TXPR_HPOS    11U

#define DMA_Mode_TXPR_UdfWr(data) do {\
  uint32_t v;\
  DMA_Mode_RgRd(v);\
  v = (v & DMA_Mode_TXPR_Wr_Mask) | ( ((data) & DMA_Mode_TXPR_Mask) << DMA_Mode_TXPR_LPOS);\
  DMA_Mode_RgWr(v);\
} while(0)

#define DMA_SBM_RgOffAddr (0x1004U)

#define DMA_SBM_RgRd(data) do {\
  (data) = ioread32((volatile void *)(MAC_BASE_ADDRESS + DMA_SBM_RgOffAddr));\
} while(0)

#define DMA_SBM_RgWr(data) do {\
  iowrite32((data), (volatile void *)(MAC_BASE_ADDRESS + DMA_SBM_RgOffAddr));\
} while(0)

#define DMA_SBM_EN_LPI_LPOS 31U
#define DMA_SBM_EN_LPI_HPOS 31U

#define DMA_SBM_LPI_XIT_PKT_LPOS 30U
#define DMA_SBM_LPI_XIT_PKT_HPOS 30U

#define DMA_SBM_WR_OSR_LMT_LPOS 24U
#define DMA_SBM_WR_OSR_LMT_HPOS 27U

#define DMA_SBM_RD_OSR_LMT_LPOS 16U
#define DMA_SBM_RD_OSR_LMT_HPOS 19U

#define DMA_SBM_RB_LPOS 15U
#define DMA_SBM_RB_HPOS 15U

#define DMA_SBM_MB_LPOS 14U
#define DMA_SBM_MB_HPOS 14U

#define DMA_SBM_ONEKBBE_LPOS 13U
#define DMA_SBM_ONEKBBE_HPOS 13U

#define DMA_SBM_AAL_LPOS 12U
#define DMA_SBM_AAL_HPOS 12U

#define DMA_SBM_EAME_LPOS 11U
#define DMA_SBM_EAME_HPOS 11U

#define DMA_SBM_AALE_LPOS 10U
#define DMA_SBM_AALE_HPOS 10U

#define DMA_SBM_BLEN256_LPOS 7U
#define DMA_SBM_BLEN256_HPOS 7U

#define DMA_SBM_BLEN128_LPOS 6U
#define DMA_SBM_BLEN128_HPOS 6U

#define DMA_SBM_BLEN64_LPOS 5U
#define DMA_SBM_BLEN64_HPOS 5U

#define DMA_SBM_BLEN32_LPOS 4U
#define DMA_SBM_BLEN32_HPOS 4U

#define DMA_SBM_BLEN16_LPOS 3U
#define DMA_SBM_BLEN16_HPOS 3U

#define DMA_SBM_BLEN8_LPOS 2U
#define DMA_SBM_BLEN8_HPOS 2U

#define DMA_SBM_BLEN4_LPOS 1U
#define DMA_SBM_BLEN4_HPOS 1U

#define DMA_SBM_FB_LPOS 0U
#define DMA_SBM_FB_HPOS 0U

#define DMA_TBS_CTRL_RgOffAddr (0x1050U)

#define DMA_TBS_CTRL_RgRd(data) do {\
  (data) = ioread32((volatile void *)(MAC_BASE_ADDRESS + DMA_TBS_CTRL_RgOffAddr));\
} while(0)

#define DMA_TBS_CTRL_RgWr(data) do {\
  iowrite32((data), (volatile void *)(MAC_BASE_ADDRESS + DMA_TBS_CTRL_RgOffAddr));\
} while(0)

#define DMA_TBS_CTRL_FTOS_LPOS 8U
#define DMA_TBS_CTRL_FTOS_HPOS 31U

#define DMA_TBS_CTRL_FGOS_LPOS 4U
#define DMA_TBS_CTRL_FGOS_HPOS 6U

#define DMA_TBS_CTRL_FTOV_LPOS 0U
#define DMA_TBS_CTRL_FTOV_HPOS 0U

#define DMA_CH_Control_RgOffAddr(i) ((0x0080U*i)+0x1100U)

#define DMA_CH_Control_RgRd(i, data) do {\
  (data) = ioread32((volatile void *)(MAC_BASE_ADDRESS + DMA_CH_Control_RgOffAddr(i)));\
} while(0)

#define DMA_CH_Control_RgWr(i, data) do {\
  iowrite32((data), (volatile void *)(MAC_BASE_ADDRESS + DMA_CH_Control_RgOffAddr(i)));\
} while(0)

#define DMA_CH_Control_SPH_LPOS 24U
#define DMA_CH_Control_SPH_HPOS 24U

#define DMA_CH_Control_DSL_LPOS 18U
#define DMA_CH_Control_DSL_HPOS 20U

#define DMA_CH_Control_PBLx8_LPOS 16U
#define DMA_CH_Control_PBLx8_HPOS 16U

#define DMA_CH_Control_MSS_LPOS 0U
#define DMA_CH_Control_MSS_HPOS 13U

#define DMA_CH_Tx_Ctrl_RgOffAddr(i) ((0x0080U*i)+0x1104U) 
#define DMA_CH_Tx_Ctrl_RES_Wr_Mask_5 (uint32_t) (0xfffff01fU)
#define DMA_CH_Tx_Ctrl_RES_Wr_Mask_23 (uint32_t) (0xff7fffffU)
#define DMA_CH_Tx_Ctrl_RES_Wr_Mask_29 (uint32_t) (0x1FFFFFFFU)

#define DMA_CH_Tx_Ctrl_RgRd(i, data) do {\
  (data) = ioread32((volatile void *)(MAC_BASE_ADDRESS + DMA_CH_Tx_Ctrl_RgOffAddr(i)));\
} while(0)

#define DMA_CH_Tx_Ctrl_RgWr(i, data) do {\
  iowrite32((data), (volatile void *)(MAC_BASE_ADDRESS + DMA_CH_Tx_Ctrl_RgOffAddr(i)));\
} while(0)

#define DMA_CH_Tx_Ctrl_EDSE_LPOS 28U
#define DMA_CH_Tx_Ctrl_EDSE_HPOS 28U

#define DMA_CH_Tx_Ctrl_TQOS_LPOS 24U
#define DMA_CH_Tx_Ctrl_TQOS_HPOS 27U

#define DMA_CH_Tx_Ctrl_ETIC_LPOS 22U
#define DMA_CH_Tx_Ctrl_ETIC_HPOS 22U

#define DMA_CH_Tx_Ctrl_TxPBL_LPOS 16U
#define DMA_CH_Tx_Ctrl_TxPBL_HPOS 21U

#define DMA_CH_Tx_Ctrl_IPBL_LPOS 15U
#define DMA_CH_Tx_Ctrl_IPBL_HPOS 15U

#define DMA_CH_Tx_Ctrl_TSE_MODE_LPOS 13U
#define DMA_CH_Tx_Ctrl_TSE_MODE_HPOS 14U

#define DMA_CH_Tx_Ctrl_TSE_LPOS 12U
#define DMA_CH_Tx_Ctrl_TSE_HPOS 12U

#define DMA_CH_Tx_Ctrl_OSF_LPOS 4U
#define DMA_CH_Tx_Ctrl_OSF_HPOS 4U

#define DMA_CH_Tx_Ctrl_TCW_LPOS 1U
#define DMA_CH_Tx_Ctrl_TCW_HPOS 3U

#define DMA_CH_Tx_Ctrl_ST_Mask (uint32_t)(1U)
#define DMA_CH_Tx_Ctrl_ST_Wr_Mask (uint32_t)(0xfffffffeU)
#define DMA_CH_Tx_Ctrl_ST_LPOS 0U
#define DMA_CH_Tx_Ctrl_ST_HPOS 0U

#define DMA_CH_Tx_Ctrl_ST_UdfWr(i, data) do {\
  uint32_t v;\
  DMA_CH_Tx_Ctrl_RgRd(i, v);\
  v = (v & (DMA_CH_Tx_Ctrl_RES_Wr_Mask_5));\
  v = (v & (DMA_CH_Tx_Ctrl_RES_Wr_Mask_23));\
  v = (v & (DMA_CH_Tx_Ctrl_RES_Wr_Mask_29));\
  v = ((v & DMA_CH_Tx_Ctrl_ST_Wr_Mask) | (((data) & DMA_CH_Tx_Ctrl_ST_Mask) << DMA_CH_Tx_Ctrl_ST_LPOS));\
  DMA_CH_Tx_Ctrl_RgWr(i, v);\
} while(0)

#define DMA_CH_Tx_Ctrl_ST_UdfRd(i, data) do {\
  DMA_CH_Tx_Ctrl_RgRd(i, data);\
  data = ((data >> DMA_CH_Tx_Ctrl_ST_LPOS) & DMA_CH_Tx_Ctrl_ST_Mask);\
} while(0)

#define DMA_CH_Rx_Ctrl_RgOffAddr(i) ((0x0080U*i)+0x1108U) 
#define DMA_CH_Rx_Ctrl_RES_Wr_Mask_15 (uint32_t)(0xffff7fffU)
#define DMA_CH_Rx_Ctrl_RES_Wr_Mask_23 (uint32_t)(0xff7fffffU)
#define DMA_CH_Rx_Ctrl_RES_Wr_Mask_28 (uint32_t)(0x8fffffffU)

#define DMA_CH_Rx_Ctrl_RgRd(i, data) do {\
  (data) = ioread32((volatile void *)(MAC_BASE_ADDRESS + DMA_CH_Rx_Ctrl_RgOffAddr(i)));\
} while(0)

#define DMA_CH_Rx_Ctrl_RgWr(i, data) do {\
  iowrite32((data), (volatile void *)(MAC_BASE_ADDRESS + DMA_CH_Rx_Ctrl_RgOffAddr(i)));\
} while(0)

#define DMA_CH_Rx_Ctrl_RPF_LPOS 31U
#define DMA_CH_Rx_Ctrl_RPF_HPOS 31U

#define DMA_CH_Rx_Ctrl_RQOS_LPOS 24U
#define DMA_CH_Rx_Ctrl_RQOS_HPOS 27U

#define DMA_CH_Rx_Ctrl_ERIC_LPOS 22U
#define DMA_CH_Rx_Ctrl_ERIC_HPOS 22U

#define DMA_CH_Rx_Ctrl_RxPBL_LPOS 16U
#define DMA_CH_Rx_Ctrl_RxPBL_HPOS 16U

#define DMA_CH_Rx_Ctrl_RBSZ_LPOS 1U
#define DMA_CH_Rx_Ctrl_RBSZ_HPOS 14U

#define DMA_CH_Rx_Ctrl_SR_Mask (uint32_t)(0x1U)
#define DMA_CH_Rx_Ctrl_SR_Wr_Mask (uint32_t)(0xfffffffeU)
#define DMA_CH_Rx_Ctrl_SR_LPOS 0U
#define DMA_CH_Rx_Ctrl_SR_HPOS 0U

#define DMA_CH_Rx_Ctrl_SR_UdfWr(i, data) do {\
  uint32_t v;\
  DMA_CH_Rx_Ctrl_RgRd(i, v);\
  v = (v & (DMA_CH_Rx_Ctrl_RES_Wr_Mask_15));\
  v = (v & (DMA_CH_Rx_Ctrl_RES_Wr_Mask_23));\
  v = (v & (DMA_CH_Rx_Ctrl_RES_Wr_Mask_28));\
  v = ((v & DMA_CH_Rx_Ctrl_SR_Wr_Mask) | (((data) & DMA_CH_Rx_Ctrl_SR_Mask) << DMA_CH_Rx_Ctrl_SR_LPOS));\
  DMA_CH_Rx_Ctrl_RgWr(i, v);\
} while(0)

#define DMA_CH_Rx_Ctrl_SR_UdfRd(i, data) do {\
  DMA_CH_Rx_Ctrl_RgRd(i, data);\
  data = ((data >> DMA_CH_Rx_Ctrl_SR_LPOS) & DMA_CH_Rx_Ctrl_SR_Mask);\
} while(0)

#define MAC_MCR_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0U))

#define MAC_MCR_RgWr(data) do {\
  iowrite32((data), (volatile void *)MAC_MCR_RgOffAddr);\
} while(0)

#define MAC_MCR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_MCR_RgOffAddr);\
} while(0)


#define MAC_MCR_PS_Mask    (uint32_t)(0x1U)

#define MAC_MCR_PS_Wr_Mask (uint32_t)(0xffff7fffU)

#define MAC_MCR_RES_Wr_Mask_7 (uint32_t)(0xffffff7fU)

#define MAC_MCR_PS_LPOS 15U

#define MAC_MCR_PS_HPOS 15U

#define MAC_MCR_PS_UdfWr(data) do {\
  uint32_t v;\
  MAC_MCR_RgRd(v);\
  v = (v & (MAC_MCR_RES_Wr_Mask_7));\
  v = ((v & MAC_MCR_PS_Wr_Mask) | (((data) & MAC_MCR_PS_Mask) << MAC_MCR_PS_LPOS));\
  MAC_MCR_RgWr(v);\
} while(0)

#define MAC_MCR_PS_UdfRd(data) do {\
  MAC_MCR_RgRd(data);\
  data = ((data >> MAC_MCR_PS_LPOS) & MAC_MCR_PS_Mask);\
} while(0)


#define MAC_MCR_FES_Mask (uint32_t)(0x1U)

#define MAC_MCR_FES_Wr_Mask (uint32_t)(0xffffbfffU)

#define MAC_MCR_FES_LPOS 14U

#define MAC_MCR_FES_HPOS 14U

#define MAC_MCR_FES_UdfWr(data) do {\
  uint32_t v;\
  MAC_MCR_RgRd(v);\
  v = (v & (MAC_MCR_RES_Wr_Mask_7));\
  v = ((v & MAC_MCR_FES_Wr_Mask) | ((data & MAC_MCR_FES_Mask) << MAC_MCR_FES_LPOS));\
  MAC_MCR_RgWr(v);\
} while(0)

#define MAC_MCR_FES_UdfRd(data) do {\
  MAC_MCR_RgRd(data);\
  data = ((data >> MAC_MCR_FES_LPOS) & MAC_MCR_FES_Mask);\
} while(0)


#define MAC_MCR_DM_Mask (uint32_t) (0x1U)

#define MAC_MCR_DM_Wr_Mask (uint32_t) (0xffffdfffU)

#define MAC_MCR_DM_LPOS 13U

#define MAC_MCR_DM_HPOS 13U

#define MAC_MCR_DM_UdfWr(data) do {\
  uint32_t v;\
  MAC_MCR_RgRd(v);\
  v = (v & (MAC_MCR_RES_Wr_Mask_7));\
  v = ((v & MAC_MCR_DM_Wr_Mask) | ((data & MAC_MCR_DM_Mask) << MAC_MCR_DM_LPOS));\
  MAC_MCR_RgWr(v);\
} while(0)

#define MAC_MCR_DM_UdfRd(data) do {\
  MAC_MCR_RgRd(data);\
  data = ((data >> MAC_MCR_DM_LPOS) & MAC_MCR_DM_Mask);\
} while(0)

#define MAC_MCR_IPC_Mask (uint32_t) (0x1U)

#define MAC_MCR_IPC_Wr_Mask (uint32_t) (0xf7ffffffU)

#define MAC_MCR_IPC_LPOS 27U

#define MAC_MCR_IPC_HPOS 27U

#define MAC_MCR_IPC_UdfWr(data) do {\
  uint32_t v;\
  MAC_MCR_RgRd(v);\
  v = (v & (MAC_MCR_RES_Wr_Mask_7));\
  v = ((v & MAC_MCR_IPC_Wr_Mask) | ((data & MAC_MCR_IPC_Mask) << MAC_MCR_IPC_LPOS));\
  MAC_MCR_RgWr(v);\
} while(0)

#define MAC_MCR_IPC_UdfRd(data) do {\
  MAC_MCR_RgRd(data);\
  data = ((data >> MAC_MCR_IPC_LPOS) & MAC_MCR_IPC_Mask);\
} while(0)

#define MAC_MCR_TE_Mask    (uint32_t)(0x1U)
#define MAC_MCR_TE_Wr_Mask (uint32_t)(0xfffffffdU)
#define MAC_MCR_TE_LPOS 1U
#define MAC_MCR_TE_HPOS 1U

#define MAC_MCR_TE_UdfWr(data) do {\
  uint32_t v;\
  MAC_MCR_RgRd(v);\
  v = (v & (MAC_MCR_RES_Wr_Mask_7));\
  v = ((v & MAC_MCR_TE_Wr_Mask) | ((data & MAC_MCR_TE_Mask) << MAC_MCR_TE_LPOS));\
  MAC_MCR_RgWr(v);\
} while(0)

#define MAC_MCR_TE_UdfRd(data) do {\
  MAC_MCR_RgRd(data);\
  data = ((data >> MAC_MCR_TE_LPOS) & MAC_MCR_TE_Mask);\
} while(0)

#define MAC_MCR_RE_Mask    (uint32_t)(0x1U)
#define MAC_MCR_RE_Wr_Mask (uint32_t)(0xfffffffeU)
#define MAC_MCR_RE_LPOS 0U
#define MAC_MCR_RE_HPOS 0U

#define MAC_MCR_RE_UdfWr(data) do {\
  uint32_t v;\
  MAC_MCR_RgRd(v);\
  v = (v & (MAC_MCR_RES_Wr_Mask_7));\
  v = ((v & MAC_MCR_RE_Wr_Mask) | ((data & MAC_MCR_RE_Mask) << MAC_MCR_RE_LPOS));\
  MAC_MCR_RgWr(v);\
} while(0)

#define MAC_MCR_RE_UdfRd(data) do {\
  MAC_MCR_RgRd(data);\
  data = ((data >> MAC_MCR_RE_LPOS) & MAC_MCR_RE_Mask);\
} while(0)

#define MAC_MCR_ACS_LPOS 20U
#define MAC_MCR_ACS_HPOS 20U

#define MAC_MCR_CST_LPOS 21U
#define MAC_MCR_CST_HPOS 21U

#define MAC_MCR_LM_LPOS  12U
#define MAC_MCR_LM_HPOS  12U

#define MAC_MECR_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x4U))

#define MAC_MECR_RgWr(data) do {\
  iowrite32((data), (volatile void *)MAC_MECR_RgOffAddr);\
} while(0)

#define MAC_MECR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_MECR_RgOffAddr);\
} while(0)


#define MAC_MPFR_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x8U))

#define MAC_MPFR_RES_Wr_Mask_22 (uint32_t)(0x803fffffU)
#define MAC_MPFR_RES_Wr_Mask_17 (uint32_t)(0xfff1ffffU)
#define MAC_MPFR_RES_Wr_Mask_11 (uint32_t)(0xffff07ffU)

#define MAC_MPFR_RgWr(data) do {\
  iowrite32((data), (volatile void *)MAC_MPFR_RgOffAddr);\
} while(0)

#define MAC_MPFR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_MPFR_RgOffAddr);\
} while(0)

#define MAC_MPFR_HPF_Mask    (uint32_t) (0x1U)

#define MAC_MPFR_HPF_Wr_Mask (uint32_t) (0xfffffbffU)

#define MAC_MPFR_HPF_LPOS 10U
#define MAC_MPFR_HPF_HPOS 10U

#define MAC_MPFR_HPF_UdfWr(data) do {\
  uint32_t v;\
  MAC_MPFR_RgRd(v);\
  v = (v & (MAC_MPFR_RES_Wr_Mask_22));\
  v = (v & (MAC_MPFR_RES_Wr_Mask_17));\
  v = (v & (MAC_MPFR_RES_Wr_Mask_11));\
  v = ((v & MAC_MPFR_HPF_Wr_Mask) | ((data & MAC_MPFR_HPF_Mask) << MAC_MPFR_HPF_LPOS ));\
  MAC_MPFR_RgWr(v);\
} while(0)

#define MAC_MPFR_HPF_UdfRd(data) do {\
  MAC_MPFR_RgRd(data);\
  data = ((data >> MAC_MPFR_HPF_LPOS) & MAC_MPFR_HPF_Mask);\
} while(0)

#define MAC_MPFR_DBF_Mask    (uint32_t) (0x1U)
#define MAC_MPFR_DBF_Wr_Mask (uint32_t) (0xffffffdfU)
#define MAC_MPFR_DBF_LPOS 5U
#define MAC_MPFR_DBF_HPOS 5U

#define MAC_MPFR_DBF_UdfWr(data) do {\
  uint32_t v;\
  MAC_MPFR_RgRd(v);\
  v = (v & (MAC_MPFR_RES_Wr_Mask_22));\
  v = (v & (MAC_MPFR_RES_Wr_Mask_17));\
  v = (v & (MAC_MPFR_RES_Wr_Mask_11));\
  v = ((v & MAC_MPFR_DBF_Wr_Mask) | ((data & MAC_MPFR_DBF_Mask) << MAC_MPFR_DBF_LPOS ));\
  MAC_MPFR_RgWr(v);\
} while(0)

#define MAC_MPFR_DBF_UdfRd(data) do {\
  MAC_MPFR_RgRd(data);\
  data = ((data >> MAC_MPFR_DBF_LPOS) & MAC_MPFR_DBF_Mask);\
} while(0)


#define MAC_MPFR_PM_Mask    (uint32_t) (0x1U)
#define MAC_MPFR_PM_Wr_Mask (uint32_t) (0xffffffefU)
#define MAC_MPFR_PM_LPOS 4U
#define MAC_MPFR_PM_HPOS 4U

#define MAC_MPFR_PM_UdfWr(data) do {\
  uint32_t v;\
  MAC_MPFR_RgRd(v);\
  v = (v & (MAC_MPFR_RES_Wr_Mask_22));\
  v = (v & (MAC_MPFR_RES_Wr_Mask_17));\
  v = (v & (MAC_MPFR_RES_Wr_Mask_11));\
  v = ((v & MAC_MPFR_PM_Wr_Mask) | ((data & MAC_MPFR_PM_Mask) << MAC_MPFR_PM_LPOS ));\
  MAC_MPFR_RgWr(v);\
} while(0)

#define MAC_MPFR_PM_UdfRd(data) do {\
  MAC_MPFR_RgRd(data);\
  data = ((data >> MAC_MPFR_PM_LPOS) & MAC_MPFR_PM_Mask);\
} while(0)

#define MAC_MPFR_RA_Mask (uint32_t) (0x1U)
#define MAC_MPFR_RA_Wr_Mask (uint32_t) (0x7fffffffU)
#define MAC_MPFR_RA_LPOS 31U
#define MAC_MPFR_RA_HPOS 31U

#define MAC_MPFR_RA_UdfWr(data) do {\
  uint32_t v;\
  MAC_MPFR_RgRd(v);\
  v = (v & (MAC_MPFR_RES_Wr_Mask_22));\
  v = (v & (MAC_MPFR_RES_Wr_Mask_17));\
  v = (v & (MAC_MPFR_RES_Wr_Mask_11));\
  v = ((v & MAC_MPFR_RA_Wr_Mask) | ((data & MAC_MPFR_RA_Mask) << MAC_MPFR_RA_LPOS ));\
  MAC_MPFR_RgWr(v);\
} while(0)

#define MAC_MPFR_RA_UdfRd(data) do {\
  MAC_MPFR_RgRd(data);\
  data = ((data >> MAC_MPFR_RA_LPOS) & MAC_MPFR_RA_Mask);\
} while(0)


#define MAC_VLANTR_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x50U))

#define MAC_VLANTR_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_VLANTR_RgOffAddr);\
} while(0)

#define MAC_VLANTR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_VLANTR_RgOffAddr);\
} while(0)

#define MAC_VLANTR_EVLS_Mask      (uint32_t)(0x3U)
#define MAC_VLANTR_EVLS_Wr_Mask   (uint32_t)(0xff9fffffU)
#define MAC_VLANTR_RES_Wr_Mask_30 (uint32_t)(0xbfffffffU)
#define MAC_VLANTR_RES_Wr_Mask_23 (uint32_t)(0xff7fffffU)

#define MAC_VLANTR_EVLS_LPOS 21U
#define MAC_VLANTR_EVLS_HPOS 22U

#define MAC_VLANTR_EVLS_UdfWr(data) do {\
  uint32_t v;\
  MAC_VLANTR_RgRd(v);\
  v = (v & (MAC_VLANTR_RES_Wr_Mask_30));\
  v = (v & (MAC_VLANTR_RES_Wr_Mask_23));\
  v = ((v & MAC_VLANTR_EVLS_Wr_Mask) | ((data & MAC_VLANTR_EVLS_Mask) << MAC_VLANTR_EVLS_LPOS));\
  MAC_VLANTR_RgWr(v);\
} while(0)

#define MAC_VLANTR_EVLS_UdfRd(data) do {\
  MAC_VLANTR_RgRd(data);\
  data = ((data >> MAC_VLANTR_EVLS_LPOS) & MAC_VLANTR_EVLS_Mask);\
} while(0)



#define MAC_QTFCR_RgOffAddr (MAC_BASE_ADDRESS + 0x70U)

#define MAC_QTFCR_RgOffAddress(i) ((volatile uint32_t *)(MAC_QTFCR_RgOffAddr + ((i)*4U)))

#define MAC_QTFCR_RES_Wr_Mask_8 (uint32_t)(0xffff00ffU)
#define MAC_QTFCR_RES_Wr_Mask_2 (uint32_t)(0xfffffff3U)

#define MAC_QTFCR_RgWr(i,data) do {\
  iowrite32(data, (volatile void *)MAC_QTFCR_RgOffAddress(i));\
} while(0)

#define MAC_QTFCR_RgRd(i,data) do {\
  (data) = ioread32((volatile void *)MAC_QTFCR_RgOffAddress(i));\
} while(0)

#define MAC_QTFCR_TFE_Mask    (uint32_t)(0x1U)
#define MAC_QTFCR_TFE_Wr_Mask (uint32_t)(0xfffffffdU)
#define MAC_QTFCR_TFE_LPOS 1U
#define MAC_QTFCR_TFE_HPOS 1U

#define MAC_QTFCR_TFE_UdfWr(i,data) do {\
  uint32_t v;\
  MAC_QTFCR_RgRd(i,v);\
  v = (v & (MAC_QTFCR_RES_Wr_Mask_8));\
  v = (v & (MAC_QTFCR_RES_Wr_Mask_2));\
  v = ((v & MAC_QTFCR_TFE_Wr_Mask) | ((data & MAC_QTFCR_TFE_Mask) << MAC_QTFCR_TFE_LPOS));\
  MAC_QTFCR_RgWr(i,v);\
} while(0)

#define MAC_QTFCR_TFE_UdfRd(i,data) do {\
  MAC_QTFCR_RgRd(i,data);\
  data = ((data >> MAC_QTFCR_TFE_LPOS) & MAC_QTFCR_TFE_Mask);\
} while(0)

#define MAC_RFCR_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x90U))

#define MAC_RFCR_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_RFCR_RgOffAddr);\
} while(0)

#define MAC_RFCR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_RFCR_RgOffAddr);\
} while(0)

#define MAC_RFCR_RES_Wr_Mask_9 (uint32_t)(0x1ffU)
#define MAC_RFCR_RES_Wr_Mask_2 (uint32_t)(0xffffff03U)

#define MAC_RFCR_RFE_Mask    (uint32_t)(0x1U)
#define MAC_RFCR_RFE_Wr_Mask (uint32_t)(0xfffffffeU)
#define MAC_RFCR_RFE_LPOS 0U
#define MAC_RFCR_RFE_HPOS 0U

#define MAC_RFCR_RFE_UdfWr(data) do {\
  uint32_t v;\
  MAC_RFCR_RgRd(v);\
  v = (v & (MAC_RFCR_RES_Wr_Mask_9));\
  v = (v & (MAC_RFCR_RES_Wr_Mask_2));\
  v = ((v & MAC_RFCR_RFE_Wr_Mask) | ((data & MAC_RFCR_RFE_Mask) << MAC_RFCR_RFE_LPOS));\
  MAC_RFCR_RgWr(v);\
} while(0)

#define MAC_RFCR_RFE_UdfRd(data) do {\
  MAC_RFCR_RgRd(data);\
  data = ((data >> MAC_RFCR_RFE_LPOS) & MAC_RFCR_RFE_Mask);\
} while(0)

#define MAC_RQC0R_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xa0U))

#define MAC_RQC0R_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_RQC0R_RgOffAddr);\
} while(0)

#define MAC_RQC0R_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_RQC0R_RgOffAddr);\
} while(0)

#define MAC_RQC0R_RES_Wr_Mask_16 (uint32_t)(0xffffU)


#define MAC_RQC0R_RXQEN3_Mask    (uint32_t)(0x3U)
#define MAC_RQC0R_RXQEN3_Wr_Mask (uint32_t)(0xffffff3fU)
#define MAC_RQC0R_RXQEN3_LPOS 6U
#define MAC_RQC0R_RXQEN3_HPOS 7U

#define MAC_RQC0R_RXQEN3_UdfWr(data) do {\
  uint32_t v;\
  MAC_RQC0R_RgRd(v);\
  v = (v & (MAC_RQC0R_RES_Wr_Mask_16));\
  v = ((v & MAC_RQC0R_RXQEN3_Wr_Mask) | ((data & MAC_RQC0R_RXQEN3_Mask) << MAC_RQC0R_RXQEN3_LPOS));\
  MAC_RQC0R_RgWr(v);\
} while(0)

#define MAC_RQC0R_RXQEN3_UdfRd(data) do {\
  MAC_RQC0R_RgRd(data);\
  data = ((data >> MAC_RQC0R_RXQEN3_LPOS) & MAC_RQC0R_RXQEN3_Mask);\
} while(0)


#define MAC_RQC0R_RXQEN2_Mask    (uint32_t)(0x3U)
#define MAC_RQC0R_RXQEN2_Wr_Mask (uint32_t)(0xffffffcfU)
#define MAC_RQC0R_RXQEN2_LPOS 4U
#define MAC_RQC0R_RXQEN2_HPOS 5U

#define MAC_RQC0R_RXQEN2_UdfWr(data) do {\
  uint32_t v;\
  MAC_RQC0R_RgRd(v);\
  v = (v & (MAC_RQC0R_RES_Wr_Mask_16));\
  v = ((v & MAC_RQC0R_RXQEN2_Wr_Mask) | ((data & MAC_RQC0R_RXQEN2_Mask) << MAC_RQC0R_RXQEN2_LPOS));\
  MAC_RQC0R_RgWr(v);\
} while(0)

#define MAC_RQC0R_RXQEN2_UdfRd(data) do {\
  MAC_RQC0R_RgRd(data);\
  data = ((data >> MAC_RQC0R_RXQEN2_LPOS) & MAC_RQC0R_RXQEN2_Mask);\
} while(0)

#define MAC_RQC0R_RXQEN1_Mask    (uint32_t)(0x3U)
#define MAC_RQC0R_RXQEN1_Wr_Mask (uint32_t)(0xfffffff3U)
#define MAC_RQC0R_RXQEN1_LPOS 2U
#define MAC_RQC0R_RXQEN1_HPOS 1U

#define MAC_RQC0R_RXQEN1_UdfWr(data) do {\
  uint32_t v;\
  MAC_RQC0R_RgRd(v);\
  v = (v & (MAC_RQC0R_RES_Wr_Mask_16));\
  v = ((v & MAC_RQC0R_RXQEN1_Wr_Mask) | ((data & MAC_RQC0R_RXQEN1_Mask) << MAC_RQC0R_RXQEN1_LPOS));\
  MAC_RQC0R_RgWr(v);\
} while(0)

#define MAC_RQC0R_RXQEN1_UdfRd(data) do {\
  MAC_RQC0R_RgRd(data);\
  data = ((data >> MAC_RQC0R_RXQEN1_LPOS) & MAC_RQC0R_RXQEN1_Mask);\
} while(0)

#define MAC_RQC0R_RXQEN0_Mask    (uint32_t)(0x3U)
#define MAC_RQC0R_RXQEN0_Wr_Mask (uint32_t)(0xfffffffcU)
#define MAC_RQC0R_RXQEN0_LPOS 0U
#define MAC_RQC0R_RXQEN0_HPOS 1U

#define MAC_RQC0R_RXQEN0_UdfWr(data) do {\
  uint32_t v;\
  MAC_RQC0R_RgRd(v);\
  v = (v & (MAC_RQC0R_RES_Wr_Mask_16));\
  v = ((v & MAC_RQC0R_RXQEN0_Wr_Mask) | ((data & MAC_RQC0R_RXQEN0_Mask) << MAC_RQC0R_RXQEN0_LPOS));\
  MAC_RQC0R_RgWr(v);\
} while(0)

#define MAC_RQC0R_RXQEN0_UdfRd(data) do {\
  MAC_RQC0R_RgRd(data);\
  data = ((data >> MAC_RQC0R_RXQEN0_LPOS) & MAC_RQC0R_RXQEN0_Mask);\
} while(0)


#define MAC_RQC1R_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xa4U))

#define MAC_RQC1R_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_RQC1R_RgOffAddr);\
} while(0)

#define MAC_RQC1R_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_RQC1R_RgOffAddr);\
} while(0)



#define MAC_RQC1R_RES_Wr_Mask_3  (0xfffffff7U)
#define MAC_RQC1R_RES_Wr_Mask_7  (0xffffff7fU)
#define MAC_RQC1R_RES_Wr_Mask_11 (0xfffff7ffU)
#define MAC_RQC1R_RES_Wr_Mask_15 (0xffff7fffU)
#define MAC_RQC1R_RES_Wr_Mask_19 (0xfff7ffffU)
#define MAC_RQC1R_RES_Wr_Mask_23 (0x7fffffU)

#define MAC_RQC1R_AVCPQ_Mask    (uint32_t)(0x7U)
#define MAC_RQC1R_AVCPQ_Wr_Mask (uint32_t)(0xfffffff8U)
#define MAC_RQC1R_AVCPQ_LPOS 0U
#define MAC_RQC1R_AVCPQ_HPOS 2U

#define MAC_RQC1R_AVCPQ_UdfWr(data) do {\
  uint32_t v;\
  MAC_RQC1R_RgRd(v);\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_23));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_19));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_15));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_11));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_7));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_3));\
  v = ((v & MAC_RQC1R_AVCPQ_Wr_Mask) | ((data & MAC_RQC1R_AVCPQ_Mask) << MAC_RQC1R_AVCPQ_LPOS));\
  MAC_RQC1R_RgWr(v);\
} while(0)

#define MAC_RQC1R_AVCPQ_UdfRd(data) do {\
  MAC_RQC1R_RgRd(data);\
  data = ((data >> MAC_RQC1R_AVCPQ_LPOS) & MAC_RQC1R_AVCPQ_Mask);\
} while(0)

#define MAC_RQC1R_PTPQ_Mask    (uint32_t)(0x7U)
#define MAC_RQC1R_PTPQ_Wr_Mask (uint32_t)(0xffffff8fU)
#define MAC_RQC1R_PTPQ_LPOS 4U
#define MAC_RQC1R_PTPQ_HPOS 6U

#define MAC_RQC1R_PTPQ_UdfWr(data) do {\
  uint32_t v;\
  MAC_RQC1R_RgRd(v);\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_23));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_19));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_15));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_11));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_7));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_3));\
  v = ((v & MAC_RQC1R_PTPQ_Wr_Mask) | ((data & MAC_RQC1R_PTPQ_Mask) << MAC_RQC1R_PTPQ_LPOS));\
  MAC_RQC1R_RgWr(v);\
} while(0)

#define MAC_RQC1R_PTPQ_UdfRd(data) do {\
  MAC_RQC1R_RgRd(data);\
  data = ((data >> MAC_RQC1R_PTPQ_LPOS) & MAC_RQC1R_PTPQ_Mask);\
} while(0)


#define MAC_RQC1R_UPQ_Mask    (uint32_t)(0x7U)
#define MAC_RQC1R_UPQ_Wr_Mask (uint32_t)(0xffff8fffU)
#define MAC_RQC1R_UPQ_LPOS 12U
#define MAC_RQC1R_UPQ_HPOS 14U

#define MAC_RQC1R_UPQ_UdfWr(data) do {\
  uint32_t v;\
  MAC_RQC1R_RgRd(v);\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_23));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_19));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_15));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_11));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_7));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_3));\
  v = ((v & MAC_RQC1R_UPQ_Wr_Mask) | ((data & MAC_RQC1R_UPQ_Mask) << MAC_RQC1R_UPQ_LPOS));\
  MAC_RQC1R_RgWr(v);\
} while(0)

#define MAC_RQC1R_UPQ_UdfRd(data) do {\
  MAC_RQC1R_RgRd(data);\
  data = ((data >> MAC_RQC1R_UPQ_LPOS) & MAC_RQC1R_UPQ_Mask);\
} while(0)


#define MAC_RQC1R_MCBQEN_Mask    (uint32_t)(0x1U)
#define MAC_RQC1R_MCBQEN_Wr_Mask (uint32_t)(0xffefffffU)
#define MAC_RQC1R_MCBQEN_LPOS 20U
#define MAC_RQC1R_MCBQEN_HPOS 20U

#define MAC_RQC1R_MCBQEN_UdfWr(data) do {\
  uint32_t v;\
  MAC_RQC1R_RgRd(v);\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_23));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_19));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_15));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_11));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_7));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_3));\
  v = ((v & MAC_RQC1R_MCBQEN_Wr_Mask) | ((data & MAC_RQC1R_MCBQEN_Mask) << MAC_RQC1R_MCBQEN_LPOS));\
  MAC_RQC1R_RgWr(v);\
} while(0)

#define MAC_RQC1R_MCBQEN_UdfRd(data) do {\
  MAC_RQC1R_RgRd(data);\
  data = ((data >> MAC_RQC1R_MCBQEN_LPOS) & MAC_RQC1R_MCBQEN_Mask);\
} while(0)

#define MAC_RQC1R_TACPQE_Mask    (uint32_t)(0x1U)
#define MAC_RQC1R_TACPQE_Wr_Mask (uint32_t)(0xffdfffffU)
#define MAC_RQC1R_TACPQE_LPOS 21U
#define MAC_RQC1R_TACPQE_HPOS 21U

#define MAC_RQC1R_TACPQE_UdfWr(data) do {\
  uint32_t v;\
  MAC_RQC1R_RgRd(v);\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_23));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_19));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_15));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_11));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_7));\
  v = (v & (MAC_RQC1R_RES_Wr_Mask_3));\
  v = ((v & MAC_RQC1R_TACPQE_Wr_Mask) | ((data & MAC_RQC1R_TACPQE_Mask) << MAC_RQC1R_TACPQE_LPOS));\
  MAC_RQC1R_RgWr(v);\
} while(0)

#define MAC_RQC1R_TACPQE_UdfRd(data) do {\
  MAC_RQC1R_RgRd(data);\
  data = ((data >> MAC_RQC1R_TACPQE_LPOS) & MAC_RQC1R_TACPQE_Mask);\
} while(0)


#define MAC_RQC2R_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xa8U))

#define MAC_RQC2R_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_RQC2R_RgOffAddr);\
} while(0)

#define MAC_RQC2R_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_RQC2R_RgOffAddr);\
} while(0)

#define MAC_RQC2R_PSRQ2_Mask    (uint32_t)(0xffU)
#define MAC_RQC2R_PSRQ2_Wr_Mask (uint32_t)(0xff00ffffU)
#define MAC_RQC2R_PSRQ2_LPOS 16U
#define MAC_RQC2R_PSRQ2_HPOS 23U
#define MAC_RQC2R_PSRQ3_LPOS 24U
#define MAC_RQC2R_PSRQ3_HPOS 31U


#define MAC_RQC2R_PSRQ2_UdfWr(data) do{\
  uint32_t v;\
  MAC_RQC2R_RgRd(v);\
  v = ((v & MAC_RQC2R_PSRQ2_Wr_Mask) | ((data & MAC_RQC2R_PSRQ2_Mask) << MAC_RQC2R_PSRQ2_LPOS));\
  MAC_RQC2R_RgWr(v);\
} while(0)

#define MAC_RQC2R_PSRQ2_UdfRd(data) do {\
  MAC_RQC2R_RgRd(data);\
  data = ((data >> MAC_RQC2R_PSRQ2_LPOS) & MAC_RQC2R_PSRQ2_Mask);\
} while(0)


#define MAC_RQC2R_PSRQ1_Mask    (uint32_t)(0xffU)
#define MAC_RQC2R_PSRQ1_Wr_Mask (uint32_t)(0xffff00ffU)
#define MAC_RQC2R_PSRQ1_LPOS 8U
#define MAC_RQC2R_PSRQ1_HPOS 15U

#define MAC_RQC2R_PSRQ1_UdfWr(data) do{\
  uint32_t v;\
  MAC_RQC2R_RgRd(v);\
  v = ((v & MAC_RQC2R_PSRQ1_Wr_Mask) | ((data & MAC_RQC2R_PSRQ1_Mask) << MAC_RQC2R_PSRQ1_LPOS));\
  MAC_RQC2R_RgWr(v);\
} while(0)

#define MAC_RQC2R_PSRQ1_UdfRd(data) do {\
  MAC_RQC2R_RgRd(data);\
  data = ((data >> MAC_RQC2R_PSRQ1_LPOS) & MAC_RQC2R_PSRQ1_Mask);\
} while(0)


#define MAC_TCR_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb00U))

#define MAC_TCR_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_TCR_RgOffAddr);\
} while(0)

#define MAC_TCR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_TCR_RgOffAddr);\
} while(0)

#define MAC_TCR_RES_Wr_Mask_7  (uint32_t) (0xffffff7fU)
#define MAC_TCR_RES_Wr_Mask_21 (uint32_t) (0xff1fffffU)
#define MAC_TCR_RES_Wr_Mask_25 (uint32_t) (0xf1ffffffU)
#define MAC_TCR_RES_Wr_Mask_29 (uint32_t) (0x1fffffffU)

#define MAC_TCR_AV8021ASMEN_Mask    (uint32_t)(0x1U)
#define MAC_TCR_AV8021ASMEN_Wr_Mask (uint32_t)(0xefffffffU)
#define MAC_TCR_AV8021ASMEN_LPOS 28U
#define MAC_TCR_AV8021ASMEN_HPOS 28U

#define MAC_TCR_AV8021ASMEN_UdfWr(data) do {\
  uint32_t v;\
  MAC_TCR_RgRd(v);\
  v = (v & (MAC_TCR_RES_Wr_Mask_29));\
  v = (v & (MAC_TCR_RES_Wr_Mask_25));\
  v = (v & (MAC_TCR_RES_Wr_Mask_21));\
  v = (v & (MAC_TCR_RES_Wr_Mask_7));\
  v = ((v & MAC_TCR_AV8021ASMEN_Wr_Mask) | ((data & MAC_TCR_AV8021ASMEN_Mask) << MAC_TCR_AV8021ASMEN_LPOS));\
  MAC_TCR_RgWr(v);\
} while(0)

#define MAC_TCR_AV8021ASMEN_UdfRd(data) do {\
  MAC_TCR_RgRd(data);\
  data = ((data >> MAC_TCR_AV8021ASMEN_LPOS) & MAC_TCR_AV8021ASMEN_Mask);\
} while(0)

#define MAC_TCR_TXTSSTSM_Mask    (uint32_t)(0x1U)
#define MAC_TCR_TXTSSTSM_Wr_Mask (uint32_t)(0xfeffffffU)
#define MAC_TCR_TXTSSTSM_LPOS 24U
#define MAC_TCR_TXTSSTSM_HPOS 24U

#define MAC_TCR_TXTSSTSM_UdfWr(data) do {\
  uint32_t v;\
  MAC_TCR_RgRd(v);\
  v = (v & (MAC_TCR_RES_Wr_Mask_29));\
  v = (v & (MAC_TCR_RES_Wr_Mask_25));\
  v = (v & (MAC_TCR_RES_Wr_Mask_21));\
  v = (v & (MAC_TCR_RES_Wr_Mask_7));\
  v = ((v & MAC_TCR_TXTSSTSM_Wr_Mask) | ((data & MAC_TCR_TXTSSTSM_Mask) << MAC_TCR_TXTSSTSM_LPOS));\
  MAC_TCR_RgWr(v);\
} while(0)

#define MAC_TCR_TXTSSTSM_UdfRd(data) do {\
  MAC_TCR_RgRd(data);\
  data = ((data >> MAC_TCR_TXTSSTSM_LPOS) & MAC_TCR_TXTSSTSM_Mask);\
} while(0)


#define MAC_TCR_TSENMACADDR_Mask    (uint32_t)(0x1U)
#define MAC_TCR_TSENMACADDR_Wr_Mask (uint32_t) (0xfffbffffU)
#define MAC_TCR_TSENMACADDR_LPOS 18U
#define MAC_TCR_TSENMACADDR_HPOS 18U

#define MAC_TCR_TSENMACADDR_UdfWr(data) do {\
  uint32_t v;\
  MAC_TCR_RgRd(v);\
  v = (v & (MAC_TCR_RES_Wr_Mask_29));\
  v = (v & (MAC_TCR_RES_Wr_Mask_25));\
  v = (v & (MAC_TCR_RES_Wr_Mask_21));\
  v = (v & (MAC_TCR_RES_Wr_Mask_7));\
  v = ((v & MAC_TCR_TSENMACADDR_Wr_Mask) | ((data & MAC_TCR_TSENMACADDR_Mask) << MAC_TCR_TSENMACADDR_LPOS));\
  MAC_TCR_RgWr(v);\
} while(0)

#define MAC_TCR_TSENMACADDR_UdfRd(data) do {\
  MAC_TCR_RgRd(data);\
  data = ((data >> MAC_TCR_TSENMACADDR_LPOS) & MAC_TCR_TSENMACADDR_Mask);\
} while(0)

#define MAC_TCR_SNAPTYPSEL_Mask (uint32_t) (0x3U)
#define MAC_TCR_SNAPTYPSEL_Wr_Mask (uint32_t) (0xfffcffffU)
#define MAC_TCR_SNAPTYPSEL_LPOS 16U
#define MAC_TCR_SNAPTYPSEL_HPOS 17U

#define MAC_TCR_SNAPTYPSEL_UdfWr(data) do {\
  uint32_t v;\
  MAC_TCR_RgRd(v);\
  v = (v & (MAC_TCR_RES_Wr_Mask_29));\
  v = (v & (MAC_TCR_RES_Wr_Mask_25));\
  v = (v & (MAC_TCR_RES_Wr_Mask_21));\
  v = (v & (MAC_TCR_RES_Wr_Mask_7));\
  v = ((v & MAC_TCR_SNAPTYPSEL_Wr_Mask) | ((data & MAC_TCR_SNAPTYPSEL_Mask) << MAC_TCR_SNAPTYPSEL_LPOS));\
  MAC_TCR_RgWr(v);\
} while(0)

#define MAC_TCR_SNAPTYPSEL_UdfRd(data) do {\
  MAC_TCR_RgRd(data);\
  data = ((data >> MAC_TCR_SNAPTYPSEL_LPOS) & MAC_TCR_SNAPTYPSEL_Mask);\
} while(0)


#define MAC_TCR_TSIPENA_Mask    (uint32_t)(0x1U)
#define MAC_TCR_TSIPENA_Wr_Mask (uint32_t)(0xfffff7ffU)
#define MAC_TCR_TSIPENA_LPOS 11U
#define MAC_TCR_TSIPENA_HPOS 11U

#define MAC_TCR_TSIPENA_UdfWr(data) do {\
  uint32_t v;\
  MAC_TCR_RgRd(v);\
  v = (v & (MAC_TCR_RES_Wr_Mask_29));\
  v = (v & (MAC_TCR_RES_Wr_Mask_25));\
  v = (v & (MAC_TCR_RES_Wr_Mask_21));\
  v = (v & (MAC_TCR_RES_Wr_Mask_7));\
  v = ((v & MAC_TCR_TSIPENA_Wr_Mask) | ((data & MAC_TCR_TSIPENA_Mask) << MAC_TCR_TSIPENA_LPOS));\
  MAC_TCR_RgWr(v);\
} while(0)

#define MAC_TCR_TSIPENA_UdfRd(data) do {\
  MAC_TCR_RgRd(data);\
  data = ((data >> MAC_TCR_TSIPENA_LPOS) & MAC_TCR_TSIPENA_Mask);\
} while(0)

#define MAC_TCR_TSVER2ENA_Mask    (uint32_t)(0x1U)
#define MAC_TCR_TSVER2ENA_Wr_Mask (uint32_t)(0xfffffbffU)
#define MAC_TCR_TSVER2ENA_LPOS 10U
#define MAC_TCR_TSVER2ENA_HPOS 10U

#define MAC_TCR_TSVER2ENA_UdfWr(data) do {\
  uint32_t v;\
  MAC_TCR_RgRd(v);\
  v = (v & (MAC_TCR_RES_Wr_Mask_29));\
  v = (v & (MAC_TCR_RES_Wr_Mask_25));\
  v = (v & (MAC_TCR_RES_Wr_Mask_21));\
  v = (v & (MAC_TCR_RES_Wr_Mask_7));\
  v = ((v & MAC_TCR_TSVER2ENA_Wr_Mask) | ((data & MAC_TCR_TSVER2ENA_Mask) << MAC_TCR_TSVER2ENA_LPOS));\
  MAC_TCR_RgWr(v);\
} while(0)

#define MAC_TCR_TSVER2ENA_UdfRd(data) do {\
  MAC_TCR_RgRd(data);\
  data = ((data >> MAC_TCR_TSVER2ENA_LPOS) & MAC_TCR_TSVER2ENA_Mask);\
} while(0)


#define MAC_TCR_TSCTRLSSR_Mask    (uint32_t)(0x1U)
#define MAC_TCR_TSCTRLSSR_Wr_Mask (uint32_t)(0xfffffdffU)
#define MAC_TCR_TSCTRLSSR_LPOS 9U
#define MAC_TCR_TSCTRLSSR_HPOS 9U

#define MAC_TCR_TSCTRLSSR_UdfWr(data) do {\
  uint32_t v;\
  MAC_TCR_RgRd(v);\
  v = (v & (MAC_TCR_RES_Wr_Mask_29));\
  v = (v & (MAC_TCR_RES_Wr_Mask_25));\
  v = (v & (MAC_TCR_RES_Wr_Mask_21));\
  v = (v & (MAC_TCR_RES_Wr_Mask_7));\
  v = ((v & MAC_TCR_TSCTRLSSR_Wr_Mask) | ((data & MAC_TCR_TSCTRLSSR_Mask) << MAC_TCR_TSCTRLSSR_LPOS));\
  MAC_TCR_RgWr(v);\
} while(0)

#define MAC_TCR_TSCTRLSSR_UdfRd(data) do {\
  MAC_TCR_RgRd(data);\
  data = ((data >> MAC_TCR_TSCTRLSSR_LPOS) & MAC_TCR_TSCTRLSSR_Mask);\
} while(0)

#define MAC_TCR_TSENA_Mask    (uint32_t)(0x1U)
#define MAC_TCR_TSENA_Wr_Mask (uint32_t)(0xfffffffeU)
#define MAC_TCR_TSENA_LPOS 0U
#define MAC_TCR_TSENA_HPOS 0U

#define MAC_TCR_TSENA_UdfWr(data) do {\
  uint32_t v;\
  MAC_TCR_RgRd(v);\
  v = (v & (MAC_TCR_RES_Wr_Mask_29));\
  v = (v & (MAC_TCR_RES_Wr_Mask_25));\
  v = (v & (MAC_TCR_RES_Wr_Mask_21));\
  v = (v & (MAC_TCR_RES_Wr_Mask_7));\
  v = ((v & MAC_TCR_TSENA_Wr_Mask) | ((data & MAC_TCR_TSENA_Mask) << MAC_TCR_TSENA_LPOS));\
  MAC_TCR_RgWr(v);\
} while(0)

#define MAC_TCR_TSENA_UdfRd(data) do {\
  MAC_TCR_RgRd(data);\
  data = ((data >> MAC_TCR_TSENA_LPOS) & MAC_TCR_TSENA_Mask);\
} while(0)

#define MAC_TCR_TSCFUPDT_Mask (uint32_t)(0x1U)
#define MAC_TCR_TSCFUPDT_Wr_Mask (uint32_t)(0xfffffffdU)
#define MAC_TCR_TSCFUPDT_LPOS 1U
#define MAC_TCR_TSCFUPDT_HPOS 1U

#define MAC_TCR_TSCFUPDT_UdfWr(data) do {\
  uint32_t v;\
  MAC_TCR_RgRd(v);\
  v = (v & (MAC_TCR_RES_Wr_Mask_29));\
  v = (v & (MAC_TCR_RES_Wr_Mask_25));\
  v = (v & (MAC_TCR_RES_Wr_Mask_21));\
  v = (v & (MAC_TCR_RES_Wr_Mask_7));\
  v = ((v & MAC_TCR_TSCFUPDT_Wr_Mask) | ((data & MAC_TCR_TSCFUPDT_Mask) << MAC_TCR_TSCFUPDT_LPOS));\
  MAC_TCR_RgWr(v);\
} while(0)

#define MAC_TCR_TSCFUPDT_UdfRd(data) do {\
  MAC_TCR_RgRd(data);\
  data = ((data >> MAC_TCR_TSCFUPDT_LPOS) & MAC_TCR_TSCFUPDT_Mask);\
} while(0)


#define MAC_TCR_TSINIT_Mask    (uint32_t)(0x1U)
#define MAC_TCR_TSINIT_Wr_Mask (uint32_t)(0xfffffffbU)
#define MAC_TCR_TSINIT_LPOS 2U
#define MAC_TCR_TSINIT_HPOS 2U

#define MAC_TCR_TSINIT_UdfWr(data) do {\
  uint32_t v;\
  MAC_TCR_RgRd(v);\
  v = (v & (MAC_TCR_RES_Wr_Mask_29));\
  v = (v & (MAC_TCR_RES_Wr_Mask_25));\
  v = (v & (MAC_TCR_RES_Wr_Mask_21));\
  v = (v & (MAC_TCR_RES_Wr_Mask_7));\
  v = ((v & MAC_TCR_TSINIT_Wr_Mask) | ((data & MAC_TCR_TSINIT_Mask) << MAC_TCR_TSINIT_LPOS));\
  MAC_TCR_RgWr(v);\
} while(0)

#define MAC_TCR_TSINIT_UdfRd(data) do {\
  MAC_TCR_RgRd(data);\
  data = ((data >> MAC_TCR_TSINIT_LPOS) & MAC_TCR_TSINIT_Mask);\
} while(0)


#define MAC_TCR_TSUPDT_Mask    (uint32_t)(0x1U)
#define MAC_TCR_TSUPDT_Wr_Mask (uint32_t)(0xfffffff7U)
#define MAC_TCR_TSUPDT_LPOS 3U
#define MAC_TCR_TSUPDT_HPOS 3U

#define MAC_TCR_TSUPDT_UdfWr(data) do {\
  uint32_t v;\
  MAC_TCR_RgRd(v);\
  v = (v & (MAC_TCR_RES_Wr_Mask_29));\
  v = (v & (MAC_TCR_RES_Wr_Mask_25));\
  v = (v & (MAC_TCR_RES_Wr_Mask_21));\
  v = (v & (MAC_TCR_RES_Wr_Mask_7));\
  v = ((v & MAC_TCR_TSUPDT_Wr_Mask) | ((data & MAC_TCR_TSUPDT_Mask) << MAC_TCR_TSUPDT_LPOS));\
  MAC_TCR_RgWr(v);\
} while(0)

#define MAC_TCR_TSUPDT_UdfRd(data) do {\
  MAC_TCR_RgRd(data);\
  data = ((data >> MAC_TCR_TSUPDT_LPOS) & MAC_TCR_TSUPDT_Mask);\
} while(0)

#define MAC_TCR_TSADDREG_Mask    (uint32_t)(0x1U)
#define MAC_TCR_TSADDREG_Wr_Mask (uint32_t)(0xffffffdfU)
#define MAC_TCR_TSADDREG_LPOS 5U
#define MAC_TCR_TSADDREG_HPOS 5U

#define MAC_TCR_TSADDREG_UdfWr(data) do {\
  uint32_t v;\
  MAC_TCR_RgRd(v);\
  v = (v & (MAC_TCR_RES_Wr_Mask_29));\
  v = (v & (MAC_TCR_RES_Wr_Mask_25));\
  v = (v & (MAC_TCR_RES_Wr_Mask_21));\
  v = (v & (MAC_TCR_RES_Wr_Mask_7));\
  v = ((v & MAC_TCR_TSADDREG_Wr_Mask) | ((data & MAC_TCR_TSADDREG_Mask) << MAC_TCR_TSADDREG_LPOS));\
  MAC_TCR_RgWr(v);\
} while(0)

#define MAC_TCR_TSADDREG_UdfRd(data) do {\
  MAC_TCR_RgRd(data);\
  data = ((data >> MAC_TCR_TSADDREG_LPOS) & MAC_TCR_TSADDREG_Mask);\
} while(0)

#define MAC_TAR_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb18U))

#define MAC_TAR_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_TAR_RgOffAddr);\
} while(0)

#define MAC_TAR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_TAR_RgOffAddr);\
} while(0)


#define MAC_SSIR_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb04U))
#define MAC_SSIR_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_SSIR_RgOffAddr);\
} while(0)

#define MAC_SSIR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_SSIR_RgOffAddr);\
} while(0)

#define MAC_SSIR_RES_Wr_Mask_0  (uint32_t) (0xffffff00U)
#define MAC_SSIR_RES_Wr_Mask_24 (uint32_t) (0xffffffU)

#define MAC_SSIR_SSINC_Mask    (uint32_t)(0xffU)
#define MAC_SSIR_SSINC_Wr_Mask (uint32_t)(0xff00ffffU)
#define MAC_SSIR_SSINC_LPOS 16U
#define MAC_SSIR_SSINC_HPOS 23U

#define MAC_SSIR_SSINC_UdfWr(data) do {\
  uint32_t v;\
  MAC_SSIR_RgRd(v);\
  v = (v & (MAC_SSIR_RES_Wr_Mask_0));\
  v = (v & (MAC_SSIR_RES_Wr_Mask_24));\
  v = ((v & MAC_SSIR_SSINC_Wr_Mask) | ((data & MAC_SSIR_SSINC_Mask) << MAC_SSIR_SSINC_LPOS));\
  MAC_SSIR_RgWr(v);\
} while(0)

#define MAC_SSIR_SSINC_UdfRd(data) do {\
  MAC_SSIR_RgRd(data);\
  data = ((data >> MAC_SSIR_SSINC_LPOS) & MAC_SSIR_SSINC_Mask);\
} while(0)

#define MAC_SSIR_SNSINC_Mask    (uint32_t)(0xffU)
#define MAC_SSIR_SNSINC_Wr_Mask (uint32_t)(0xffff00ffU)
#define MAC_SSIR_SNSINC_LPOS 8U
#define MAC_SSIR_SNSINC_HPOS 15U

#define MAC_SSIR_SNSINC_UdfWr(data) do {\
  uint32_t v;\
  MAC_SSIR_RgRd(v);\
  v = (v & (MAC_SSIR_RES_Wr_Mask_0));\
  v = (v & (MAC_SSIR_RES_Wr_Mask_24));\
  v = ((v & MAC_SSIR_SNSINC_Wr_Mask) | ((data & MAC_SSIR_SNSINC_Mask) << MAC_SSIR_SNSINC_LPOS));\
  MAC_SSIR_RgWr(v);\
} while(0)

#define MAC_SSIR_SNSINC_UdfRd(data) do {\
  MAC_SSIR_RgRd(data);\
  data = ((data >> MAC_SSIR_SNSINC_LPOS) & MAC_SSIR_SNSINC_Mask);\
} while(0)


#define MAC_STSUR_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb10U))

#define MAC_STSUR_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_STSUR_RgOffAddr);\
} while(0)

#define MAC_STSUR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_STSUR_RgOffAddr);\
} while(0)

#define MAC_STNSUR_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb14U))

#define MAC_STNSUR_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_STNSUR_RgOffAddr);\
} while(0)

#define MAC_STNSUR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_STNSUR_RgOffAddr);\
} while(0)

#define MAC_STNSUR_TSSS_Mask    (uint32_t) (0x7fffffffU)
#define MAC_STNSUR_TSSS_Wr_Mask (uint32_t) (0x80000000U)
#define MAC_STNSUR_TSSS_LPOS 0U
#define MAC_STNSUR_TSSS_HPOS 30U

#define MAC_STNSUR_TSSS_UdfWr(data) do{\
  uint32_t v;\
  MAC_STNSUR_RgRd(v);\
  v = ((v & MAC_STNSUR_TSSS_Wr_Mask) | ((data & MAC_STNSUR_TSSS_Mask)<<0));\
  MAC_STNSUR_RgWr(v);\
} while(0)

#define MAC_STNSUR_TSSS_UdfRd(data) do {\
  MAC_STNSUR_RgRd(data);\
  data = ((data >> 0) & MAC_STNSUR_TSSS_Mask);\
} while(0)


#define MAC_STNSUR_ADDSUB_Mask    (uint32_t) (0x1U)
#define MAC_STNSUR_ADDSUB_Wr_Mask (uint32_t) (0x7FFFFFFFU)
#define MAC_STNSUR_ADDSUB_LPOS 31U
#define MAC_STNSUR_ADDSUB_HPOS 31U

#define MAC_STNSUR_ADDSUB_UdfWr(data) do{\
  uint32_t v;\
  MAC_STNSUR_RgRd(v);\
  v = ((v & MAC_STNSUR_ADDSUB_Wr_Mask) | ((data & MAC_STNSUR_ADDSUB_Mask) << 0U));\
  MAC_STNSUR_RgWr(v);\
} while(0)

#define MAC_STNSUR_ADDSUB_UdfRd(data) do {\
  MAC_STNSUR_RgRd(data);\
  data = ((data >> 0) & MAC_STNSUR_ADDSUB_Mask);\
} while(0)


#define MAC_STSR_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb08U))

#define MAC_STSR_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_STSR_RgOffAddr);\
} while(0)

#define MAC_STSR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_STSR_RgOffAddr);\
} while(0)


#define MAC_STNSR_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb0cU))

#define MAC_STNSR_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_STNSR_RgOffAddr);\
} while(0)

#define MAC_STNSR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_STNSR_RgOffAddr);\
} while(0)


#define MAC_HW_Feat0_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x11cU))

#define MAC_HW_Feat0_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_HW_Feat0_RgOffAddr);\
} while(0)

#define MAC_HW_Feat0_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_HW_Feat0_RgOffAddr);\
} while(0)

#define MAC_HW_Feat0_ACTPHYSEL_LPOS 28U
#define MAC_HW_Feat0_ACTPHYSEL_HPOS 30U

#define MAC_HW_Feat0_SAVLANINS_LPOS 27U
#define MAC_HW_Feat0_SAVLANINS_HPOS 27U

#define MAC_HW_Feat0_TSSTSSEL_LPOS 25U
#define MAC_HW_Feat0_TSSTSSEL_HPOS 26U

#define MAC_HW_Feat0_MACADR64SEL_LPOS 24U
#define MAC_HW_Feat0_MACADR64SEL_HPOS 24U

#define MAC_HW_Feat0_MACADR32SEL_LPOS 23U
#define MAC_HW_Feat0_MACADR32SEL_HPOS 23U

#define MAC_HW_Feat0_ADDMACADRSEL_LPOS 18U
#define MAC_HW_Feat0_ADDMACADRSEL_HPOS 22U

#define MAC_HW_Feat0_RXCOESEL_LPOS 16U
#define MAC_HW_Feat0_RXCOESEL_HPOS 16U

#define MAC_HW_Feat0_TXCOESEL_LPOS 14U
#define MAC_HW_Feat0_TXCOESEL_HPOS 14U

#define MAC_HW_Feat0_EEESEL_LPOS 13U
#define MAC_HW_Feat0_EEESEL_HPOS 13U

#define MAC_HW_Feat0_TSSEL_LPOS 12U
#define MAC_HW_Feat0_TSSEL_HPOS 12U

#define MAC_HW_Feat0_ARPOFFSEL_LPOS 9U
#define MAC_HW_Feat0_ARPOFFSEL_HPOS 9U

#define MAC_HW_Feat0_MMCSEL_LPOS 8U
#define MAC_HW_Feat0_MMCSEL_HPOS 8U

#define MAC_HW_Feat0_MGKSEL_LPOS 7U
#define MAC_HW_Feat0_MGKSEL_HPOS 7U

#define MAC_HW_Feat0_RWKSEL_LPOS 6U
#define MAC_HW_Feat0_RWKSEL_HPOS 6U

#define MAC_HW_Feat0_SMASEL_LPOS 5U
#define MAC_HW_Feat0_SMASEL_HPOS 5U

#define MAC_HW_Feat0_VLHASH_LPOS 4U
#define MAC_HW_Feat0_VLHASH_HPOS 4U

#define MAC_HW_Feat0_PCSSEL_LPOS 3U
#define MAC_HW_Feat0_PCSSEL_HPOS 3U

#define MAC_HW_Feat0_HDSEL_LPOS 2U
#define MAC_HW_Feat0_HDSEL_HPOS 2U

#define MAC_HW_Feat0_GMIISEL_LPOS 1U
#define MAC_HW_Feat0_GMIISEL_HPOS 1U

#define MAC_HW_Feat0_MIISEL_LPOS 0U
#define MAC_HW_Feat0_MIISEL_HPOS 0U


#define MAC_HW_Feat1_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x120U))

#define MAC_HW_Feat1_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_HW_Feat1_RgOffAddr);\
} while(0)

#define MAC_HW_Feat1_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_HW_Feat1_RgOffAddr);\
} while(0)

#define MAC_HW_Feat1_L3L4FNUM_LPOS 27U
#define MAC_HW_Feat1_L3L4FNUM_HPOS 30U

#define MAC_HW_Feat1_HASHTBLSZ_LPOS 24U
#define MAC_HW_Feat1_HASHTBLSZ_HPOS 25U

#define MAC_HW_Feat1_POUOST_LPOS 23U
#define MAC_HW_Feat1_POUOST_HPOS 23U

#define MAC_HW_Feat1_RAVSEL_LPOS 21U
#define MAC_HW_Feat1_RAVSEL_HPOS 21U

#define MAC_HW_Feat1_AVSEL_LPOS 20U
#define MAC_HW_Feat1_AVSEL_HPOS 20U

#define MAC_HW_Feat1_DBGMEMA_LPOS 19U
#define MAC_HW_Feat1_DBGMEMA_HPOS 19U

#define MAC_HW_Feat1_TSOEN_LPOS 18U
#define MAC_HW_Feat1_TSOEN_HPOS 18U

#define MAC_HW_Feat1_SPHEN_LPOS 17U
#define MAC_HW_Feat1_SPHEN_HPOS 17U

#define MAC_HW_Feat1_DCBEN_LPOS 16U
#define MAC_HW_Feat1_DCBEN_HPOS 16U

#define MAC_HW_Feat1_ADDR64_LPOS 14U
#define MAC_HW_Feat1_ADDR64_HPOS 15U

#define MAC_HW_Feat1_ADVTHWORD_LPOS 13U
#define MAC_HW_Feat1_ADVTHWORD_HPOS 13U

#define MAC_HW_Feat1_PTOEN_LPOS 12U
#define MAC_HW_Feat1_PTOEN_HPOS 12U

#define MAC_HW_Feat1_OSTEN_LPOS 11U
#define MAC_HW_Feat1_OSTEN_HPOS 11U

#define MAC_HW_Feat1_TXFIFOSIZE_LPOS 6U
#define MAC_HW_Feat1_TXFIFOSIZE_HPOS 10U

#define MAC_HW_Feat1_SPRAM_LPOS 5U
#define MAC_HW_Feat1_SPRAM_HPOS 5U

#define MAC_HW_Feat1_RXFIFOSIZE_LPOS 0U
#define MAC_HW_Feat1_RXFIFOSIZE_HPOS 4U

#define MAC_HW_Feat2_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x124U))

#define MAC_HW_Feat2_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_HW_Feat2_RgOffAddr);\
} while(0)

#define MAC_HW_Feat2_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_HW_Feat2_RgOffAddr);\
} while(0)

#define MAC_HW_Feat2_AUXSNAPNUM_LPOS 28U
#define MAC_HW_Feat2_AUXSNAPNUM_HPOS 30U

#define MAC_HW_Feat2_PPSOUTNUM_LPOS 24U
#define MAC_HW_Feat2_PPSOUTNUM_HPOS 26U

#define MAC_HW_Feat2_TXCHCNT_LPOS 18U
#define MAC_HW_Feat2_TXCHCNT_HPOS 21U

#define MAC_HW_Feat2_RXCHCNT_LPOS 12U
#define MAC_HW_Feat2_RXCHCNT_HPOS 15U

#define MAC_HW_Feat2_TXQCNT_LPOS 6U
#define MAC_HW_Feat2_TXQCNT_HPOS 9U

#define MAC_HW_Feat2_RXQCNT_LPOS 0U
#define MAC_HW_Feat2_RXQCNT_HPOS 3U

#define MAC_HW_Feat3_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x128U))

#define MAC_HW_Feat3_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_HW_Feat3_RgOffAddr);\
} while(0)

#define MAC_HW_Feat3_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_HW_Feat3_RgOffAddr);\
} while(0)


#define MAC_HW_Feat3_ASP_LPOS 28U
#define MAC_HW_Feat3_ASP_HPOS 29U

#define MAC_HW_Feat3_TBSSEL_LPOS 27U
#define MAC_HW_Feat3_TBSSEL_HPOS 27U

#define MAC_HW_Feat3_FPESEL_LPOS 26U
#define MAC_HW_Feat3_FPESEL_HPOS 26U

#define MAC_HW_Feat3_ESTWID_LPOS 20U
#define MAC_HW_Feat3_ESTWID_HPOS 21U

#define MAC_HW_Feat3_ESTDEP_LPOS 17U
#define MAC_HW_Feat3_ESTDEP_HPOS 19U

#define MAC_HW_Feat3_ESTSEL_LPOS 16U
#define MAC_HW_Feat3_ESTSEL_HPOS 16U

#define MAC_HW_Feat3_FRPES_LPOS 13U
#define MAC_HW_Feat3_FRPES_HPOS 14U

#define MAC_HW_Feat3_FRPBS_LPOS 11U
#define MAC_HW_Feat3_FRPBS_HPOS 12U

#define MAC_HW_Feat3_FRPSEL_LPOS 10U
#define MAC_HW_Feat3_FRPSEL_HPOS 10U

#define MAC_HW_Feat3_PDUPSEL_LPOS 9U
#define MAC_HW_Feat3_PDUPSEL_HPOS 9U

#define MAC_HW_Feat3_DVLAN_LPOS 5U
#define MAC_HW_Feat3_DVLAN_HPOS 5U

#define MAC_HW_Feat3_CBTISEL_LPOS 4U
#define MAC_HW_Feat3_CBTISEL_HPOS 4U

#define MAC_HW_Feat3_NRVF_LPOS 0U
#define MAC_HW_Feat3_NRVF_HPOS 2U

/* Presentation Time - MCR req*/
#define MAC_Aux_TSNS_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb48U))
#define MAC_Aux_TSS_RgOffAddr  ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb4CU))

#define MAC_Aux_TSNS_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_Aux_TSNS_RgOffAddr);\
} while(0)

#define MAC_Aux_TSS_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_Aux_TSS_RgOffAddr);\
} while(0)


#define MAC_IntStatus_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb0U))

#define MAC_IntStatus_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_IntStatus_RgOffAddr);\
} while(0)

#define MAC_IntStatus_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_IntStatus_RgOffAddr);\
} while(0)

#define MAC_IER_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb4U))

#define MAC_IER_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_IER_RgOffAddr);\
} while(0)

#define MAC_IER_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_IER_RgOffAddr);\
} while(0)

#define MAC_RxTxStatus_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb8U))

#define MAC_RxTxStatus_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_RxTxStatus_RgOffAddr);\
} while(0)

#define MAC_RxTxStatus_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_RxTxStatus_RgOffAddr);\
} while(0)

#define MAC_PHYIF_Ctrl_Status_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xf8U))

#define MAC_PHYIF_Ctrl_Status_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_PHYIF_Ctrl_Status_RgOffAddr);\
} while(0)

#define MAC_PHYIF_Ctrl_Status_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_PHYIF_Ctrl_Status_RgOffAddr);\
} while(0)


#define MAC_INTSTATUS_TSIS_MASK         0x00001000U
#define MAC_INTSTATUS_TSIS_SHIFT        12U
#define MAC_TSSTATUS_ATSNS_MASK         0x3E000000U /* Number of Auxilliary TS Snapshot */
#define MAC_TSSTATUS_ATSNS_SHIFT        25U /* Number of Auxilliary TS Snapshot */
#define MAC_TSSTATUS_ATSTM_MISSED_MASK  0x01000000U /* Aux TS Snapshot Missed */
#define MAC_TSSTATUS_ATSTM_MISSED_SHIFT 24U /* Aux TS Snapshot Missed */
#define MAC_TSSTATUS_ATSSTN_MASK        0x000F0000U /* Auxilliary Timestamp Snapshot Trigger Identifier */
#define MAC_TSSTATUS_ATSSTN_SHIFT       16U /* Auxilliary Timestamp Snapshot Trigger Identifier */
#define MAC_TSSTATUS_AUXTSTRIG_MASK     0x00000004U /* Auxilliary Timestamp Trigger Snapshot: aux snapshot is written to the FIFO */
#define MAC_TSSTATUS_AUXTSTRIG_SHIFT    2U /* Auxilliary Timestamp Trigger Snapshot: aux snapshot is written to the FIFO */

#define MAC_AUXCTRL_ATSEN2_MASK      0x00000040U/* Auxilliary Snapshot 2 Enable */
#define MAC_AUXCTRL_ATSEN2_SHIFT     6U

#define MAC_IER_TSIE_LPOS_MASK       0x00001000U

#define MAC_AuxControl_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb40U))
#define MAC_TSStatus_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xb20U))

#define MAC_TSStatus_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_TSStatus_RgOffAddr);\
} while(0)

#define MAC_TSStatus_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_TSStatus_RgOffAddr);\
} while(0)

#define MAC_AuxControl_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_AuxControl_RgOffAddr);\
} while(0)

#define MAC_AuxControl_RgRd(data) do {\
  (data) = ioread32((volatile void *)MAC_AuxControl_RgOffAddr);\
} while(0)

#define MAC_MMC_Control_RgOffAddr       ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x700U))
#define MAC_MMC_RxIntMask_RgOffAddr     ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x70CU))
#define MAC_MMC_TxIntMask_RgOffAddr     ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x710U))
#define MAC_MMC_IPC_RxIntMask_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x800U))
#define MAC_MMC_FPE_TxIntMask_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x8A4U))
#define MAC_MMC_FPE_RxIntMask_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x8C4U))
  
#define CNTRS_RESET           0U
#define CNTRS_STOP_ROLLOVER   1U
#define CNTRS_RESET_ON_READ   2U
#define CNTRS_FREEZ           3U
#define CNTRS_EN_PRESET       4U
#define CNTRS_PRESET_LVL      5U
#define CNTRS_UPDATE_DBP      8U
  
#define MAC_MMC_Control_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_MMC_Control_RgOffAddr);\
} while(0)

#define MAC_MMC_RxIntMask_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_MMC_RxIntMask_RgOffAddr);\
} while(0)

#define MAC_MMC_TxIntMask_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_MMC_TxIntMask_RgOffAddr);\
} while(0)

#define MAC_MMC_IPC_RxIntMask_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_MMC_IPC_RxIntMask_RgOffAddr);\
} while(0)

#define MAC_MMC_FPE_TxIntMask_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_MMC_FPE_TxIntMask_RgOffAddr);\
} while(0)

#define MAC_MMC_FPE_RxIntMask_RgWr(data) do {\
  iowrite32(data, (volatile void *)MAC_MMC_FPE_RxIntMask_RgOffAddr);\
} while(0)

#define MAC_IER_MDIOIE_LPOS  18U
#define MAC_IER_FPEIE_LPOS   17U
#define MAC_IER_RXSTSIE_LPOS 14U
#define MAC_IER_TXSTSIE_LPOS 13U
#define MAC_IER_TSIE_LPOS    12U
#define MAC_IER_LPIIE_LPOS    5U
#define MAC_IER_PMTIE_LPOS    4U
#define MAC_IER_PHYIE_LPOS    3U
#define MAC_IER_PCSANCIE_LPOS 2U
#define MAC_IER_PCSLCHGIE_LPOS 1U
#define MAC_IER_RGSMIIIE_LPOS  0U

#define MTL_OMR_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xc00U))

#define MTL_OMR_RgWr(data) do {\
  iowrite32(data, (volatile void *)MTL_OMR_RgOffAddr);\
} while(0)

#define MTL_OMR_RgRd(data) do {\
  (data) = ioread32((volatile void *)MTL_OMR_RgOffAddr);\
} while(0)

#define MTL_OMR_RES_Wr_Mask_0  (uint32_t)(0xfffffffeU)
#define MTL_OMR_RES_Wr_Mask_3  (uint32_t)(0xffffffe7U)
#define MTL_OMR_RES_Wr_Mask_7  (uint32_t)(0xffffff7fU)
#define MTL_OMR_RES_Wr_Mask_10 (uint32_t)(0xffff83ffU)
#define MTL_OMR_RES_Wr_Mask_16 (uint32_t)(0xffffU)

#define MTL_OMR_FRPE_Mask    (uint32_t)(0x1U)
#define MTL_OMR_FRPE_Wr_Mask (uint32_t)(0xffff7fffU)
#define MTL_OMR_FRPE_LPOS 15U
#define MTL_OMR_FRPE_HPOS 15U

#define MTL_OMR_FRPE_UdfWr(data) do {\
  uint32_t v;\
  MTL_OMR_RgRd(v);\
  v = (v & (MTL_OMR_RES_Wr_Mask_0));\
  v = (v & (MTL_OMR_RES_Wr_Mask_3));\
  v = (v & (MTL_OMR_RES_Wr_Mask_7));\
  v = (v & (MTL_OMR_RES_Wr_Mask_10));\
  v = (v & (MTL_OMR_RES_Wr_Mask_16));\
  v = ((v & MTL_OMR_FRPE_Wr_Mask) | ((data & MTL_OMR_FRPE_Mask) << MTL_OMR_FRPE_LPOS));\
  MTL_OMR_RgWr(v);\
} while(0)

#define MTL_OMR_FRPE_UdfRd(data) do {\
  MTL_OMR_RgRd(data);\
  data = ((data >> MTL_OMR_FRPE_LPOS) & MTL_OMR_FRPE_Mask);\
} while(0)

#define MTL_OMR_CNTCLR_Mask    (uint32_t)(0x1U)
#define MTL_OMR_CNTCLR_Wr_Mask (uint32_t)(0xfffffdffU)
#define MTL_OMR_CNTCLR_LPOS 9U
#define MTL_OMR_CNTCLR_HPOS 9U

#define MTL_OMR_CNTCLR_UdfWr(data) do {\
  uint32_t v;\
  MTL_OMR_RgRd(v);\
  v = (v & (MTL_OMR_RES_Wr_Mask_0));\
  v = (v & (MTL_OMR_RES_Wr_Mask_3));\
  v = (v & (MTL_OMR_RES_Wr_Mask_7));\
  v = (v & (MTL_OMR_RES_Wr_Mask_10));\
  v = (v & (MTL_OMR_RES_Wr_Mask_16));\
  v = ((v & MTL_OMR_CNTCLR_Wr_Mask) | ((data & MTL_OMR_CNTCLR_Mask) << MTL_OMR_CNTCLR_LPOS));\
  MTL_OMR_RgWr(v);\
} while(0)

#define MTL_OMR_CNTCLR_UdfRd(data) do {\
  MTL_OMR_RgRd(data);\
  data = ((data >> MTL_OMR_CNTCLR_LPOS) & MTL_OMR_CNTCLR_Mask);\
} while(0)

#define MTL_OMR_CNTPRST_Mask    (uint32_t)(0x1U)
#define MTL_OMR_CNTPRST_Wr_Mask (uint32_t)(0xfffffeffU)
#define MTL_OMR_CNTPRST_LPOS 8U
#define MTL_OMR_CNTPRST_HPOS 8U

#define MTL_OMR_CNTPRST_UdfWr(data) do {\
  uint32_t v;\
  MTL_OMR_RgRd(v);\
  v = (v & (MTL_OMR_RES_Wr_Mask_0));\
  v = (v & (MTL_OMR_RES_Wr_Mask_3));\
  v = (v & (MTL_OMR_RES_Wr_Mask_7));\
  v = (v & (MTL_OMR_RES_Wr_Mask_10));\
  v = (v & (MTL_OMR_RES_Wr_Mask_16));\
  v = ((v & MTL_OMR_CNTPRST_Wr_Mask) | ((data & MTL_OMR_CNTPRST_Mask) << MTL_OMR_CNTPRST_LPOS));\
  MTL_OMR_RgWr(v);\
} while(0)

#define MTL_OMR_CNTPRST_UdfRd(data) do {\
  MTL_OMR_RgRd(data);\
  data = ((data >> MTL_OMR_CNTPRST_LPOS) & MTL_OMR_CNTPRST_Mask);\
} while(0)

#define MTL_OMR_SCHALG_Mask    (uint32_t)(0x3U)
#define MTL_OMR_SCHALG_Wr_Mask (uint32_t)(0xffffff9fU)
#define MTL_OMR_SCHALG_LPOS 5U
#define MTL_OMR_SCHALG_HPOS 6U

#define MTL_OMR_SCHALG_UdfWr(data) do {\
  uint32_t v;\
  MTL_OMR_RgRd(v);\
  v = (v & (MTL_OMR_RES_Wr_Mask_0));\
  v = (v & (MTL_OMR_RES_Wr_Mask_3));\
  v = (v & (MTL_OMR_RES_Wr_Mask_7));\
  v = (v & (MTL_OMR_RES_Wr_Mask_10));\
  v = (v & (MTL_OMR_RES_Wr_Mask_16));\
  v = ((v & MTL_OMR_SCHALG_Wr_Mask) | ((data & MTL_OMR_SCHALG_Mask) << MTL_OMR_SCHALG_LPOS));\
  MTL_OMR_RgWr(v);\
} while(0)

#define MTL_OMR_SCHALG_UdfRd(data) do {\
  MTL_OMR_RgRd(data);\
  data = ((data >> MTL_OMR_SCHALG_LPOS) & MTL_OMR_SCHALG_Mask);\
} while(0)

#define MTL_OMR_RAA_Mask    (uint32_t)(0x1U)
#define MTL_OMR_RAA_Wr_Mask (uint32_t)(0xfffffffbU)
#define MTL_OMR_RAA_LPOS 2U
#define MTL_OMR_RAA_HPOS 2U

#define MTL_OMR_RAA_UdfWr(data) do {\
  uint32_t v;\
  MTL_OMR_RgRd(v);\
  v = (v & (MTL_OMR_RES_Wr_Mask_0));\
  v = (v & (MTL_OMR_RES_Wr_Mask_3));\
  v = (v & (MTL_OMR_RES_Wr_Mask_7));\
  v = (v & (MTL_OMR_RES_Wr_Mask_10));\
  v = (v & (MTL_OMR_RES_Wr_Mask_16));\
  v = ((v & MTL_OMR_RAA_Wr_Mask) | ((data & MTL_OMR_RAA_Mask) << MTL_OMR_RAA_LPOS));\
  MTL_OMR_RgWr(v);\
} while(0)

#define MTL_OMR_RAA_UdfRd(data) do {\
  MTL_OMR_RgRd(data);\
  data = ((data >> MTL_OMR_RAA_LPOS) & MTL_OMR_RAA_Mask);\
} while(0)

#define MTL_OMR_DTXSTS_Mask    (uint32_t)(0x1U)
#define MTL_OMR_DTXSTS_Wr_Mask (uint32_t)(0xfffffffdU)
#define MTL_OMR_DTXSTS_LPOS 1U
#define MTL_OMR_DTXSTS_HPOS 1U

#define MTL_OMR_DTXSTS_UdfWr(data) do {\
  uint32_t v;\
  MTL_OMR_RgRd(v);\
  v = (v & (MTL_OMR_RES_Wr_Mask_0));\
  v = (v & (MTL_OMR_RES_Wr_Mask_3));\
  v = (v & (MTL_OMR_RES_Wr_Mask_7));\
  v = (v & (MTL_OMR_RES_Wr_Mask_10));\
  v = (v & (MTL_OMR_RES_Wr_Mask_16));\
  v = ((v & MTL_OMR_DTXSTS_Wr_Mask) | ((data & MTL_OMR_DTXSTS_Mask) << MTL_OMR_DTXSTS_LPOS));\
  MTL_OMR_RgWr(v);\
} while(0)

#define MTL_OMR_DTXSTS_UdfRd(data) do {\
  MTL_OMR_RgRd(data);\
  data = ((data >> MTL_OMR_DTXSTS_LPOS) & MTL_OMR_DTXSTS_Mask);\
} while(0)


#define MTL_RQDCM0R_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xc30U))

#define MTL_RQDCM0R_RgWr(data) do {\
  iowrite32(data, (volatile void *)MTL_RQDCM0R_RgOffAddr);\
} while(0)

#define MTL_RQDCM0R_RgRd(data) do {\
  (data) = ioread32((volatile void *)MTL_RQDCM0R_RgOffAddr);\
} while(0)

#define MTL_RQDCM0R_RES_Wr_Mask_29 (uint32_t)(0x1fffffffU)
#define MTL_RQDCM0R_RES_Wr_Mask_27 (uint32_t)(0xf7ffffffU)
#define MTL_RQDCM0R_RES_Wr_Mask_21 (uint32_t)(0xff1fffffU)
#define MTL_RQDCM0R_RES_Wr_Mask_19 (uint32_t)(0xfff7ffffU)
#define MTL_RQDCM0R_RES_Wr_Mask_13 (uint32_t)(0xffff1fffU)
#define MTL_RQDCM0R_RES_Wr_Mask_11 (uint32_t)(0xfffff7ffU)
#define MTL_RQDCM0R_RES_Wr_Mask_5  (uint32_t)(0xffffff1fU)
#define MTL_RQDCM0R_RES_Wr_Mask_3  (uint32_t)(0xfffffff7U)

#define MTL_RQDCM0R_Q3DDMACH_Mask    (uint32_t)(0x1U)
#define MTL_RQDCM0R_Q3DDMACH_Wr_Mask (uint32_t)(0xefffffffU)
#define MTL_RQDCM0R_Q3DDMACH_LPOS 28U
#define MTL_RQDCM0R_Q3DDMACH_HPOS 28U

#define MTL_RQDCM0R_Q3DDMACH_UdfWr(data) do {\
  uint32_t v;\
  MTL_RQDCM0R_RgRd(v);\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_29));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_27));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_21));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_19));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_13));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_11));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_5)) ;\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_3)) ;\
  v = ((v & MTL_RQDCM0R_Q3DDMACH_Wr_Mask) | ((data & MTL_RQDCM0R_Q3DDMACH_Mask) << MTL_RQDCM0R_Q3DDMACH_LPOS));\
  MTL_RQDCM0R_RgWr(v);\
} while(0)

#define MTL_RQDCM0R_Q3DDMACH_UdfRd(data) do {\
  MTL_RQDCM0R_RgRd(data);\
  data = ((data >> MTL_RQDCM0R_Q3DDMACH_LPOS) & MTL_RQDCM0R_Q3DDMACH_Mask);\
} while(0)


#define MTL_RQDCM0R_Q3MDMACH_Mask    (uint32_t)(0x3U)
#define MTL_RQDCM0R_Q3MDMACH_Wr_Mask (uint32_t)(0xfcffffffU)
#define MTL_RQDCM0R_Q3MDMACH_LPOS 24U
#define MTL_RQDCM0R_Q3MDMACH_HPOS 26U

#define MTL_RQDCM0R_Q3MDMACH_UdfWr(data) do {\
  uint32_t v;\
  MTL_RQDCM0R_RgRd(v);\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_29));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_27));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_21));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_19));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_13));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_11));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_5)) ;\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_3)) ;\
  v = ((v & MTL_RQDCM0R_Q3MDMACH_Wr_Mask) | ((data & MTL_RQDCM0R_Q3MDMACH_Mask) << MTL_RQDCM0R_Q3MDMACH_LPOS));\
  MTL_RQDCM0R_RgWr(v);\
} while(0)

#define MTL_RQDCM0R_Q3MDMACH_UdfRd(data) do {\
  MTL_RQDCM0R_RgRd(data);\
  data = ((data >> MTL_RQDCM0R_Q3MDMACH_LPOS) & MTL_RQDCM0R_Q3MDMACH_Mask);\
} while(0)


#define MTL_RQDCM0R_Q2DDMACH_Mask    (uint32_t)(0x1U)
#define MTL_RQDCM0R_Q2DDMACH_Wr_Mask (uint32_t)(0xffefffffU)
#define MTL_RQDCM0R_Q2DDMACH_LPOS 20U
#define MTL_RQDCM0R_Q2DDMACH_HPOS 20U

#define MTL_RQDCM0R_Q2DDMACH_UdfWr(data) do {\
  uint32_t v;\
  MTL_RQDCM0R_RgRd(v);\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_29));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_27));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_21));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_19));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_13));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_11));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_5)) ;\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_3)) ;\
  v = ((v & MTL_RQDCM0R_Q2DDMACH_Wr_Mask) | ((data & MTL_RQDCM0R_Q2DDMACH_Mask) << MTL_RQDCM0R_Q2DDMACH_LPOS));\
  MTL_RQDCM0R_RgWr(v);\
} while(0)

#define MTL_RQDCM0R_Q2DDMACH_UdfRd(data) do {\
  MTL_RQDCM0R_RgRd(data);\
  data = ((data >> MTL_RQDCM0R_Q2DDMACH_LPOS) & MTL_RQDCM0R_Q2DDMACH_Mask);\
} while(0)

#define MTL_RQDCM0R_Q2MDMACH_Mask    (uint32_t)(0x3U)
#define MTL_RQDCM0R_Q2MDMACH_Wr_Mask (uint32_t)(0xfffcffffU)
#define MTL_RQDCM0R_Q2MDMACH_LPOS 16U
#define MTL_RQDCM0R_Q2MDMACH_HPOS 18U

#define MTL_RQDCM0R_Q2MDMACH_UdfWr(data) do {\
  uint32_t v;\
  MTL_RQDCM0R_RgRd(v);\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_29));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_27));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_21));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_19));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_13));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_11));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_5)) ;\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_3)) ;\
  v = ((v & MTL_RQDCM0R_Q2MDMACH_Wr_Mask) | ((data & MTL_RQDCM0R_Q2MDMACH_Mask) << MTL_RQDCM0R_Q2MDMACH_LPOS));\
  MTL_RQDCM0R_RgWr(v);\
} while(0)

#define MTL_RQDCM0R_Q2MDMACH_UdfRd(data) do {\
  MTL_RQDCM0R_RgRd(data);\
  data = ((data >> MTL_RQDCM0R_Q2MDMACH_LPOS) & MTL_RQDCM0R_Q2MDMACH_Mask);\
} while(0)

#define MTL_RQDCM0R_Q1DDMACH_Mask    (uint32_t)(0x1U)
#define MTL_RQDCM0R_Q1DDMACH_Wr_Mask (uint32_t)(0xffffefffU)
#define MTL_RQDCM0R_Q1DDMACH_LPOS 12U
#define MTL_RQDCM0R_Q1DDMACH_HPOS 12U

#define MTL_RQDCM0R_Q1DDMACH_UdfWr(data) do {\
  uint32_t v;\
  MTL_RQDCM0R_RgRd(v);\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_29));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_27));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_21));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_19));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_13));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_11));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_5)) ;\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_3)) ;\
  v = ((v & MTL_RQDCM0R_Q1DDMACH_Wr_Mask) | ((data & MTL_RQDCM0R_Q1DDMACH_Mask) << MTL_RQDCM0R_Q1DDMACH_LPOS));\
  MTL_RQDCM0R_RgWr(v);\
} while(0

#define MTL_RQDCM0R_Q1DDMACH_UdfRd(data) do {\
  MTL_RQDCM0R_RgRd(data);\
  data = ((data >> MTL_RQDCM0R_Q1DDMACH_LPOS) & MTL_RQDCM0R_Q1DDMACH_Mask);\
} while(0)

#define MTL_RQDCM0R_Q1MDMACH_Mask    (uint32_t)(0x3U)
#define MTL_RQDCM0R_Q1MDMACH_Wr_Mask (uint32_t)(0xfffffcffU)
#define MTL_RQDCM0R_Q1MDMACH_LPOS 8U
#define MTL_RQDCM0R_Q1MDMACH_HPOS 10U

#define MTL_RQDCM0R_Q1MDMACH_UdfWr(data) do {\
  uint32_t v;\
  MTL_RQDCM0R_RgRd(v);\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_29));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_27));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_21));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_19));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_13));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_11));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_5)) ;\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_3)) ;\
  v = ((v & MTL_RQDCM0R_Q1MDMACH_Wr_Mask) | ((data & MTL_RQDCM0R_Q1MDMACH_Mask) << MTL_RQDCM0R_Q1MDMACH_LPOS));\
  MTL_RQDCM0R_RgWr(v);\
} while(0

#define MTL_RQDCM0R_Q1MDMACH_UdfRd(data) do {\
  MTL_RQDCM0R_RgRd(data);\
  data = ((data >> MTL_RQDCM0R_Q1MDMACH_LPOS) & MTL_RQDCM0R_Q1MDMACH_Mask);\
} while(0)

#define MTL_RQDCM0R_Q0DDMACH_Mask    (uint32_t)(0x1U)
#define MTL_RQDCM0R_Q0DDMACH_Wr_Mask (uint32_t)(0xffffffefU)
#define MTL_RQDCM0R_Q0DDMACH_LPOS 4U
#define MTL_RQDCM0R_Q0DDMACH_HPOS 4U

#define MTL_RQDCM0R_Q0DDMACH_UdfWr(data) do {\
  uint32_t v;\
  MTL_RQDCM0R_RgRd(v);\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_29));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_27));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_21));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_19));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_13));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_11));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_5)) ;\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_3)) ;\
  v = ((v & MTL_RQDCM0R_Q0DDMACH_Wr_Mask) | ((data & MTL_RQDCM0R_Q0DDMACH_Mask) << MTL_RQDCM0R_Q0DDMACH_LPOS));\
  MTL_RQDCM0R_RgWr(v);\
} while(0)

#define MTL_RQDCM0R_Q0DDMACH_UdfRd(data) do {\
  MTL_RQDCM0R_RgRd(data);\
  data = ((data >> MTL_RQDCM0R_Q0DDMACH_LPOS) & MTL_RQDCM0R_Q0DDMACH_Mask);\
} while(0)

#define MTL_RQDCM0R_Q0MDMACH_Mask    (uint32_t)(0x3U)
#define MTL_RQDCM0R_Q0MDMACH_Wr_Mask (uint32_t)(0xFFFFFFFCU)
#define MTL_RQDCM0R_Q0MDMACH_LPOS 0U
#define MTL_RQDCM0R_Q0MDMACH_HPOS 2U

#define MTL_RQDCM0R_Q0MDMACH_UdfWr(data) do {\
  uint32_t v;\
  MTL_RQDCM0R_RgRd(v);\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_29));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_27));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_21));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_19));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_13));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_11));\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_5)) ;\
  v = (v & (MTL_RQDCM0R_RES_Wr_Mask_3)) ;\
  v = ((v & MTL_RQDCM0R_Q0MDMACH_Wr_Mask) | ((data & MTL_RQDCM0R_Q0MDMACH_Mask) << MTL_RQDCM0R_Q0MDMACH_LPOS));\
  MTL_RQDCM0R_RgWr(v);\
} while(0)

#define MTL_RQDCM0R_Q0MDMACH_UdfRd(data) do {\
  MTL_RQDCM0R_RgRd(data);\
  data = ((data >> MTL_RQDCM0R_Q0MDMACH_LPOS) & MTL_RQDCM0R_Q0MDMACH_Mask);\
} while(0)


#define MTL_TBS_CTRL_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xc40U))

#define MTL_TBS_CTRL_RgWr(data) do {\
  iowrite32(data, (volatile void *)MTL_TBS_CTRL_RgOffAddr);\
} while(0)

#define MTL_TBS_CTRL_RgRd(data) do {\
  (data) = ioread32((volatile void *)MTL_TBS_CTRL_RgOffAddr);\
} while(0)

#define MTL_TBS_CTRL_ESTM_Mask    (uint32_t)(0x1U)
#define MTL_TBS_CTRL_ESTM_Wr_Mask (uint32_t)(0xfffffffeU)
#define MTL_TBS_CTRL_ESTM_LPOS 0U
#define MTL_TBS_CTRL_ESTM_HPOS 0U

#define MTL_TBS_CTRL_ESTM_UdfWr(data) do {\
  uint32_t v;\
  MTL_TBS_CTRL_RgRd(v);\
  v = ((v & MTL_TBS_CTRL_ESTM_Wr_Mask) | ((data & MTL_TBS_CTRL_ESTM_Mask) << MTL_TBS_CTRL_ESTM_LPOS));\
  MTL_TBS_CTRL_RgWr(v);\
}while(0)

#define MTL_TBS_CTRL_ESTM_UdfRd(data) do {\
  MTL_TBS_CTRL_RgRd(data);\
  data = ((data >> MTL_TBS_CTRL_ESTM_LPOS) & MTL_TBS_CTRL_ESTM_Mask);\
} while(0)


#define MTL_EST_CTRL_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xc50U))

#define MTL_EST_CTRL_RgWr(data) do {\
  iowrite32(data, (volatile void *)MTL_EST_CTRL_RgOffAddr);\
} while(0)

#define MTL_EST_CTRL_RgRd(data) do {\
  (data) = ioread32((volatile void *)MTL_EST_CTRL_RgOffAddr);\
} while(0)

#define MTL_EST_CTRL_EEST_Mask    (uint32_t)(0x1U)
#define MTL_EST_CTRL_EEST_Wr_Mask (uint32_t)(0x1U)
#define MTL_EST_CTRL_EEST_LPOS 0U
#define MTL_EST_CTRL_EEST_HPOS 0U

#define MTL_EST_CTRL_EEST_UdfWr(data) do {\
  uint32_t v;\
  MTL_EST_CTRL_RgRd(v);\
  v = ((v & MTL_EST_CTRL_EEST_Wr_Mask) | ((data & MTL_EST_CTRL_EEST_Mask) << MTL_EST_CTRL_EEST_LPOS));\
  MTL_EST_CTRL_RgWr(v);\
}while(0)

#define MTL_EST_CTRL_EEST_UdfRd(data) do {\
  MTL_EST_CTRL_RgRd(data);\
  data = ((data >> MTL_EST_CTRL_EEST_LPOS) & MTL_EST_CTRL_EEST_Mask);\
} while(0)

/***************** FRP reltaed register set ************************/

#define MTL_RXP_Ctrl_Status_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xca0U))

#define MTL_RXP_Ctrl_Status_RgWr(data) do {\
  iowrite32(data, (volatile void *)MTL_RXP_Ctrl_Status_RgOffAddr);\
} while(0)

#define MTL_RXP_Ctrl_Status_RgRd(data) do {\
  (data) = ioread32((volatile void *)MTL_RXP_Ctrl_Status_RgOffAddr);\
} while(0)

#define MTL_RXP_CTL_Wr_Mask_8_15  (uint32_t)(0xffff00ffU)
#define MTL_RXP_CTL_Wr_Mask_24_30 (uint32_t)(0x80ffffffU)

#define MTL_RXP_CTL_RXPI_Mask    (uint32_t)(0x1U)
#define MTL_RXP_CTL_RXPI_Wr_Mask (uint32_t)(0x7fffffffU)
#define MTL_RXP_CTL_RXPI_LPOS 31U
#define MTL_RXP_CTL_RXPI_HPOS 31U

#define MTL_RXP_CTL_RXPI_UdfWr(data) do {\
  uint32_t v;\
  MTL_RXP_Ctrl_Status_RgRd(v);\
  v = (v & (MTL_RXP_CTL_Wr_Mask_8_15));\
  v = (v & (MTL_RXP_CTL_Wr_Mask_24_30));\
  v = ((v & MTL_RXP_CTL_RXPI_Wr_Mask) | ((data & MTL_RXP_CTL_RXPI_Mask) << MTL_RXP_CTL_RXPI_LPOS));\
  MTL_RXP_Ctrl_Status_RgWr(v);\
} while(0)

#define MTL_RXP_CTL_RXPI_UdfRd(data) do {\
  MTL_RXP_Ctrl_Status_RgRd(data);\
  data = ((data >> MTL_RXP_CTL_RXPI_LPOS) & MTL_RXP_CTL_RXPI_Mask);\
} while(0)

#define MTL_RXP_CTL_NPE_Mask    (uint32_t)(0xffU)
#define MTL_RXP_CTL_NPE_Wr_Mask (uint32_t)(0xff00ffffU)
#define MTL_RXP_CTL_NPE_LPOS 16U
#define MTL_RXP_CTL_NPE_HPOS 23U

#define MTL_RXP_CTL_NPE_UdfWr(data) do {\
  uint32_t v;\
  MTL_RXP_Ctrl_Status_RgRd(v);\
  v = (v & (MTL_RXP_CTL_Wr_Mask_8_15));\
  v = (v & (MTL_RXP_CTL_Wr_Mask_24_30));\
  v = ((v & MTL_RXP_CTL_NPE_Wr_Mask) | ((data & MTL_RXP_CTL_NPE_Mask) << MTL_RXP_CTL_NPE_LPOS));\
  MTL_RXP_Ctrl_Status_RgWr(v);\
} while(0)

#define MTL_RXP_CTL_NPE_UdfRd(data) do {\
  MTL_RXP_Ctrl_Status_RgRd(data);\
  data = ((data >> MTL_RXP_CTL_NPE_LPOS) & MTL_RXP_CTL_NPE_Mask);\
} while(0)

#define MTL_RXP_CTL_NVE_Mask    (uint32_t)(0xffU)
#define MTL_RXP_CTL_NVE_Wr_Mask (uint32_t)(0xffffff00U)
#define MTL_RXP_CTL_NVE_LPOS 0U
#define MTL_RXP_CTL_NVE_HPOS 7U

#define MTL_RXP_CTL_NVE_UdfWr(data) do {\
  uint32_t v;\
  MTL_RXP_Ctrl_Status_RgRd(v);\
  v = (v & (MTL_RXP_CTL_Wr_Mask_8_15));\
  v = (v & (MTL_RXP_CTL_Wr_Mask_24_30));\
  v = ((v & MTL_RXP_CTL_NVE_Wr_Mask) | ((data & MTL_RXP_CTL_NVE_Mask) << MTL_RXP_CTL_NVE_LPOS));\
  MTL_RXP_Ctrl_Status_RgWr(v);\
} while(0)

#define MTL_RXP_CTL_NVE_UdfRd(data) do {\
  MTL_RXP_Ctrl_Status_RgRd(data);\
  data = ((data >> MTL_RXP_CTL_NVE_LPOS) & MTL_RXP_CTL_NVE_Mask);\
} while(0)


#define MTL_RXP_INDIR_AccCtrl_STS_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xcb0U))

#define MTL_RXP_INDIR_AccCtrl_STS_RgWr(data) do {\
  iowrite32(data, (volatile void *)MTL_RXP_INDIR_AccCtrl_STS_RgOffAddr);\
} while(0)

#define MTL_RXP_INDIR_AccCtrl_STS_RgRd(data) do {\
  (data) = ioread32((volatile void *)MTL_RXP_INDIR_AccCtrl_STS_RgOffAddr);\
} while(0)

#define MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_8_15  (uint32_t)(0xffff00ffU)
#define MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_17_19 (uint32_t)(0xfff1ffffU)
#define MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_23_30 (uint32_t)(0x807fffffU)

#define MTL_RXP_INDIR_AccCtrl_STS_STARTBUSY_Mask (uint32_t)(0x1U)
#define MTL_RXP_INDIR_AccCtrl_STS_STARTBUSY_Wr_Mask (uint32_t)(0x7ffffffffU)
#define MTL_RXP_INDIR_AccCtrl_STS_STARTBUSY_LPOS 31U
#define MTL_RXP_INDIR_AccCtrl_STS_STARTBUSY_HPOS 31U

#define MTL_RXP_INDIR_AccCtrl_STS_STARTBUSY_UdfWr(data) do {\
  uint32_t v;\
  MTL_RXP_INDIR_AccCtrl_STS_RgRd(v);\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_8_15));\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_17_19));\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_23_30));\
  v = ((v & MTL_RXP_INDIR_AccCtrl_STS_STARTBUSY_Wr_Mask) | \
  ((data & MTL_RXP_INDIR_AccCtrl_STS_STARTBUSY_Mask) << MTL_RXP_INDIR_AccCtrl_STS_STARTBUSY_LPOS));\
  MTL_RXP_INDIR_AccCtrl_STS_RgWr(v);\
} while(0)

#define MTL_RXP_Indirect_AccCtrl_STARTBUSY_UdfRd(data) do {\
  MTL_RXP_INDIR_AccCtrl_STS_RgRd(data);\
  data = ((data >> MTL_RXP_INDIR_AccCtrl_STS_STARTBUSY_LPOS) & MTL_RXP_INDIR_AccCtrl_STS_STARTBUSY_Mask);\
} while(0)

#define MTL_RXP_INDIR_AccCtrl_STS_RXPEIEC_Mask    (uint32_t)(0x3U)
#define MTL_RXP_INDIR_AccCtrl_STS_RXPEIEC_Wr_Mask (uint32_t)(0xff9fffffU)
#define MTL_RXP_INDIR_AccCtrl_STS_RXPEIEC_LPOS 21U
#define MTL_RXP_INDIR_AccCtrl_STS_RXPEIEC_HPOS 22U

#define MTL_RXP_INDIR_AccCtrl_STS_RXPEIEC_UdfWr(data) do {\
  uint32_t v;\
  MTL_RXP_INDIR_AccCtrl_STS_RgRd(v);\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_8_15));\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_17_19));\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_23_30));\
  v = ((v & MTL_RXP_INDIR_AccCtrl_STS_RXPEIEC_Wr_Mask) | \
  ((data & MTL_RXP_INDIR_AccCtrl_STS_RXPEIEC_Mask) << MTL_RXP_INDIR_AccCtrl_STS_RXPEIEC_LPOS));\
  MTL_RXP_INDIR_AccCtrl_STS_RgWr(v);\
} while(0)

#define MTL_RXP_Indirect_AccCtrl_RXPEIEC_UdfRd(data) do {\
  MTL_RXP_INDIR_AccCtrl_STS_RgRd(data);\
  data = ((data >> MTL_RXP_INDIR_AccCtrl_STS_RXPEIEC_LPOS) & MTL_RXP_INDIR_AccCtrl_STS_RXPEIEC_Mask);\
} while(0)

#define MTL_RXP_INDIR_AccCtrl_STS_RXPEIEE_Mask    (uint32_t)(0x1U)
#define MTL_RXP_INDIR_AccCtrl_STS_RXPEIEE_Wr_Mask (uint32_t)(0xffefffffU)
#define MTL_RXP_INDIR_AccCtrl_STS_RXPEIEE_LPOS 20U
#define MTL_RXP_INDIR_AccCtrl_STS_RXPEIEE_HPOS 20U

#define MTL_RXP_INDIR_AccCtrl_STS_RXPEIEE_UdfWr(data) do {\
  uint32_t v;\
  MTL_RXP_INDIR_AccCtrl_STS_RgRd(v);\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_8_15));\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_17_19));\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_23_30));\
  v = ((v & MTL_RXP_INDIR_AccCtrl_STS_RXPEIEE_Wr_Mask) | \
  ((data & MTL_RXP_INDIR_AccCtrl_STS_RXPEIEE_Mask) << MTL_RXP_INDIR_AccCtrl_STS_RXPEIEE_LPOS));\
  MTL_RXP_INDIR_AccCtrl_STS_RgWr(v);\
} while(0)

#define MTL_RXP_Indirect_AccCtrl_RXPEIEE_UdfRd(data) do {\
  MTL_RXP_INDIR_AccCtrl_STS_RgRd(data);\
  data = ((data >> MTL_RXP_INDIR_AccCtrl_STS_RXPEIEE_LPOS) & MTL_RXP_INDIR_AccCtrl_STS_RXPEIEE_Mask);\
} while(0)


#define MTL_RXP_INDIR_AccCtrl_STS_WRRDN_Mask    (uint32_t)(0x1U)
#define MTL_RXP_INDIR_AccCtrl_STS_WRRDN_Wr_Mask (uint32_t)(0xfffeffffU)
#define MTL_RXP_INDIR_AccCtrl_STS_WRRDN_LPOS 16U
#define MTL_RXP_INDIR_AccCtrl_STS_WRRDN_HPOS 16U

#define MTL_RXP_INDIR_AccCtrl_STS_WRRDN_UdfWr(data) do {\
  uint32_t v;\
  MTL_RXP_INDIR_AccCtrl_STS_RgRd(v);\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_8_15));\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_17_19));\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_23_30));\
  v = ((v & MTL_RXP_INDIR_AccCtrl_STS_WRRDN_Wr_Mask) | \
  ((data & MTL_RXP_INDIR_AccCtrl_STS_WRRDN_Mask) << MTL_RXP_INDIR_AccCtrl_STS_WRRDN_LPOS));\
  MTL_RXP_INDIR_AccCtrl_STS_RgWr(v);\
} while(0)

#define MTL_RXP_Indirect_AccCtrl_WRRDN_UdfRd(data) do {\
  MTL_RXP_INDIR_AccCtrl_STS_RgRd(data);\
  data = ((data >> MTL_RXP_INDIR_AccCtrl_STS_WRRDN_LPOS) & MTL_RXP_INDIR_AccCtrl_STS_WRRDN_Mask);\
} while(0)

#define MTL_RXP_INDIR_AccCtrl_STS_ADDR_Mask    (uint32_t)(0xffU)
#define MTL_RXP_INDIR_AccCtrl_STS_ADDR_Wr_Mask (uint32_t)(0xffffff00U)
#define MTL_RXP_INDIR_AccCtrl_STS_ADDR_LPOS 0U
#define MTL_RXP_INDIR_AccCtrl_STS_ADDR_HPOS 7U

#define MTL_RXP_INDIR_AccCtrl_STS_ADDRUdfWr(data) do {\
  uint32_t v;\
  MTL_RXP_INDIR_AccCtrl_STS_RgRd(v);\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_8_15));\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_17_19));\
  v = (v & (MTL_RXP_INDIR_AccCtrl_STS_Wr_Mask_23_30));\
  v = ((v & MTL_RXP_INDIR_AccCtrl_STS_ADDR_Wr_Mask) | \
    ((data & MTL_RXP_INDIR_AccCtrl_STS_ADDR_Mask) << MTL_RXP_INDIR_AccCtrl_STS_ADDR_LPOS));\
  MTL_RXP_INDIR_AccCtrl_STS_RgWr(v);\
} while(0)

#define MTL_RXP_Indirect_AccCtrl_ADDR_UdfRd(data) do {\
  MTL_RXP_INDIR_AccCtrl_STS_RgRd(data);\
  data = ((data >> MTL_RXP_INDIR_AccCtrl_STS_ADDR_LPOS) & MTL_RXP_INDIR_AccCtrl_STS_ADDR_Mask);\
} while(0)


#define MTL_RXP_Indirect_Acc_DATA_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0xcb4U))

#define MTL_RXP_Indirect_Acc_DATA_RgWr(data) do {\
  iowrite32(data, (volatile void *)MTL_RXP_Indirect_Acc_DATA_RgOffAddr);\
} while(0)

#define MTL_RXP_Indirect_Acc_DATA_RgRd(data) do {\
  (data) = ioread32((volatile void *)MTL_RXP_Indirect_Acc_DATA_RgOffAddr);\
} while(0)



/******************************************************************************/


#define MTL_OMR_CNTCLR_Mask    (uint32_t)(0x1U)
#define MTL_OMR_CNTCLR_Wr_Mask (uint32_t)(0xfffffdffU)
#define MTL_OMR_CNTCLR_LPOS 9U
#define MTL_OMR_CNTCLR_HPOS 9U

#define MTL_TxQOMR_RgOffAddr(i)   ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0040U*(i))+0x0D00U))
#define MTL_TxQOMR_RES_Wr_Mask_7  (uint32_t)(0xF780U)
#define MTL_TxQOMR_RES_Wr_Mask_22 (uint32_t)(0xFFC00000U)

#define MTL_TxQOMR_RgWr(i, data) do {\
  iowrite32(data, (volatile void *)MTL_TxQOMR_RgOffAddr(i));\
} while(0)

#define MTL_TxQOMR_RgRd(i, data) do {\
  (data) = ioread32((volatile void *)MTL_TxQOMR_RgOffAddr(i));\
} while(0)

#define MTL_TxQOMR_FTQ_Mask (uint32_t)(0x1U)
#define MTL_TxQOMR_FTQ_Wr_Mask (uint32_t)(0xfffffffeU)
#define MTL_TxQOMR_FTQ_LPOS 0U
#define MTL_TxQOMR_FTQ_HPOS 0U

#define MTL_TxQOMR_FTQ_UdfWr(i, data) do {\
  uint32_t v;\
  MTL_TxQOMR_RgRd(i,v);\
  v = (v & (MTL_TxQOMR_RES_Wr_Mask_7));\
    v = (v & (MTL_TxQOMR_RES_Wr_Mask_22));\
  v = ((v & MTL_TxQOMR_FTQ_Wr_Mask) | ((data & MTL_TxQOMR_FTQ_Mask) << MTL_TxQOMR_FTQ_LPOS));\
  MTL_TxQOMR_RgWr(i,v);\
} while(0)

#define MTL_TxQOMR_FTQ_UdfRd(i,data) do {\
  MTL_TxQOMR_RgRd(i,data);\
  data = ((data >> MTL_TxQOMR_FTQ_LPOS) & MTL_TxQOMR_FTQ_Mask);\
} while(0)

#define MTL_TxQOMR_TSF_LPOS 1U
#define MTL_TxQOMR_TSF_HPOS 1U

#define MTL_TxQOMR_TXQEN_LPOS 2U
#define MTL_TxQOMR_TXQEN_HPOS 3U

#define MTL_TxQOMR_TTC_LPOS 4U
#define MTL_TxQOMR_TTC_HPOS 6U

#define MTL_TxQOMR_TQS_LPOS 16U
#define MTL_TxQOMR_TQS_HPOS 21U

#define MTL_TxQW_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0040U*(i))+0xd18U))
#define MTL_TxQW_RES_Wr_Mask_21 (uint32_t) (0x1fffffU)

#define MTL_TxQW_RgWr(i, data) do {\
  iowrite32(data, (volatile void *)MTL_TxQW_RgOffAddr(i));\
} while(0)

#define MTL_TxQW_RgRd(i, data) do {\
  (data) = ioread32((volatile void *)MTL_TxQW_RgOffAddr(i));\
} while(0)

#define MTL_TxQW_ISCQW_Mask (uint32_t)(0x1fffffU)
#define MTL_TxQW_ISCQW_Wr_Mask (uint32_t)(0xffe00000U)
#define MTL_TxQW_ISCQW_LPOS 0U
#define MTL_TxQW_ISCQW_HPOS 20U

#define MTL_TxQW_ISCQW_UdfWr(i, data) do {\
  uint32_t v;\
  MTL_TxQW_RgRd(i,v);\
  v = (v & (MTL_TxQW_RES_Wr_Mask_21));\
  v = ((v & MTL_TxQW_ISCQW_Wr_Mask) | ((data & MTL_TxQW_ISCQW_Mask) << MTL_TxQW_ISCQW_LPOS));\
  MTL_TxQW_RgWr(i,v);\
} while(0)

#define MTL_TxQW_ISCQW_UdfRd(i,data) do {\
  MTL_TxQW_RgRd(i,data);\
  data = ((data >> MTL_TxQW_ISCQW_LPOS) & MTL_TxQW_ISCQW_Mask);\
} while(0)



#define MTL_RxQOMR_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0040U*(i))+0x0d30U))

#define MTL_RxQOMR_RgWr(i, data) do {\
  iowrite32(data, (volatile void *)MTL_RxQOMR_RgOffAddr(i));\
} while(0)

#define MTL_RxQOMR_RgRd(i, data) do {\
  (data) = ioread32((volatile void *)MTL_RxQOMR_RgOffAddr(i));\
} while(0)

#define MTL_RxQOMR_RQS_LPOS 20U
#define MTL_RxQOMR_RQS_HPOS 25U

#define MTL_RxQOMR_RFD_LPOS 14U
#define MTL_RxQOMR_RFD_HPOS 18U

#define MTL_RxQOMR_RFA_LPOS 8U
#define MTL_RxQOMR_RFA_HPOS 12U

#define MTL_RxQOMR_EHFC_LPOS 7U
#define MTL_RxQOMR_EHFC_HPOS 7U

#define MTL_RxQOMR_DIS_TCP_EF_LPOS 6U
#define MTL_RxQOMR_DIS_TCP_EF_HPOS 6U

#define MTL_RxQOMR_RSF_LPOS 5U
#define MTL_RxQOMR_RSF_HPOS 5U

#define MTL_RxQOMR_FEP_LPOS 4U
#define MTL_RxQOMR_FEP_HPOS 4U

#define MTL_RxQOMR_FUP_LPOS 3U
#define MTL_RxQOMR_FUP_HPOS 3U

#define MTL_RxQOMR_RTC_LPOS 0U
#define MTL_RxQOMR_RTC_HPOS 1U

#define MTL_TxQETSC_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + ((0x0040U*(i))+0x0d10U)))
#define MTL_TxQETSC_RES_Wr_Mask_0 (uint32_t)(0xfffffffcU)
#define MTL_TxQETSC_RES_Wr_Mask_7 (uint32_t)(0x7FU)

#define MTL_TxQETSC_RgWr(i, data) do {\
  iowrite32(data, (volatile void *)MTL_TxQETSC_RgOffAddr(i));\
} while(0)

#define MTL_TxQETSC_RgRd(i, data) do {\
  (data) = ioread32((volatile void *)MTL_TxQETSC_RgOffAddr(i));\
} while(0)

#define MTL_TxQETSC_AVALG_Mask    (uint32_t) (0x1U)
#define MTL_TxQETSC_AVALG_Wr_Mask (uint32_t) (0xfffffffbU)
#define MTL_TxQETSC_AVALG_LPOS 2U
#define MTL_TxQETSC_AVALG_HPOS 2U

#define MTL_TxQETSC_AVALG_UdfWr(i, data) do {\
  uint32_t v;\
  MTL_TxQETSC_RgRd(i,v);\
  v = (v & (MTL_TxQETSC_RES_Wr_Mask_0));\
  v = (v & (MTL_TxQETSC_RES_Wr_Mask_7));\
  v = ((v & MTL_TxQETSC_AVALG_Wr_Mask) | ((data & MTL_TxQETSC_AVALG_Mask) << MTL_TxQETSC_AVALG_LPOS));\
  MTL_TxQETSC_RgWr(i,v);\
} while(0)

#define MTL_TxQETSC_AVALG_UdfRd(i,data) do {\
  MTL_TxQETSC_RgRd(i,data);\
  data = ((data >> MTL_TxQETSC_AVALG_LPOS) & MTL_TxQETSC_AVALG_Mask);\
} while(0)

#define MTL_TxQETSC_CC_Mask (uint32_t) (0x1U)
#define MTL_TxQETSC_CC_Wr_Mask (uint32_t) (0xfffffff7U)
#define MTL_TxQETSC_CC_LPOS 3U
#define MTL_TxQETSC_CC_HPOS 3U

#define MTL_TxQETSC_CC_UdfWr(i, data) do {\
  uint32_t v;\
  MTL_TxQETSC_RgRd(i,v);\
  v = (v & (MTL_TxQETSC_RES_Wr_Mask_0));\
  v = (v & (MTL_TxQETSC_RES_Wr_Mask_7));\
  v = ((v & MTL_TxQETSC_CC_Wr_Mask) | ((data & MTL_TxQETSC_CC_Mask) << MTL_TxQETSC_CC_LPOS));\
  MTL_TxQETSC_RgWr(i,v);\
} while(0)

#define MTL_TxQETSC_CC_UdfRd(i,data) do {\
  MTL_TxQETSC_RgRd(i,data);\
  data = ((data >> MTL_TxQETSC_CC_LPOS) & MTL_TxQETSC_CC_Mask);\
} while(0)


#define MTL_TxQSSCredit_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + ((0x0040U*(i))+0x0d1cU)))
#define MTL_TxQSSCredit_RES_Wr_Mask_14 (uint32_t) (0x3fffU)

#define MTL_TxQSSCredit_RgWr(i, data) do {\
  iowrite32(data, (volatile void *)MTL_TxQSSCredit_RgOffAddr(i));\
} while(0)

#define MTL_TxQSSCredit_RgRd(i, data) do {\
  (data) = ioread32((volatile void *)MTL_TxQSSCredit_RgOffAddr(i));\
} while(0)

#define MTL_TxQSSCredit_SSC_Mask    (uint32_t)(0x3fffU)
#define MTL_TxQSSCredit_SSC_Wr_Mask (uint32_t)(0xffffc000U)
#define MTL_TxQSSCredit_SSC_LPOS 0U
#define MTL_TxQSSCredit_SSC_HPOS 13U

#define MTL_TxQSSCredit_SSC_UdfWr(i, data) do {\
  uint32_t v;\
  MTL_TxQSSCredit_RgRd(i,v);\
  v = (v & (MTL_TxQSSCredit_RES_Wr_Mask_14));\
  v = ((v & MTL_TxQSSCredit_SSC_Wr_Mask) | ((data & MTL_TxQSSCredit_SSC_Mask) << MTL_TxQSSCredit_SSC_LPOS));\
  MTL_TxQSSCredit_RgWr(i,v);\
} while(0)

#define MTL_TxQSSCredit_SSC_UdfRd(i,data) do {\
  MTL_TxQSSCredit_RgRd(i,data);\
  data = ((data >> MTL_TxQSSCredit_SSC_LPOS) & MTL_TxQSSCredit_SSC_Mask);\
} while(0)


#define MTL_TxQHiCredit_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + ((0x0040U*(i))+0x0d20U)))
#define MTL_TxQHiCredit_RES_Wr_Mask_29 (uint32_t) (0x1fffffffU)

#define MTL_TxQHiCredit_RgWr(i, data) do {\
  iowrite32(data, (volatile void *)MTL_TxQHiCredit_RgOffAddr(i));\
} while(0)

#define MTL_TxQHiCredit_RgRd(i, data) do {\
  (data) = ioread32((volatile void *)MTL_TxQHiCredit_RgOffAddr(i));\
} while(0)

#define MTL_TxQHiCredit_HC_Mask    (uint32_t) (0x1fffffffU)
#define MTL_TxQHiCredit_HC_Wr_Mask (uint32_t) (0xe0000000U)
#define MTL_TxQHiCredit_HC_LPOS 0U
#define MTL_TxQHiCredit_HC_HPOS 28U

#define MTL_TxQHiCredit_HC_UdfWr(i, data) do {\
  uint32_t v;\
  MTL_TxQHiCredit_RgRd(i,v);\
  v = (v & (MTL_TxQHiCredit_RES_Wr_Mask_29));\
  v = ((v & MTL_TxQHiCredit_HC_Wr_Mask) | ((data & MTL_TxQHiCredit_HC_Mask) << MTL_TxQHiCredit_HC_LPOS));\
  MTL_TxQHiCredit_RgWr(i,v);\
} while(0)

#define MTL_TxQHiCredit_HC_UdfRd(i,data) do {\
  MTL_TxQHiCredit_RgRd(i,data);\
  data = ((data >> MTL_TxQHiCredit_HC_LPOS) & MTL_TxQHiCredit_HC_Mask);\
} while(0)


#define MTL_TxQLoCredit_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + ((0x0040U*(i))+0x0D24U)))
#define MTL_TxQLoCredit_RES_Wr_Mask_29 (uint32_t) (0x1fffffffU)

#define MTL_TxQLoCredit_RgWr(i, data) do {\
  iowrite32(data, (volatile void *)MTL_TxQLoCredit_RgOffAddr(i));\
} while(0)

#define MTL_TxQLoCredit_RgRd(i, data) do {\
  (data) = ioread32((volatile void *)MTL_TxQLoCredit_RgOffAddr(i));\
} while(0)

#define MTL_TxQLoCredit_LC_Mask    (uint32_t) (0x1fffffffU)
#define MTL_TxQLoCredit_LC_Wr_Mask (uint32_t) (0xe0000000U)
#define MTL_TxQLoCredit_LC_LPOS 0U
#define MTL_TxQLoCredit_LC_HPOS 28U

#define MTL_TxQLoCredit_LC_UdfWr(i, data) do {\
  uint32_t v;\
  MTL_TxQLoCredit_RgRd(i,v);\
  v = (v & (MTL_TxQLoCredit_RES_Wr_Mask_29));\
  v = ((v & MTL_TxQLoCredit_LC_Wr_Mask) | ((data & MTL_TxQLoCredit_LC_Mask) << MTL_TxQLoCredit_LC_LPOS));\
  MTL_TxQLoCredit_RgWr(i,v);\
} while(0)

#define MTL_TxQLoCredit_LC_UdfRd(i,data) do {\
  MTL_TxQLoCredit_RgRd(i,data);\
  data = ((data >> MTL_TxQLoCredit_LC_LPOS) & MTL_TxQLoCredit_LC_Mask);\
} while(0)



#define NEMACCTL_RgOffAddr          (0x0010U)
#define NEMACCTL_PHY_INTF_SEL_Mask  (uint32_t)(0x7U)
#define NEMACCTL_PHY_INTF_SEL_Wr_Mask  (uint32_t) (0xffffffc7U)
#define NEMACCTL_PHY_INTF_SEL_LPOS  3U
#define NEMACCTL_PHY_INTF_SEL_HPOS  5U

#define TC9562_NEMACCTL_RgRd(data) do {\
  (data) = ioread32((volatile void *)(TC9562_REG_BASE_ADRS + NEMACCTL_RgOffAddr));\
} while(0)

#define TC9562_NEMACCTL_RgWr(data) do {\
  iowrite32((data), (volatile void *)(TC9562_REG_BASE_ADRS + NEMACCTL_RgOffAddr));\
} while(0)

#define NEMACCTL_PHY_INTF_SEL_UdfRd(data) do {\
  TC9562_NEMACCTL_RgRd(data);\
  data = ((data >> NEMACCTL_PHY_INTF_SEL_LPOS) & NEMACCTL_PHY_INTF_SEL_Mask);\
} while(0)

#define NEMACCTL_PHY_INTF_SEL_UdfWr(data) do {\
  uint32_t v;\
  TC9562_NEMACCTL_RgRd(v);\
  v = ((v & NEMACCTL_PHY_INTF_SEL_Wr_Mask) | ((data & NEMACCTL_PHY_INTF_SEL_Mask) << NEMACCTL_PHY_INTF_SEL_LPOS));\
  TC9562_NEMACCTL_RgWr(v);\
}while(0)


#define NEMACCTL_MAC_SP_SEL_Mask    (uint32_t) (0x3U)
#define NEMACCTL_MAC_SP_SEL_Wr_Mask (uint32_t) (0xFFFFFFFCU)
#define NEMACCTL_MAC_SP_SEL_LPOS 0U
#define NEMACCTL_MAC_SP_SEL_HPOS 1U

#define NEMACCTL_MAC_SP_SEL_UdfRd(data) do {\
  TC9562_NEMACCTL_RgRd(data);\
  data = ((data >> NEMACCTL_MAC_SP_SEL_LPOS) & NEMACCTL_MAC_SP_SEL_Mask);\
} while(0)

#define NEMACCTL_MAC_SP_SEL_UdfWr(data) do {\
  uint32_t v;\
  TC9562_NEMACCTL_RgRd(v);\
  v = ((v & NEMACCTL_MAC_SP_SEL_Wr_Mask) | ((data & NEMACCTL_MAC_SP_SEL_Mask) << NEMACCTL_MAC_SP_SEL_LPOS));\
  TC9562_NEMACCTL_RgWr(v);\
}while(0)


#define RX_NORMAL_DESC_RDES0_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define RX_NORMAL_DESC_RDES0_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define RX_NORMAL_DESC_RDES1_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define RX_NORMAL_DESC_RDES1_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define RX_NORMAL_DESC_RDES2_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define RX_NORMAL_DESC_RDES2_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define RX_NORMAL_DESC_RDES3_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define RX_NORMAL_DESC_RDES3_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define DMA_CH_RxDRLen_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0080U*(i))+0x1130U))

#define DMA_CH_RxDesc_RingLen_Rd(i, data) do {\
  (data) = ioread32((volatile void *)(DMA_CH_RxDRLen_RgOffAddr(i)));\
} while(0)

#define DMA_CH_RxDesc_RingLen_Wr(i, data) do {\
  iowrite32((data), (volatile void *)(DMA_CH_RxDRLen_RgOffAddr(i)));\
} while(0)


#define DMA_CH_RxDLA_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0080U*(i))+0x111cU))

#define DMA_CH_RxDesc_List_Addr_Rd(i, data) do {\
  (data) = ioread32((volatile void *)(DMA_CH_RxDLA_RgOffAddr(i)));\
} while(0)

#define DMA_CH_RxDesc_List_Addr_Wr(i, data) do {\
  iowrite32((data), (volatile void *)(DMA_CH_RxDLA_RgOffAddr(i)));\
} while(0)

#define GET_RX_DESC_DMA_ADDR(qInx, dInx) ((uint32_t)(emac_dev.RxNormDescBase[qInx] + dInx))

#define DMA_CH_RxDTP_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0080U*(i))+0x1128U))

#define DMA_CH_RxDesc_Tail_Ptr_Rd(i, data) do {\
  (data) = ioread32((volatile void *)(DMA_CH_RxDTP_RgOffAddr(i)));\
} while(0)

#define DMA_CH_RxDesc_Tail_Ptr_Wr(i, data) do {\
  iowrite32((data), (volatile void *)(DMA_CH_RxDTP_RgOffAddr(i)));\
} while(0)

#define TX_NORMAL_DESC_TDES0_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define TX_NORMAL_DESC_TDES0_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define TX_NORMAL_DESC_TDES1_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define TX_NORMAL_DESC_TDES1_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define TX_NORMAL_DESC_TDES2_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define TX_NORMAL_DESC_TDES2_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define TX_NORMAL_DESC_TDES3_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define TX_NORMAL_DESC_TDES3_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)


#define TX_ENH_DESC_TDES0_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define TX_ENH_DESC_TDES0_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define TX_ENH_DESC_TDES1_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define TX_ENH_DESC_TDES1_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define TX_ENH_DESC_TDES2_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define TX_ENH_DESC_TDES2_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define TX_ENH_DESC_TDES3_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define TX_ENH_DESC_TDES3_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define TX_ENH_DESC_TDES4_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define TX_ENH_DESC_TDES4_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define TX_ENH_DESC_TDES5_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define TX_ENH_DESC_TDES5_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define TX_ENH_DESC_TDES6_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define TX_ENH_DESC_TDES6_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)

#define TX_ENH_DESC_TDES7_Ml_Rd(ptr, data) do { \
  data = ptr; \
} while(0)

#define TX_ENH_DESC_TDES7_Ml_Wr(ptr, data) do { \
  ptr = data; \
} while(0)


#define DMA_CH_TxDRLen_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0080U*(i))+0x112cU))

#define DMA_CH_TxDesc_RingLen_Rd(i, data) do {\
  (data) = ioread32((volatile void *)(DMA_CH_TxDRLen_RgOffAddr(i)));\
} while(0)

#define DMA_CH_TxDesc_RingLen_Wr(i, data) do {\
  iowrite32((data), (volatile void *)(DMA_CH_TxDRLen_RgOffAddr(i)));\
} while(0)

#define DMA_CH_TxDTP_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0080U*(i))+0x1120U))

#define DMA_CH_TxDesc_Tail_Ptr_Rd(i, data) do {\
  (data) = ioread32((volatile void *)(DMA_CH_TxDTP_RgOffAddr(i)));\
} while(0)

#define DMA_CH_TxDesc_Tail_Ptr_Wr(i, data) do {\
  iowrite32((data), (volatile void *)(DMA_CH_TxDTP_RgOffAddr(i)));\
} while(0)

#define DMA_CH_TxDLA_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0080U*(i))+0x1114U))

#define DMA_CH_TxDesc_List_Addr_Rd(i, data) do {\
  (data) = ioread32((volatile void *)(DMA_CH_TxDLA_RgOffAddr(i)));\
} while(0)

#define DMA_CH_TxDesc_List_Addr_Wr(i, data) do {\
  iowrite32((data), (volatile void *)(DMA_CH_TxDLA_RgOffAddr(i)));\
} while(0)


#define DMA_Int_Status_RgOffAddr ((volatile uint32_t *)(MAC_BASE_ADDRESS + 0x1008U))

#define DMA_Int_Status_RgRd(data) do {\
  iowrite32(data, (volatile void *)DMA_Int_Status_RgOffAddr);\
} while(0)

#define DMA_Int_Status_RgWr(data) do {\
  (data) = ioread32((volatile void *)DMA_Int_Status_RgOffAddr);\
} while(0)


#define GET_TX_NORM_DESC_DMA_ADDR(qInx, dInx) ((uint32_t)(emac_dev.TxNormDescBase[(qInx)] + (dInx)))
#define GET_TX_ENH_DESC_DMA_ADDR(qInx, dInx) ((uint32_t)(emac_dev.TxEnhDescBase + (dInx)))


#define TX_NORM_DESC2_B1L_LPOS 0U
#define TX_NORM_DESC2_TTSE_LPOS 30U
#define TX_NORM_DESC2_IOC_LPOS 31U

#define TX_NORM_DESC2_B1L_Mask 0x3fffU

#define TX_NORM_DESC3_DMA_OWN 0x80000000U
#define TX_NORM_DESC3_DMA_OWN_LPOS 31U
#define TX_NORM_DESC3_FD_LPOS 29U
#define TX_NORM_DESC3_LD_LPOS 28U
#define TX_NORM_DESC3_FL 0x7fffU
#define TX_NORM_DESC3_FL_LPOS 0

#define TX_ENH_DESC4_LTV_LPOS 31U
#define TX_ENH_DESC4_LT_LPOS 0U
#define TX_ENH_DESC4_LT_Wr_Mask (uint32_t) (0xffU)

#define TX_NORM_DESC3_TTSS_Mask 0x20000U

#define TX_ENH_DESC5_LT_LPOS 8U
#define TX_ENH_DESC5_LT_Wr_Mask (uint32_t) (0x00ffffffU)

#define TX_ENH_DESC2_B1L_LPOS 0U
#define TX_ENH_DESC2_TTSE_LPOS 30U
#define TX_ENH_DESC2_IOC_LPOS 31U

#define TX_ENH_DESC2_B1L_Mask 0x3fffU

#define TX_ENH_DESC3_DMA_OWN 0x80000000U
#define TX_ENH_DESC3_DMA_OWN_LPOS 31U
#define TX_ENH_DESC3_FD_LPOS 29U
#define TX_ENH_DESC3_LD_LPOS 28U
#define TX_ENH_DESC3_FL 0x7fffU
#define TX_ENH_DESC3_FL_LPOS 0U

#define RX_NORM_DESC3_DMA_OWN 0x80000000U
#define RX_NORM_DESC3_CTXT 0x40000000U
#define RX_NORM_DESC1_IPCE 0x80U
#define RX_NORM_DESC1_IPHE 0x8U
#define RX_NORM_DESC3_CE 0x1000000U
#define RX_NORM_DESC3_OE 0x200000U
#define RX_NORM_DESC3_RE 0x100000U
#define RX_NORM_DESC3_DE 0x80000U
#define RX_NORM_DESC3_RWT 0x400000U
#define RX_NORM_DESC3_PL 0x7FFFU
#define RX_NORM_DESC1_TSA 0x4000U


#define DMA_CHIE_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0080U*(i))+0x1134U))

#define DMA_CH_IntEnable_Rd(i, data) do {\
  (data) = ioread32((volatile void *)(DMA_CHIE_RgOffAddr(i)));\
}while(0)

#define DMA_CH_IntEnable_Wr(i, data) do {\
  iowrite32((data), (volatile void *)(DMA_CHIE_RgOffAddr(i)));\
} while(0)

#define DMA_CH_IntEnable_NIE_LPOS 15U
#define DMA_CH_IntEnable_NIE_HPOS 15U

#define DMA_CH_IntEnable_AIE_LPOS 14U
#define DMA_CH_IntEnable_AIE_HPOS 14U

#define DMA_CH_IntEnable_CDEE_LPOS 13U
#define DMA_CH_IntEnable_CDEE_HPOS 13U

#define DMA_CH_IntEnable_FBEE_LPOS 12U
#define DMA_CH_IntEnable_FBEE_HPOS 12U

#define DMA_CH_IntEnable_ERIE_LPOS 11U
#define DMA_CH_IntEnable_ERIE_HPOS 11U

#define DMA_CH_IntEnable_ETIE_LPOS 10U
#define DMA_CH_IntEnable_ETIE_HPOS 10U

#define DMA_CH_IntEnable_RWTE_LPOS 9U
#define DMA_CH_IntEnable_RWTE_HPOS 9U

#define DMA_CH_IntEnable_RSE_LPOS 8U
#define DMA_CH_IntEnable_RSE_HPOS 8U

#define DMA_CH_IntEnable_RBUE_LPOS 7U
#define DMA_CH_IntEnable_RBUE_HPOS 7U

#define DMA_CH_IntEnable_RIE_LPOS 6U
#define DMA_CH_IntEnable_RIE_HPOS 6U

#define DMA_CH_IntEnable_TBUE_LPOS 2U
#define DMA_CH_IntEnable_TBUE_HPOS 2U

#define DMA_CH_IntEnable_TXSE_LPOS 1U
#define DMA_CH_IntEnable_TXSE_HPOS 1U

#define DMA_CH_IntEnable_TIE_LPOS 0U
#define DMA_CH_IntEnable_TIE_HPOS 0U

#define TC9562_INTMCUMASK1_RgOffAddr ((volatile uint32_t *)(TC9562_REG_BASE_ADRS + 0x8024U))

#define TC9562_INTMCUMASK1_Rd(data) do {\
  (data) = ioread32((volatile void *)(TC9562_INTMCUMASK1_RgOffAddr));\
}while(0)

#define TC9562_INTMCUMASK1_Wr(data) do {\
  iowrite32((data), (volatile void *)(TC9562_INTMCUMASK1_RgOffAddr));\
} while(0)

#define TC9562_INTMCUMASK1_MAC (uint32_t)(0xfffff00U)
#define TC9562_MAC_EVENT_MASK  (uint32_t)(0x00000400U)
#define TC9562_MAC_EVENT_SHIFT (10U)
/* Disable Tx Ch 1, 2 as only these will be handled by Standalone. Do not disable AVB Tx channel 5*/
#define TC9562_INTMCUMASK1_TxMAC_CM3 (uint32_t) (0x3000U)

#define TC9562_INTMCUMASK1_TxCH4  (uint32_t) (0x8000U)

#define TC9562_INTMCUMASK_EN_INT (uint32_t) (0x7FFF800U)
#define DMA_CHSR_RgOffAddr(i) ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0080U*(i))+0x1160U))

#define DMA_CHSR_Rd(i, data) do {\
  (data) = ioread32((volatile void *)(DMA_CHSR_RgOffAddr(i)));\
}while(0)

#define DMA_CHSR_Wr(i, data) do {\
  iowrite32((data), (volatile void *)(DMA_CHSR_RgOffAddr(i)));\
} while(0)

#define DMA_CHSR_Tx_Mask 0x7F407U
#define DMA_CHSR_Rx_Mask 0x38FBC0U

#define TC9562_MACSTATUS_RgOffAddr ((volatile uint32_t *)(TC9562_REG_BASE_ADRS + 0x800CU))

#define TC9562_MACSTATUS_Rd(data) do {\
  (data) = ioread32((volatile void *)(TC9562_MACSTATUS_RgOffAddr));\
}while(0)

#define TC9562_MACSTATUS_Wr(data) do {\
  iowrite32((data), (volatile void *)(TC9562_MACSTATUS_RgOffAddr));\
} while(0)

#define htons(n) (((((unsigned short)(n) & 0xFFU)) << 8U) | (((unsigned short)(n) & 0xFF00U) >> 8U))
#define ntohs(n) (((((unsigned short)(n) & 0xFFU)) << 8U) | (((unsigned short)(n) & 0xFF00U) >> 8U))

#define TC9562_INTMASK1_RxCH0 19


#define PTP_LOCALTIME_SNAPSHOT_SEC_OFFSET   (0x30F4U)
#define PTP_LOCALTIME_SNAPSHOT_nSEC_OFFSET  (0x30F8U)
#define MAC_STNSR_TSSS_LPOS           0U
#define MAC_STNSR_TSSS_HPOS           30U

#define NANO_SECONDS_ROLLOVER_VALUE       (1000000000U)



/**************************  PPS Register Set Access *************************************/
#define TC9562_PPSControl_RgOffAddr             (MAC_BASE_ADDRESS + 0xB70U)
#define TC9562_PPS_TargetTimeSec_RgOffAddr(i)   ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0010U*(i))+ 0x0B80U))
#define TC9562_PPS_TargetTimenSec_RgOffAddr(i)  ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0010U*(i))+ 0x0B84U))
#define TC9562_PPS_Interval_RgOffAddr(i)        ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0010U*(i))+ 0x0B88U))
#define TC9562_PPS_Width_RgOffAddr(i)           ((volatile uint32_t *)(MAC_BASE_ADDRESS + (0x0010U*(i))+ 0x0B8CU))

#define TC9562_PPSControl_Rd(data) do {\
  (data) = ioread32((volatile void *)(TC9562_PPSControl_RgOffAddr));\
}while(0)

#define TC9562_PPSControl_Wr(data) do {\
  iowrite32((data), (volatile void *)(TC9562_PPSControl_RgOffAddr));\
} while(0)

#define TC9562_TargetTimeSec_Rd(i, data) do {\
  (data) = ioread32((volatile void *)(TC9562_PPS_TargetTimeSec_RgOffAddr(i)));\
}while(0)

#define TC9562_TargetTimeSec_Wr(i, data) do {\
  iowrite32((data), (volatile void *)(TC9562_PPS_TargetTimeSec_RgOffAddr(i)));\
} while(0)

#define TC9562_TargetTimenSec_Rd(i, data) do {\
  (data) = ioread32((volatile void *)(TC9562_PPS_TargetTimenSec_RgOffAddr(i)));\
}while(0)

#define TC9562_TargetTimenSec_Wr(i, data) do {\
  iowrite32((data), (volatile void *)(TC9562_PPS_TargetTimenSec_RgOffAddr(i)));\
} while(0)

#define TC9562_Interval_Rd(i, data) do {\
  (data) = ioread32((volatile void *)(TC9562_PPS_Interval_RgOffAddr(i)));\
}while(0)

#define TC9562_Interval_Wr(i, data) do {\
  iowrite32((data), (volatile void *)(TC9562_PPS_Interval_RgOffAddr(i)));\
} while(0)

#define TC9562_Width_Rd(i, data) do {\
  (data) = ioread32((volatile void *)(TC9562_PPS_Width_RgOffAddr(i)));\
}while(0)

#define TC9562_Width_Wr(i, data) do {\
  iowrite32((data), (volatile void *)(TC9562_PPS_Width_RgOffAddr(i)));\
} while(0)




#endif //_TC9562_REGACC_H_
