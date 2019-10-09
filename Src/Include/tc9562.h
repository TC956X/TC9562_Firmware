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

#ifndef __tc9562_H__
#define __tc9562_H__

#include "tc9562_regacc.h"

/*
 *********************************************************************************************************
 *
 * Target        : TC9562
 * Filename      : tc9562.h
 *
 *********************************************************************************************************
 */

/*======================================================================
MACRO DEFINITION
======================================================================*/
/* Debugging count SRAM area start address */
#define TC9562_M3_DBG_CNT_START            0x2000F800       /* DMEM: 0x2000_0000 - 0x2001_0000 */
/* Debugging count SRAM area size */
#define TC9562_M3_DBG_CNT_SIZE             ( 18*4 )

#define TC9562_M3_DBG_VER_START            0x2000F900

/* TC9562_M3_DBG_CNT_START + 4*0:  DMA TX0
*   .........................
* TC9562_M3_DBG_CNT_START + 4*5:   DMA TX5
* TC9562_M3_DBG_CNT_START + 4*6:   DMA RX0
*   .........................
* TC9562_M3_DBG_CNT_START + 4*11:  DMA RX5
* TC9562_M3_DBG_CNT_START + 4*12:  INTC WDT Expiry Count
* TC9562_M3_DBG_CNT_START + 4*13:  MAC LPI/PWR/EVENT
* TC9562_M3_DBG_CNT_START + 4*15:  guiM3Ticks 
*/

#define INTMCUMASK1_MSI                     0xCFFFFF00U
#define INTMCUMASK2_MSI                     0x003E9FFFU
#define MAC_PMT_INTR_STS                    0x10U
#define PIPE_RESET							0x00000000U

//#define TC9562_REG_BASE						0x40000000U            
#define TC9562_EMAC_REG_BASE				0x4000A000U  /*EMAC Registers */

#define TC9562_NMODESTS_OFFS				0x0004U
#define TC9562_NEMACCTL_OFFS				0x0010U
#define TC9562_NSLEEPCTR_OFFS				0x0014U
#define TC9562_NPCIEBOOT_OFFS				0x0018U
#define TC9562_NCTLSTS_OFFS					0x1000U
#define TC9562_NCLKCTRL_OFFS				0x1004U	
#define TC9562_NRSTCTRL_OFFS				0x1008U

#define TC9562_PCIE_REG_BASE				0x40020000U  /*PCI registers  */

/* PCIe register address offsets */
/* PCIe configuration space */
#define LINK_CONTROL_2_STATUS_2_REG         0x0070U
#define MSI_SEND_TRIGGER_OFFS					      0x209CU    
#define VENDOR_ID_REG                       0x2000U
#define REVISION_ID_REG                     0x2008U
#define BAR_RANGE_SET_REG                   0x200CU
#define BAR_0_CFG_REG                       0x2010U
#define BAR_1_CFG_REG                       0x2014U
#define BAR_2_CFG_REG                       0x2018U
#define BAR_3_CFG_REG                       0x201CU
#define BAR_4_CFG_REG                       0x2020U
#define BAR_5_CFG_REG                       0x2024U
#define SUBSYSTEM_ID_REG                    0x202CU
#define LINK_CAPA_REG                       0x204CU
#define LNK_CAP2_REG                        0x206CU
#define PM_ST_CNTRL_REG                     0x2084U
#define MSI_CAPA_NXT_POINT_REG              0x2090U
#define PBSC_EP_REG                         0x20A4U
#define FUNC_READY_REG                      0x2FC0U
#define PCLK_SYNC_RST                       0x5000U
#define PIPE_RESET_SH_REG                   0x6000U
#define SYS_FREQ_REG                        0x6018U
#define PHY_CFGOUT_REG                      0x6020U
#define RANGE00_UP_OFFSET_REG               0x6200U
#define RANGE00_EN_REG                      0x6204U
#define RANGE00_UP_RPLC_REG                 0x6208U
#define RANGE00_WIDTH_REG                   0x620CU
#define PM_CTRL2_REG                        0x7104U
#define PINT0_INT_REG                       0xA000U
#define PINT0_INT_MSK_REG                   0xA004U
#define PINT1_INT_REG                       0xA010U
#define PINT1_INT_MSK_REG                   0xA014U
#define PINT2_INT_REG                       0xA020U
#define PINT2_INT_MSK_REG                   0xA024U
#define VC_VALID_INT_REG                    0xA100U
#define VC_VALID_INT_MSK_REG                0xA104U
#define MAC_INFORM_INT_REG                  0xA108U
#define MAC_INFORM_INT_MSK_REG              0xA10CU
#define RECEIVE_MSG_REG                     0xA110U
#define RECEIVE_MSG_MSK_REG                 0xA114U
#define RC_INTERNAL_INT_REG                 0xA120U
#define RC_INTERNAL_INT_MSK_REG             0xA124U
#define POWER_INFORM_INT_REG                0xA128U
#define POWER_INFORM_INT_MSK_REG            0xA12CU
#define VC0_MAXI_ERR_REG                    0xA280U
#define VC0_AXIW_ERR_REG                    0xA300U
#define VC0_AXIW_ERR_MSK_REG                0xA308U
#define VC0_AXIR_ERR_REG                    0xA310U
#define VC0_AXIR_ERR_MSK_REG                0xA318U
#define FLR_REG                             0xA400U
#define FLR_MSK_REG                         0xA404U
#define FUNCTION_READY_REG                  0xA40CU
#define PMCSR_CHANGE_REG                    0xA410U
#define PMCSR_COPY_VAL0003_REG              0xA420U
#define PMCSR_CHANGE_MSK_REG                0xA414U
#define TR_PEND_BIT_COPY_REG                0xA500U
#define TR_PEND_CHNAGE_DET_REG              0xA504U
#define TR_PEND_CHNAGE_DET_MSK_REG          0xA508U

#define PINT0_INT_MSK_VAL							0x01FF0100U
#define PINT_INTI_MSK_VAL							0x0101011DU
#define MAC_INFORM_INT_MSK_VAL				0x0000070BU
#define SYS_FREQ_VAL									0x000000BCU
#define PHY_CFGOUT_VAL								0x00000000U
#define PIPE_RESET_SH_VAL							0x00000001U	
#define PINT0_INT_VAL									0x00000002U
#define BAR_RANGE_SET_VAL							0x6C6D6B1CU
#define BAR_0_CFG_VAL									0x40000004U
#define BAR_1_CFG_VAL									0x00000000U
#define BAR_2_CFG_VAL									0x0000000CU
#define BAR_3_CFG_VAL									0x00000000U
#define BAR_4_CFG_VAL									0xC000000CU
#define BAR_5_CFG_VAL									0x00000000U
#define MAC_INFORM_INT_VAL						0x00000004U
#define PORT_INT1_VAL									0x00000002U
#define FUNC_READY_VAL								0x00000001U
#define VENDOR_ID_VAL									0x021F1179U
#define SUBSYSTEM_ID_VAL							0x00011179U
#define MSI_CAPA_NXT_POINT_VAL				0x00000000U
#define PCLK_SYNC_VAL									0x00000000U

/* M3 registers */
/* Int Ctrl'er Type Reg.          */
#define CPU_REG_NVIC_NVIC              ( *( ( uint32_t * )( 0xE000E004U ) ) )   
/* SysTick Ctrl & Status Reg.     */
#define CPU_REG_NVIC_ST_CTRL           ( *( ( uint32_t * )( 0xE000E010U ) ) ) 
/* SysTick Reload      Value Reg. */
#define CPU_REG_NVIC_ST_RELOAD         ( *( ( uint32_t * )( 0xE000E014U ) ) )
/* SysTick Current     Value Reg. */  
#define CPU_REG_NVIC_ST_CURRENT        ( *( ( uint32_t * )( 0xE000E018U ) ) )   
/* SysTick Calibration Value Reg. */
#define CPU_REG_NVIC_ST_CAL            ( *( ( uint32_t * )( 0xE000E01CU ) ) ) 
/* System Handlers 12 to 15 Prio. */  
#define CPU_REG_NVIC_SHPRI3            ( *( ( uint32_t * )( 0xE000ED20U ) ) )
/* Vect Tbl Offset Reg.           */  
#define CPU_REG_NVIC_VTOR              ( *( ( uint32_t * )( 0xE000ED08U ) ) )   
            
/* Interrupt registers */            
#define TC9562_INTC_REG_BASE           0x40008000U
#define INTSTATUS_OFFS                 0x0000U
#define GDMASTATUS_OFFS                0x0008U
#define MACSTATUS_OFFS                 0x000CU
#define TDMSTATUS_OFFS                 0x0010U
#define PCIEL12FLG_OFFS                0x0018U
#define I2CSPIINTFLG_OFFS              0x001CU
#define INTMCUMASK0_OFFS               0x0020U
#define INTMCUMASK1_OFFS               0x0024U
#define INTMCUMASK2_OFFS               0x0028U
#define INTEXTMASK0_OFFS               0x0030U
#define INTEXTMASK1_OFFS               0x0034U
#define INTEXTMASK2_OFFS               0x0038U
#define INTINTXMASK0_OFFS              0x0040U
#define INTINTXMASK1_OFFS              0x0044U
#define INTINTXMASK2_OFFS              0x0048U
#define EXTINTCFG_OFFS                 0x004CU
#define PMEINTCFG_OFFS                 0x0050U
#define MCUFLG_OFFS                    0x0054U
#define EXT_FLG_OFFS                   0x0058U
#define MACPPO_OFFS                    0x005CU
#define INTINTWDCTL_OFFS               0x0060U
#define INTINTWDEXP_OFFS               0x0064U
#define INTINTWDMON_OFFS               0x0068U
#define PMECTRL_OFFS                   0x0070U

#define WDEXP_WDEXP_MAX_MASK           (0x0FFFFFFF) /* 0xFFF_FFFF*16ns = 4.29sec */
#define WDEXP_WDEXP_DEF_MASK           (0x03FFFFFF) /* 0x3FF_FFFF*16ns = 1.07sec */
#define WDEXP_WDEXP_100MS_MASK         (0x01999999) /* 0x199_9999*16ns = 100.0msec */
#define WDCTL_WDEnable_MASK            (0x00000002)
#define WDCTL_WDRestart_MASK           (0x00000001)

/* SPI */            
#define TC9562_SPI_REG_BASE               0x40005000U
#define TC9562_SPIC_FlshMemMap0_OFFS      0x0000U
#define TC9562_SPIC_FlshMemMap1_OFFS      0x0004U
#define TC9562_SPIC_DirAccCtrl0_OFFS      0x0008U
#define TC9562_SPIC_DirAccCtrl1_OFFS      0x000CU
#define TC9562_SPIC_DirRdCtrl0_OFFS       0x0010U
#define TC9562_SPIC_DirRdCtrl1_OFFS       0x0014U
                                              
#define TC9562_SPIC_PrgAccCtrl0_OFFS      0x0400U
#define TC9562_SPIC_PrgAccCtrl1_OFFS      0x0404U
#define TC9562_SPIC_PrgAccIntEn_OFFS      0x0408U
#define TC9562_SPIC_PrgAccStat_OFFS       0x040CU

/* 0x500 - 0x507 */
#define TC9562_SPIC_PriBufDat_Start_OFFS  0x0500U
/* 0x600 - 0x6FF */
#define TC9562_SPIC_SecBufDat_Start_OFFS  0x0600U 

/* GPIO */            

#define  TC9562_GPIO_REG_BASE        ( uint32_t )( 0x40001200 ) 
#define  TC9562_GPIO_INPUT0          ( *( uint32_t * )( TC9562_GPIO_REG_BASE + 0x0000 ) )
#define  TC9562_GPIO_INPUT1          ( *( uint32_t * )( TC9562_GPIO_REG_BASE + 0x0004 ) )
#define  TC9562_GPIO_INPUT_ENABLE0   ( *( uint32_t * )( TC9562_GPIO_REG_BASE + 0x0008 ) )   /* 0-31 */
#define  TC9562_GPIO_INPUT_ENABLE1   ( *( uint32_t * )( TC9562_GPIO_REG_BASE + 0x000C ) )   /* 32-63 */
#define  TC9562_GPIO_OUTPUT0         ( *( uint32_t * )( TC9562_GPIO_REG_BASE + 0x0010 ) )
#define  TC9562_GPIO_OUTPUT1         ( *( uint32_t * )( TC9562_GPIO_REG_BASE + 0x0014 ) )


/* EMAC registers */			
#define TC9562_EMAC_PHY_BASE					0x00000001U
#define NTC_EMAC_MAC_CONFIG_OFFS				0x0000U
#define TC9562_EMAC_MAC_Ext_Config_OFFS			0x0004U
#define TC9562_EMAC_MAC_Packet_Filter_OFFS     	0x0008U
#define TC9562_EMAC_RX_Flow_Ctrl_OFFS          	0x0090U
#define TC9562_EMAC_MAC_RXQ_CTRL0_OFFS			0x00A0U
#define TC9562_EMAC_MAC_RXQ_CTRL1_OFFS			0x00A4U
#define TC9562_EMAC_MAC_RXQ_CTRL2_OFFS			0x00A8U
#define TC9562_EMAC_MAC_Interrupt_Status_OFFS  	0x00B0U
#define TC9562_EMAC_MAC_Interrupt_Enable_OFFS  	0x00B4U
#define TC9562_EMAC_MAC_PMT_Control_Sts        	0x00C0U
#define TC9562_EMAC_MAC_DEBUG_OFFS				0x0114U
#define TC9562_EMAC_MAC_HW_Feature0_OFFS		0x011CU
#define TC9562_EMAC_MDIO_Address_OFFS			0x0200U
#define TC9562_EMAC_MAC_Address0_High_OFFS 		0x0300U
#define TC9562_EMAC_MAC_Address0_Low_OFFS   	0x0304U
#define TC9562_EMAC_MAC_Address1_High_OFFS		0x0308U
#define TC9562_EMAC_MAC_Address1_Low_OFFS		0x030CU
#define TC9562_EMAC_MAC_Address2_High_OFFS		0x0310U
#define TC9562_EMAC_MAC_Address2_Low_OFFS		0x0314U
#define TC9562_EMAC_MTL_TxQ0_OFFS				0x0D08U
#define TC9562_EMAC_MTL_RxQ0_OFFS				0x0D38U
#define TC9562_EMAC_DMA0_CH0_Tx_OFFS           	0x1104U

/* PINT0 Interrupts */
#define PINT0_ST_PCLK_VALID                 0x00000001U
#define PINT0_ST_PCLK_VALKD_CHG             0x00000002U
#define PINT0_ST_LINK_STATE_CHG             0x00000100U
#define PINT0_ST_LINK_STATE_CHG_0_LD_CHG    0x00010000U
#define PINT0_ST_LINK_STATE_CHG_1_L0_CHG    0x00020000U
#define PINT0_ST_LINK_STATE_CHG_2_L1_CHG    0x00040000U
#define PINT0_ST_LINK_STATE_CHG_3_L1_CHG    0x00080000U
#define PINT0_ST_LINK_STATE_CHG_4_L2_CHG    0x00100000U
#define PINT0_ST_LINK_STATE_CHG_5_L12ENT    0x00200000U
#define PINT0_ST_LINK_STATE_CHG_6_L12IDLE   0x00400000U
#define PINT0_ST_ASPM_L1ENTER_READY         0x01000000U

/* PINT1 interrupts */
#define PINT1_ST_VC_VALID_INT_RES           0x00000001U
#define PINT1_ST_MAC_INFORM_INT_RES         0x00000002U
#define PINT1_ST_RECEIVE_MSG_INT_RES        0x00000004U
#define PINT1_ST_POWER_INFORM_INT           0x00000010U
#define PINT1_ST_VC0_MAXI_ERR_DET_RES       0x00010000U
#define PINT1_ST_VC0_SAXI_ERR_DET_RES       0x01000000U

/* PINT2 interrupts */
#define PINT2_ST_FLR_INT_RES                0x00000001U
#define PINT2_ST_PMCSR_CHANGE_INT_RES       0x00000008U
#define PINT2_ST_TR_PND_CHG_INT_RES         0x00000010U

/* MAC interrupts */
#define MAC_INFORM_ML_PA_LINKDOWN           0x00000001U
#define MAC_INFORM_ML_PA_DISABLED           0x00000002U
#define MAC_INFORM_ML_PA_HOTRESET           0x00000004U
#define MAC_INFORM_ML_PA_EXITHOTRESET       0x00000008U
#define MAC_INFORM_ML_PA_RATECHGRQ          0x00000400U

/* Power interrupts */
#define PME_TURN_OFF_RCV_ST                 0x00000008U
    
/* Config Register */
#define TC9562_NCLKCTRL_INTCEN_LPOS						(4)
#define TC9562_NRSTCTRL_INTRST_LPOS						(4)
#define TC9562_NRSTCTRL_PCIRST_LPOS						(9)
#define TC9562_NRSTCTRL_MSIGENRST_LPOS				(18)

/* PCIE Boot Register */
#define TC9562_NPCIEBOOT_PCIE_RST_LPOS				(15)
#define TC9562_NPCIEBOOT_PCIE_HW_BT_ST_LPOS		(1)
#define TC9562_NPCIEBOOT_PCIE_HW_BT_ST_HPOS		(3)
#define TC9562_NPCIEBOOT_PCIE_HW_BT_ST_MASK		(0xE)
#define TC9562_NPCIEBOOT_PCIE_HW_SEQ_IDLE			(0x0)
#define TC9562_NPCIEBOOT_PCIE_HW_SEQ_WHRST		(0x5)	

/* MCU Flags */
#define TC9562_MCUFLAG_PR_RE									(14)
#define TC9562_MCUFLAG_PR_FE									(13)
#define TC9562_MCUFLAG_PR_RST_MON							(15)

#define TC9562_MCUFLAG_PR_RE_VAL							(0x4000)
#define TC9562_MCUFLAG_PR_FE_VAL							(0x2000)


#define TC9562_PME_EN_BIT_POS                               (0)

#define TC9562_INTMCUMASK2_PCIE_MSI						(0x1F00)
#define TC9562_INTEXTMASK2_PCIE_MSI						(0x1F00)
#define TC9562_INTINTXMASK2_PCIE_MSI						(0x1F00)

/*=====================================================================
ENUMERATION
======================================================================*/
/** Interrupt Number Definition */
typedef enum IRQn
{
/******  Cortex-M3 Specific Interrupt Numbers ***********************************************************************/
    INT_SRC_NMI                 = -14,  /* Non Maskable                */
    INT_SRC_Hard_Fault          = -13,  /* Cortex-M3 Hard Fault        */     
    INT_SRC_Memory_Management   = -12,  /* Cortex-M3 Memory Management */     
    INT_SRC_Bus_Fault           = -11,  /* Cortex-M3 Bus Fault         */     
    INT_SRC_Usage_Fault         = -10,  /* Cortex-M3 Usage Fault       */     
    INT_SRC_SVC                 = -5,   /* Cortex-M3 SV Call           */     
    INT_SRC_Debug_Monitor       = -4,   /* Cortex-M3 Debug Monitor     */     
    INT_SRC_Pend_SV             = -2,   /* Cortex-M3 Pend SV           */
    SysTick_IRQn                = -1,   /* Cortex-M3 System Tick       */

/* **** TC9562 Specific Interrupt Numbers *************************/
    INT_SRC_NBR_INTIO1			= 0U,   /* Interrupt Pin 0. */
    INT_SRC_NBR_INTIO2          = 1U,   /* Interrupt Pin 1. */
    INT_SRC_NBR_INTIO3          = 2U,   /* Interrupt Pin 2. */
    INT_SRC_NBR_EXT_INT         = 3U,   /* External Input Interrupt Pin */
    INT_SRC_NBR_I2C_SLAVE       = 4U,   /* I2C slave interrupt */
    INT_SRC_NBR_I2C_MASTER      = 5U,   /* I2C mater interrupt */
    INT_SRC_NBR_SPI_SLAVE       = 6U,   /* SPI slave interrupt */
    INT_SRC_NBR_QSPI            = 7U,   /* QSPI */

    INT_SRC_NBR_MAC_LPI_EXIT    = 8U,   /* eMAC LPI exit */
    INT_SRC_NBR_MAC_POWER       = 9U,   /* eMAC power management */
    INT_SRC_NBR_MAC_EVENTS      = 10U,  /* eMAC event from LPI, GRMII, Management counter */

    INT_SRC_NBR_EMACTXDMA0      = 11U,  /* eMAC Tx DMA Chnl 0 traffic */
    INT_SRC_NBR_EMACTXDMA1      = 12U,  /* eMAC Tx DMA Chnl 1 traffic */
    INT_SRC_NBR_EMACTXDMA2      = 13U,  /* eMAC Tx DMA Chnl 2 traffic */
    INT_SRC_NBR_EMACTXDMA3      = 14U,  /* eMAC Tx DMA Chnl 3 traffic */
    INT_SRC_NBR_EMACTXDMA4      = 15U,  /* eMAC Tx DMA Chnl 4 traffic */
    INT_SRC_NBR_EMACTXDMA5      = 16U,  /* eMAC Tx DMA Chnl 5 traffic */
    INT_SRC_NBR_EMACTXDMA6      = 17U,  /* eMAC Tx DMA Chnl 6 traffic */
    INT_SRC_NBR_EMACTXDMA7      = 18U,  /* eMAC Tx DMA Chnl 7 traffic */
    
    INT_SRC_NBR_EMACRXDMA0      = 19U,  /* eMAC Rx DMA Chnl 0.  */
    INT_SRC_NBR_EMACRXDMA1      = 20U,  /* eMAC Rx DMA Chnl 1.  */
    INT_SRC_NBR_EMACRXDMA2      = 21U,  /* eMAC Rx DMA Chnl 2.  */
    INT_SRC_NBR_EMACRXDMA3      = 22U,  /* eMAC Rx DMA Chnl 3.  */
    INT_SRC_NBR_EMACRXDMA4      = 23U,  /* eMAC Rx DMA Chnl 4.  */
    INT_SRC_NBR_EMACRXDMA5      = 24U,  /* eMAC Rx DMA Chnl 5.  */
    INT_SRC_NBR_EMACRXDMA6      = 25U,  /* eMAC Rx DMA Chnl 6.  */
    INT_SRC_NBR_EMACRXDMA7      = 26U,  /* eMAC Rx DMA Chnl 7.  */

    INT_SRC_NBR_EMACPPO		    = 27U,  /* MAC PPO. */
    INT_SRC_NBR_GDMA0			= 30U,  /* GDMA Channel 0. */
    INT_SRC_NBR_GDMA1			= 31U,  /* GDMA Channel 1. */
    INT_SRC_NBR_GDMA2			= 32U,  /* GDMA Channel 2. */
    INT_SRC_NBR_GDMA3		    = 33U,  /* GDMA Channel 3.  */
    INT_SRC_NBR_GDMA4			= 34U,  /* GDMA Channel 4. */
    INT_SRC_NBR_GDMA5			= 35U,  /* GDMA Channel 5. */
                                
    INT_SRC_NBR_GDMAGEN         = 36U,  /* qSPI interrupt. */
    INT_SRC_NBR_SHA             = 37U,  /* SHA. */
    INT_SRC_NBR_UART0           = 38U,  /* UART0. */
    INT_SRC_NBR_UART1		    = 39U,  /* UART1. */
    INT_SRC_NBR_MSIGEN 		    = 40U,  /* MSIGEN. */
    INT_SRC_NBR_PCIEC0          = 41U,  /* PCIe Controller Int0. */
    INT_SRC_NBR_PCIEC1          = 42U,  /* PCIe Controller Int1. */
    INT_SRC_NBR_PCIEC2          = 43U,  /* PCIe Controller Int2. */
    INT_SRC_NBR_PCIE_L12        = 44U,  /* PCIe L12 Int */
    INT_SRC_NBR_MCU_FLAG        = 45U,  /* MCU FLAG Int */
    INT_SRC_NBR_WDT             = 48U,  /*  INTC WatchDog timer */
    INT_SRC_NBR_TDM0  		    = 49U,  /* TDM stream 0 input error */
    INT_SRC_NBR_TDM1  		    = 50U,  /* TDM stream 1 input error */
    INT_SRC_NBR_TDM2  		    = 51U,  /* TDM stream 1 input error */
    INT_SRC_NBR_TDM3  		    = 52U,  /* TDM stream 1 input error */
    INT_SRC_NBR_TDMOUT  	    = 53U,  /* TDM output error */
} IRQn_Type;

typedef struct FW_Version_s
{
	unsigned char rel_dbg;  // "R for Release, D for debug"
	unsigned char major;
	unsigned char minor;
	unsigned char sub_minor;	
}FW_Version_t;

/*Processor and Core Peripheral Section */

/* Configuration of the Cortex-M3 Processor and Core Peripherals */
#define __CM3_REV               0x0200  /* Cortex-M3 Core Revision */
#define __MPU_PRESENT           0       /* MPU present or not      */
#define __NVIC_PRIO_BITS        3       /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig  0       /* Set to 1 if different SysTick Config is used */

#include "core_cm3.h"                   /* Cortex-M3 processor and core peripherals */

#endif /* _TC9562_H__ */

