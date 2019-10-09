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
 * Filename      : main.c
 *
 *********************************************************************************************************
 */

/*=====================================================================
INCLUDE FILES
===================================================================== */
#include "common.h"
#include "tc9562_uart_driver.h"
#include "tc9562_gpio_driver.h"

/*======================================================================
GLOBAL VARIABLE
======================================================================*/
volatile uint64_t guiM3Ticks = TC9562_VAL_ZERO ;    /* System ticks */

/*======================================================================
MACRO DEFINITION
======================================================================*/
#define INT_MCUMASK0_MASK           0xFFFFFFF0U
#define INT_MCUMASK_MASK_ALL        0xFFFFFFFFU
#define INT_MCUMASK2_MASK           0xFFFFF1BFU
#define EFUSE_ADDR                  0x0000C000U
//#define TC9562_CLK_CTRL_ENABLE      0x0773FAB5U
#define TC9562_UART0_ENABLE         0x11U
#define CPU_FREQ                    187500000U
#define SYS_TICK_PRIO               0x00E00000U
#define TC9562_GPIO0_REG            0x0000        
#define TC9562_PMT_INT_CONFIG       0x0F00000FU   

#define TC9562_GPIO3_OUTPUT         DEF_DISABLED  

static const FW_Version_t version = {'R', 1, 0, 0};

/*=====================================================================
FUNCTION PROTOTYPES
==================================================================== */
static void delay( const uint32_t uiMs );
static void Enable_Interrupts_for_WOL(void) ;
static void Configure_PME(void) ;
static void Enter_into_Low_Power_Mode(void) ;
/*=====================================================================
FUNCTION DEFINITION
==================================================================== */
/*
*    Function    :    delay( uiMS )
*    Purpose     :    Delay by uiMS time
*    Inputs      :    uiMS
*    Outputs     :    None
*    Return Value:    None
*    Limitations :    None 
*/
static void delay( const uint32_t uiMs )
{
  uint64_t ticks = guiM3Ticks + uiMs ;
  uint64_t uiCurrentTicks = guiM3Ticks ;

  while ( uiCurrentTicks < ticks )
  {
    uiCurrentTicks = guiM3Ticks;
  }
  return;
}
/* End of delay() */

/*  Function     :    Hw_Reg_Write32( uiAddr_Base, uiOffset, uiVal )
*    Purpose     :    write data to an address
*    Inputs      :    uiAddr_Base - base address to write
*    Inputs      :    uiOffset - offset from base address
*    Inputs      :    uiVal - value to write
*    Outputs     :    None
*    Return Value:    None
*    Limitations :    None 
*/
void Hw_Reg_Write32 ( const uint32_t uiAddr_base, const uint32_t uiOffset,
                      const uint32_t uiVal )
{
  volatile uint32_t *puiAddr = ( volatile uint32_t * )
                               ( uint32_t * )( uiAddr_base + uiOffset ) ;
  
  if ( NULL != puiAddr )
  {        
    *puiAddr = uiVal;
  }
}
/* End of Hw_Reg_Write32() */

/*
*    Function    :    Hw_Reg_Read32( uiAddr_Base, uiOffset )
*    Purpose     :    Read data from an address
*    Inputs      :    uiAddr_Base - base address to read from
*    Inputs      :    uiOffset - offset from base address
*    Outputs     :    None
*    Return Value:    puiAddr - Pointer to read data
*    Limitations :    None 
*/
uint32_t Hw_Reg_Read32 ( const uint32_t uiAddr_base,
                         const uint32_t uiOffset )
{
  uint32_t            uiVal  = 0 ;
  volatile uint32_t  *puiAddr = ( volatile uint32_t *)
                               ( uint32_t *)( uiAddr_base + uiOffset ) ;

  if ( NULL != puiAddr )
  {
    uiVal = *puiAddr ;
  }
  return ( uiVal ) ;     /* Returns address */
}
/* End of Hw_Reg_Read32() */

/*
*    Function    :    SysTick_Handler( )
*    Purpose     :    System Tick Handler
*    Inputs      :    None
*    Outputs     :    None
*    Return Value:    None
*    Limitation  :    None
*/
void SysTick_Handler( void )
{
  guiM3Ticks++ ;
  *(uint32_t*)( TC9562_M3_DBG_CNT_START + ( TC9562_FIFTEEN * TC9562_FOUR ) ) = guiM3Ticks ;
  
#if ( DEF_ENABLED == TC9562_GPIO3_OUTPUT )
  if (guiM3Ticks & 1) 
  {
    tc9562_gpio.OutputData(GPIO_THREE, GPIO_HIGH);
  }
  else  
  {
    tc9562_gpio.OutputData(GPIO_THREE, GPIO_LOW);
  }
#endif
}
/* End of SysTick_Handler */

/*
*    Function    :    SysInit( )
*    Purpose     :    System Initialization
*    Inputs      :    None
*    Outputs     :    None
*    Return Value:    None
*    Limitation  :    None
*/
void SysInit( void )
{
  uint32_t uiCount ;    /* CPU_FREQ ticks : cpu_freq/1000 for 1ms */
  uint32_t uiData ;     /* stores Register value */
  
  /* Enable all clocks (MCU, UART, eMAC ...) except reserved bits and MSIGEN module */
  Hw_Reg_Write32 ( TC9562_REG_BASE, TC9562_NCLKCTRL_OFFS, TC9562_CLK_CTRL_ENABLE ) ;
  /* Reset is de-asserted (MCU, UART, eMAC ...) */
  Hw_Reg_Write32 ( TC9562_REG_BASE, TC9562_NRSTCTRL_OFFS, TC9562_VAL_ZERO ) ;
  
  /* Enable UART0 : SIN0, SOUT0 (Function 1)   */
  uiData = Hw_Reg_Read32 ( TC9562_REG_BASE, TC9562_NFUNCEN5_OFFS ) ;
  uiData |= (TC9562_UART0_ENABLE);
  Hw_Reg_Write32 ( TC9562_REG_BASE, TC9562_NFUNCEN5_OFFS, uiData ) ; 

#if ( DEF_ENABLED == TC9562_GPIO3_OUTPUT )
  tc9562_gpio.ConfigOutput(GPIO_THREE); /* config GPIO0 for output */
#endif

 
#if ( DEF_ENABLED == TC9562_M3_MASK_INT_IN_MSIGEN )
  /* Enable INTIO1, INTIO2, INTIO3, INT_I, and mask I2C_S, I2C_M, SPI_S, qSPI */
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTMCUMASK0_OFFS, INT_MCUMASK0_MASK ) ;
  /* Mask all MAC_LPI, MAC_PM, MAC_EVENT, MAC_TX_0_7, MAC_RX_0_7, PPO, GDMA_0_1 */
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTMCUMASK1_OFFS, INT_MCUMASK_MASK_ALL ) ;
  /* Enable UART0, PCIe Controller and Mask GDMA_2_5, GDMA_general, SHA, UART1, MSIGEN, PCIe_L12, WDT, TDM_I_0_3, TDM_O  */
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTMCUMASK2_OFFS, INT_MCUMASK2_MASK ) ;
#endif

  /* Mask all External interrupts */
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTEXTMASK1_OFFS, INT_MCUMASK_MASK_ALL ) ;
  /* Mask all INTX interrupt */
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTINTXMASK1_OFFS, INT_MCUMASK_MASK_ALL ) ;
  
  /* Vector interrupt from external host to MCU */    
  uiData = Hw_Reg_Read32( TC9562_INTC_REG_BASE, MCUFLG_OFFS ) ;
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, MCUFLG_OFFS, uiData ) ;

  /* CPU_FREQ ticks : cpu_freq/1000 for 1ms */
  uiCount = ( CPU_FREQ / TC9562_THOUSAND ) ;
  CPU_REG_NVIC_ST_RELOAD = ( uiCount - TC9562_ONE ) ;
  
  /* Set SysTick handler */    
  CPU_REG_NVIC_SHPRI3 = SYS_TICK_PRIO ;
  /* Enable timer.*/    
  CPU_REG_NVIC_ST_CTRL |= ( TC9562_COMMON_FOUR | TC9562_COMMON_ONE ) ;
  /* Enable timer interrupt.*/
  CPU_REG_NVIC_ST_CTRL |= TC9562_COMMON_TWO ;
  /* Enable WatchDog Timer */
  uiData = Hw_Reg_Read32( TC9562_INTC_REG_BASE, INTINTWDEXP_OFFS ) ;    
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTINTWDEXP_OFFS, 
                  ( uiData | ( WDEXP_WDEXP_MAX_MASK ) ) ) ;

  uiData = Hw_Reg_Read32( TC9562_INTC_REG_BASE, INTINTWDCTL_OFFS ) ;
  Hw_Reg_Write32( TC9562_INTC_REG_BASE, INTINTWDCTL_OFFS, ( uiData | 
                  ( WDCTL_WDEnable_MASK ) | ( WDCTL_WDRestart_MASK ) ) ) ;
  
  CPU_IntEn( ) ;
}
/* End of SysInit */

/**
 * @brief Enable Interrupts for WOL (PME_TURN_OFF Event)
 * @param  None
 * @return None
 */
static void Enable_Interrupts_for_WOL(void)
{
  uint32_t data;
  
  /* Unmask POWER_INFORM_INT_RES in PORT_INT1 regsiter */
  data = Hw_Reg_Read32(TC9562_PCIE_REG_BASE, PINT1_INT_MSK_REG);
  data &= ~(PINT1_ST_POWER_INFORM_INT);   /* enable POWER_INFORM_INT */
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, PINT1_INT_MSK_REG, data);  
  /* Enable PME_TURN_OFF Event Interrupt */
  Hw_Reg_Write32(TC9562_PCIE_REG_BASE, POWER_INFORM_INT_MSK_REG, ~PME_TURN_OFF_RCV_ST);  
    
  /* Enable the PCIe Interrupt */
  NVIC_EnableIRQ(INT_SRC_NBR_PCIEC1);
}
/* End of Enable_Interrupts_for_WOL */

/**
 * @brief PME Configuration
 * @param  None
 * @return None
 */
 
static void Configure_PME(void)
{
  /* pme_mac_msk (BIT4)=0; PME (mac_pmt_intr_o) from mac will trigger PME pin to be asserted
     pme_o_pls (BIT16)=0; level signal
     pme_o_lvl (BIT17)=0; active low level signal
  */
  uint32_t data;
  data = Hw_Reg_Read32(TC9562_INTC_REG_BASE, PMECTRL_OFFS);

  if((data & (1 << TC9562_PME_EN_BIT_POS)) == 0x1)
  {
    data &= ~(1 << TC9562_PME_EN_BIT_POS);
    Hw_Reg_Write32(TC9562_INTC_REG_BASE, PMECTRL_OFFS , data);
  }  

  Hw_Reg_Write32(TC9562_INTC_REG_BASE, PMEINTCFG_OFFS , TC9562_PMT_INT_CONFIG);
  
  data = Hw_Reg_Read32(TC9562_INTC_REG_BASE, PMECTRL_OFFS);
    data |= (1 << TC9562_PME_EN_BIT_POS);  
  Hw_Reg_Write32(TC9562_INTC_REG_BASE, PMECTRL_OFFS , data);
}
/* End of Configure_PME */

/**
 * @brief Entering into low power state
 * @param  None
 * @return None
 */
static void Enter_into_Low_Power_Mode(void)
{  
  /* Start EMAC Power Down Sequence */
  TC9562_emac_power_dwn_seq();
}
/* End of Enter_into_Low_Power_Mode */

/*
*    Function    :    main( )
*    Purpose     :    The standard entry point for C code. It is assumed
*                     that your code will call main() once you have 
*                     performed all necessary initialization.
*    Inputs      :    None
*    Outputs     :    None
*    Return Value:    None
*    Limitation  :    None
*/
int32_t main ( void )
{
  int32_t iCount ;  /* Iteration count */
  char ver_str[32] ;
  
#ifdef TC9562_SWITCH_HW_TO_SW_SEQ
  uint32_t  uiItr = 0;
  uint32_t uiData ;                /*Stores registers values */
  volatile uint32_t uiData2 ;               /*Stores registers values */
#endif    
      
  for ( iCount = TC9562_ZERO; iCount < ( TC9562_M3_DBG_CNT_SIZE / TC9562_FOUR ); iCount++ ) 
  {        
    /* clear the count */
    *( uint32_t*)( TC9562_M3_DBG_CNT_START + ( iCount * TC9562_FOUR ) ) = TC9562_ZERO ;
  }

  *( uint32_t*)(TC9562_M3_DBG_VER_START) = *( uint32_t *)(&version);
  sprintf(ver_str, "Version %s_%d.%d-%d", (version.rel_dbg == 'R')?"REL":"DBG", 
              version.major, version.minor, version.sub_minor);
  
#if ( DEF_ENABLED == TC9562_GPIO3_OUTPUT )
  /* Initialize GPIO */
  tc9562_gpio.Initialize();
#endif  
  /* System Init function */
  SysInit( ) ;
 
  /**  UART0 Init  **/ 
  Driver_USART0.Initialize(NULL);
  
  
  Enable_Interrupts_for_WOL(); /* Enable Power Management Interrupts */
  Configure_PME(); /* PME Configuration */
    
  /* Assign function pointer to the callback function for PME_Turn_Off event */
  cb_pcie_enter_low_states = Enter_into_Low_Power_Mode;
  
#ifdef TC9562_SWITCH_HW_TO_SW_SEQ
  /* Switch to SW Sequencer */
  
  /* Enable Clock for INTC */
  uiData = Hw_Reg_Read32 ( TC9562_REG_BASE, TC9562_NCLKCTRL_OFFS ) ;
  uiData |= (1 << TC9562_NCLKCTRL_INTCEN_LPOS);
  Hw_Reg_Write32 ( TC9562_REG_BASE, TC9562_NCLKCTRL_OFFS, uiData) ;
  
  /* Reset INTC */
  uiData = Hw_Reg_Read32 ( TC9562_REG_BASE, TC9562_NRSTCTRL_OFFS ) ;
  uiData &= (~(1 << TC9562_NRSTCTRL_INTRST_LPOS));
  Hw_Reg_Write32(TC9562_REG_BASE, TC9562_NRSTCTRL_OFFS, uiData);
  
  /* Disable HW Sequencer */
  uiData = Hw_Reg_Read32 ( TC9562_REG_BASE, TC9562_NPCIEBOOT_OFFS ) ;
  uiData |= (1 << TC9562_NPCIEBOOT_PCIE_RST_LPOS);
  Hw_Reg_Write32 ( TC9562_REG_BASE, TC9562_NPCIEBOOT_OFFS, uiData) ;
  
  /* Do dummy read */
  while(uiItr < 10)
  {
    uiData2 = Hw_Reg_Read32 ( TC9562_REG_BASE, TC9562_NPCIEBOOT_OFFS ) ;
    uiItr++;
  }
  
  do {
    uiData2 = Hw_Reg_Read32 ( TC9562_REG_BASE, TC9562_NPCIEBOOT_OFFS ) ;
    uiData2 &= TC9562_NPCIEBOOT_PCIE_HW_BT_ST_MASK;
    uiData2 = uiData2 >> TC9562_NPCIEBOOT_PCIE_HW_BT_ST_LPOS;
  }while((!((uiData2 == TC9562_NPCIEBOOT_PCIE_HW_SEQ_IDLE) || (uiData2 == TC9562_NPCIEBOOT_PCIE_HW_SEQ_WHRST))));
  
  TC9562_PCIE_Init_SWSeq();
  TC9562_Ser_Printf( "\r\nTC9562 switched to SW Sequencer\r\n" ) ;
#endif
  
  /* Enable EMAC/DMA interrupts */
  for ( iCount = INT_SRC_NBR_MAC_LPI_EXIT ;  
        iCount <= INT_SRC_NBR_EMACPPO ; iCount++ )
  {
#if ( DEF_DISABLED == TC9562_DISABLE_MAC_EVENTS_INT )
    if ( iCount != INT_SRC_NBR_MAC_EVENTS )    
#endif
    {
      NVIC_EnableIRQ( ( IRQn_Type )iCount ) ;
    }
  }
 
  /*Enable WatchDog Timer Interrupt */
  NVIC_EnableIRQ( ( IRQn_Type )INT_SRC_NBR_WDT ) ;

  TC9562_Ser_Printf( "\r\nTC9562 PCIe Firmware %s is Loaded Successfully and running on Cortex M3\r\n", ver_str ) ;
  
  Hw_Reg_Write32(TC9562_EMAC_REG_BASE, TC9562_NCTLSTS_OFFS, 1);
  
  while ( TC9562_ONE ) 
  { 
    /* about 10ms */
    delay( TC9562_TEN ) ;
  }
}
/* End of main */
