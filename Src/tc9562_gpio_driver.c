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
 * Filename      : tc9562_gpio_driver.c
 *
 *********************************************************************************************************
 */

/*====================================================================
INCLUDE FILES
==================================================================== */
#include "tc9562_gpio_driver.h"
#include "tc9562_intc_driver.h"

static int32_t gpio_init;
static int8_t gpio_dir[36];
static Enable_gpio0 gpio0_mem, *gpio0;
static Enable_gpio1 gpio1_mem, *gpio1;


/*uint32_t pinMuxC7;*/

ARM_DRIVER_VERSION tc9562_GPIO_GetVersion (void)
{
  /* Driver Version */
  static ARM_DRIVER_VERSION DriverVersion = {
    ARM_GPIO_API_VERSION,
    ARM_GPIO_DRV_VERSION
  };
  return DriverVersion;
}

/**
  \fn          int8_t tc9562_GPIO_Initialize (void)
  \brief       Initialize GPIO Driver
  \return      \ref execution_status
*/
int8_t tc9562_GPIO_Initialize (void)
{
  static reg_pinMuxCtrl0 *pinMux0;
  static reg_pinMuxCtrl1 *pinMux1;
  static reg_pinMuxCtrl2 *pinMux2;
  static reg_pinMuxCtrl3 *pinMux3;
  static reg_pinMuxCtrl4 *pinMux4;
  static reg_pinMuxCtrl5 *pinMux5;
  static reg_pinMuxCtrl6 *pinMux6;
  static uint32_t pinMuxC1;
  static uint32_t pinMuxC2;
  static uint32_t pinMuxC3;
  static uint32_t pinMuxC4;
  static uint32_t pinMuxC5;
  static uint32_t pinMuxC6;
  static uint32_t pinMuxC0;

  if (gpio_init == 1)
  {
    return ARM_DRIVER_ERROR;
  }
  /*Read the pinMux value*/
  pinMuxC1 = hw_reg_read32(TC9562_REG_BASE_ADRS, TC9562_NFUNCEN1_OFFS);
  pinMuxC2 = hw_reg_read32(TC9562_REG_BASE_ADRS, TC9562_NFUNCEN2_OFFS);
  pinMuxC3 = hw_reg_read32(TC9562_REG_BASE_ADRS, TC9562_NFUNCEN3_OFFS);
  pinMuxC4 = hw_reg_read32(TC9562_REG_BASE_ADRS, TC9562_NFUNCEN4_OFFS);
  pinMuxC5 = hw_reg_read32(TC9562_REG_BASE_ADRS, TC9562_NFUNCEN5_OFFS);
  pinMuxC6 = hw_reg_read32(TC9562_REG_BASE_ADRS, TC9562_NFUNCEN6_OFFS);
  pinMuxC0 = hw_reg_read32(TC9562_REG_BASE_ADRS, TC9562_NFUNCEN0_OFFS);

  /* Store pinMux value to Structures */
  pinMux1 = (reg_pinMuxCtrl1 *)&(pinMuxC1);
  pinMux2 = (reg_pinMuxCtrl2 *)&(pinMuxC2);
  pinMux3 = (reg_pinMuxCtrl3 *)&(pinMuxC3);
  pinMux4 = (reg_pinMuxCtrl4 *)&(pinMuxC4);
  pinMux5 = (reg_pinMuxCtrl5 *)&(pinMuxC5);
  pinMux6 = (reg_pinMuxCtrl6 *)&(pinMuxC6);
  pinMux0 = (reg_pinMuxCtrl0 *)&(pinMuxC0);

  gpio0 = &gpio0_mem;
  gpio1 = &gpio1_mem;
  if (pinMux4->GPIO0 == TC9562_TWO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_ZERO);
  }
  if (pinMux4->GPIO1 == TC9562_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_ONE);
  }
  if (pinMux4->GPIO2 == TC9562_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_TWO);
  }
  if (pinMux4->GPIO3 == TC9562_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_THREE);
  }
  if (pinMux4->GPIO4 == TC9562_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_FOUR);
  }
  if (pinMux4->GPIO5 == TC9562_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_FIVE);
  }
  if (pinMux4->GPIO6 == TC9562_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_SIX);
  }
  if (pinMux6->INT0 == TC9562_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_NINE);
  }
  if (pinMux5->SIN0 == TC9562_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_TEN);
  }
  if (pinMux5->SOUT0 == TC9562_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_ELEVEN);
  }
  if (pinMux5->SIN1 == TC9562_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_TWELVE);
  }
  if (pinMux5->SOUT1 == TC9562_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_THIRTEEN);
  }
  if (pinMux3->I2S_TDM_CLK == TC9562_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_TWENTYNINE);
  }
  if (pinMux3->I2S_TDM_LR == TC9562_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_THIRTY);
  }
  if (pinMux3->I2S_TDM_D0 == TC9562_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (0x1U << GPIO_THIRTYONE);
  }
  if (pinMux3->I2S_TDM_D1 == TC9562_ZERO)
  {
    gpio1->gpio_pins1 = gpio1->gpio_pins1 | (TC9562_ONE << GPIO_ZERO); /* 32 */
  }
  if (pinMux3->I2S_TDM_D2 == TC9562_ZERO)
  {
    gpio1->gpio_pins1 = gpio1->gpio_pins1 | (TC9562_ONE << GPIO_ONE); /* 33 */
  }
  if (pinMux3->I2S_TDM_D3 == TC9562_ZERO)
  {
    gpio1->gpio_pins1 = gpio1->gpio_pins1 | (TC9562_ONE << GPIO_TWO); /* 34 */
  }
  if (pinMux0->JTAG == TC9562_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (0x7C180);/* gpio pins 7, 8, 14, 15, 16, 17, 18 */
  }
  if (pinMux2->SS0_CLK == TC9562_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_NINETEEN);
  }
  if (pinMux2->SS0_IO0 == TC9562_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_TWENTY);
  }
  if (pinMux2->SS0_IO1 == TC9562_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_TWENTYONE);
  }
  if (pinMux2->SS0_IO2 == TC9562_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_TWENTYTWO);
  }
  if (pinMux2->SS0_IO3 == TC9562_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_TWENTYTHREE);
  }
  if (pinMux2->SS0_N == TC9562_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_TWENTYFOUR);
  }
  if (pinMux1->MII_TXER == TC9562_ZERO)
  {
    gpio1->gpio_pins1 = gpio1->gpio_pins1 | (TC9562_ONE << GPIO_THREE); /* 35 */
  }
  if (pinMux1->MII_RXER == TC9562_ZERO)
  {
    gpio1->gpio_pins1 = ((gpio1->gpio_pins1) | (TC9562_ONE << GPIO_FOUR)); /* 36 */
  }
  if (pinMux0->SPIO_CLK == TC9562_TWO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_TWENTYEIGHT);
  }
  if (pinMux0->SPIO_MOSI == TC9562_TWO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_TWENTYSEVEN);
  }
  if (pinMux0->SPIO_SS_N == TC9562_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_TWENTYFIVE);
  }
  if (pinMux0->SPIO_MISO == TC9562_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC9562_ONE << GPIO_TWENTYSIX);
  }
  gpio_init = 1;
  return ARM_DRIVER_OK;
}

/**
  \fn          int8_t tc9562_GPIO_ConfigOutput(uint8_t gpio_pin)
  \brief       Config the gpio pin as output pin
  \param[in]   gpio_pin [0-36]
  \return      \ref execution_status
*/
int8_t tc9562_GPIO_ConfigOutput(uint8_t gpio_pin)
{
  uint32_t config;

  if (gpio_init == 0)
  {
    return ARM_DRIVER_ERROR;
  }
  if (gpio_pin < 32)
  {
    if ((((gpio0->gpio_pins0) >> gpio_pin) & 0x1) == 0)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else if (gpio_pin < 37)
  {
    if ((((gpio1->gpio_pins1) >> (gpio_pin - 32)) & 0x1) == 0)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if(gpio_pin < GPIO_THIRTYTWO)
  {
    config = ~(0x1U << gpio_pin) ;
    hw_reg_write32(TC9562_REG_BASE_ADRS, GPIO0_ENABLE, (hw_reg_read32 (TC9562_REG_BASE_ADRS, GPIO0_ENABLE)) & config);
  } else
  {
    config = ~(0x1U << (gpio_pin - GPIO_THIRTYTWO)) ;
    hw_reg_write32(TC9562_REG_BASE_ADRS, GPIO1_ENABLE, (hw_reg_read32 (TC9562_REG_BASE_ADRS, GPIO1_ENABLE)) & config);
  }
  gpio_dir[gpio_pin] = 1;
  return (int8_t)ARM_DRIVER_OK;
}

/**
  \fn          int8_t tc9562_GPIO_OutputData(uint8_t  gpio_pin, uint8_t flag)
  \brief       Write the output value in gpio pin
  \param [in]  gpio_pin [0-36]
  \param [in]  flag HIGH/LOW
  \return      \ref execution_status
*/
int8_t tc9562_GPIO_OutputData(uint8_t  gpio_pin, uint8_t flag)
{
  uint8_t status;

  if (gpio_init == 0)
  {
    return ARM_DRIVER_ERROR;
  }
  if (gpio_pin < 32)
  {
    if ((((gpio0->gpio_pins0) >> gpio_pin) & 0x1) == 0)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else if (gpio_pin < 37)
  {
    if ((((gpio1->gpio_pins1) >> (gpio_pin - 32)) & 0x1) == 0)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (gpio_dir[gpio_pin] != 1)
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  status = (uint8_t)tc9562_GPIO_CheckArgFlag(flag);
  if(status == GPIO_SUCCESS)
  {
    if(gpio_pin < GPIO_THIRTYTWO)
    {
      tc9562_GPIO_ConfigReg(GPIO0_OUT_REG, 0x1U << gpio_pin, flag);
    }
    else
    {
      tc9562_GPIO_ConfigReg(GPIO1_OUT_REG, 0x1U << (gpio_pin - GPIO_THIRTYTWO), flag);
    }
  }
  return (int8_t)status;
}

/**
  \fn          void tc9562_GPIO_ConfigReg(uint32_t Addr_Offs, uint32_t config, uint8_t flag)
  \brief       Write data into gpio register
  \param [in]  Addr_Offs GPIO register offset
  \param [in]  config Value on corresponding bit of gpio pin
  \param [in]  flag HIGH/LOW
  \return      NULL
*/
void tc9562_GPIO_ConfigReg(uint32_t Addr_Offs, uint32_t config, uint8_t flag)
{
  uint32_t data;
  data = hw_reg_read32(TC9562_REG_BASE_ADRS, Addr_Offs);
  if(flag == TC9562_ONE)
  {
    data |= config;
  } else
  {
    data &= ~(config);
  }
  hw_reg_write32(TC9562_REG_BASE_ADRS, Addr_Offs, data);
}

/**
  \fn          int8_t tc9562_GPIO_ConfigInput(uint8_t gpio_pin)
  \brief       Config the gpio pin as input pin
  \param [in]  gpio_pin [0-36]
  \return      \ref execution_status
*/
int8_t tc9562_GPIO_ConfigInput(uint8_t gpio_pin)
{
  uint32_t config;

  if (gpio_init == 0)
  {
    return ARM_DRIVER_ERROR;
  }
  if (gpio_pin < 32)
  {
    if ((((gpio0->gpio_pins0) >> gpio_pin) & 0x1) == 0)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else if (gpio_pin < 37)
  {
    if ((((gpio1->gpio_pins1) >> (gpio_pin - 32)) & 0x1) == 0)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if(gpio_pin < GPIO_THIRTYTWO)
  {
    config = (0x1U << gpio_pin) ;
    hw_reg_write32(TC9562_REG_BASE_ADRS, GPIO0_ENABLE, (hw_reg_read32 (TC9562_REG_BASE_ADRS, GPIO0_ENABLE))| config);
  } else
  {
    config = (0x1U << (gpio_pin - GPIO_THIRTYTWO)) ;
    hw_reg_write32(TC9562_REG_BASE_ADRS, GPIO1_ENABLE, (hw_reg_read32 (TC9562_REG_BASE_ADRS, GPIO1_ENABLE))| config);
  }
  gpio_dir[gpio_pin] = 0;
  return (int8_t)ARM_DRIVER_OK;
}

/**
  \fn          int8_t tc9562_GPIO_InputData (uint8_t gpio_pin)
  \brief       Read the data from input pin
  \param [in]  gpio_pin [0-36]
  \return      \ref execution_status
*/
int8_t tc9562_GPIO_InputData (uint8_t gpio_pin)
{
  uint32_t ret;

  if (gpio_init == 0)
  {
    return ARM_DRIVER_ERROR;
  }
  if (gpio_pin < 32)
  {
    if ((((gpio0->gpio_pins0) >> gpio_pin) & 0x1) == 0)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else if (gpio_pin < 37)
  {
    if ((((gpio1->gpio_pins1) >> (gpio_pin - 32)) & 0x1) == 0)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (gpio_dir[gpio_pin] != 0)
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if(gpio_pin < GPIO_THIRTYTWO)
  {
    ret = hw_reg_read32(TC9562_REG_BASE_ADRS, GPIO0_REG);
    ret = (ret >> gpio_pin)& 0x1U;
  }
  else
  {
    ret = hw_reg_read32(TC9562_REG_BASE_ADRS, GPIO1_REG);
    ret = (ret >> (gpio_pin - GPIO_THIRTYTWO))& 0x1U;
  }
  return (int8_t)ret;
}

/**
  \fn          int8_t tc9562_GPIO_CheckArgFlag(uint8_t data)
  \brief       Check the argument flag as valid
  \param [in]  data Flag
  \return      \ref execution_status
*/
int8_t tc9562_GPIO_CheckArgFlag(uint8_t data)
{
  int8_t status;
  if ((data == GPIO_HIGH) || (data == GPIO_LOW))
  {
    status = GPIO_SUCCESS;
  } else
  {
    status = ARM_DRIVER_ERROR_PARAMETER;  /*Invalid argument. argument other than 0 or 1.*/
  }
  return status;
}

/**
  \fn          int8_t tc9562_GPIO_Uninitialize (void)
  \brief       De-initialize GPIO Interface.
  \return      \ref execution_status
*/
int8_t tc9562_GPIO_Uninitialize (void)
{
  if (gpio_init == 0)
  {
    return ARM_DRIVER_ERROR;
  }
  else
  {
    gpio_init = 0;
    gpio0->gpio_pins0 = 0;
    gpio1->gpio_pins1 = 0;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int8_t tc9562_GPIO_ConfigIntr(uint8_t pinNum, uint8_t config)
  \brief       Enable or Disable External Interrupts
  \param [in]  pinNum [Interrupt 0, 1, 2]
  \param [in]  config Enable\Disable
  \return      \ref execution_status
*/
int8_t tc9562_GPIO_ConfigIntr(uint8_t pinNum, uint8_t config)
{
  uint32_t regVal;
  if (gpio_init == 0)
  {
    return ARM_DRIVER_ERROR;
  }
  if (pinNum == INTI03)
  {
    if(config == GPIO_IRQ_ENABLE)
    {
      hw_reg_write32 (TC9562_REG_BASE_ADRS, TC9562_NFUNCEN7_OFFS, MUXF0);
    } else
    {
      NVIC_DisableIRQ(INT_SRC_NBR_INTI03);
      return ARM_DRIVER_OK;
    }
  } else if ((pinNum == INTI02) || (pinNum == INTI01))
  {
    if(config == GPIO_IRQ_ENABLE)
    {
      /*set input direction*/
      hw_reg_write32 (TC9562_REG_BASE_ADRS, GPIO1_ENABLE, (hw_reg_read32(TC9562_REG_BASE_ADRS, GPIO1_ENABLE) | ((pinNum-34) << TC9562_THREE)));
    } else
    {
      regVal = hw_reg_read32 (TC9562_REG_BASE_ADRS, TC9562_NFUNCEN1_OFFS);
      if (pinNum == INTI01)
      {
        regVal &= ~(MUXF0 << MII_TXER_POS);
        hw_reg_write32 (TC9562_REG_BASE_ADRS, TC9562_NFUNCEN1_OFFS, regVal);
        NVIC_DisableIRQ(INT_SRC_NBR_INTI01);
      } else
      {
        regVal &= ~(MUXF0 << MII_RXER_POS);
        hw_reg_write32 (TC9562_REG_BASE_ADRS, TC9562_NFUNCEN1_OFFS, regVal);
        NVIC_DisableIRQ(INT_SRC_NBR_INTI02);
      }
      return ARM_DRIVER_OK;
    }
  } else
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (pinNum)
  {

  case INTI01:
    /*Pin MUX register update*/
    regVal = hw_reg_read32 (TC9562_REG_BASE_ADRS, TC9562_NFUNCEN1_OFFS);
    regVal |= (MUXF2 << MII_TXER_POS);
    hw_reg_write32 (TC9562_REG_BASE_ADRS, TC9562_NFUNCEN1_OFFS, regVal);
    hw_reg_write32 (TC9562_REG_BASE_ADRS, EXTINTCFG, hw_reg_read32(TC9562_REG_BASE_ADRS, EXTINTCFG) | (0x2));
    /*INTMCUMASK register update*/
    regVal = hw_reg_read32 (TC9562_REG_BASE_ADRS, INTMCUMASK0);
    regVal &= ~(0x1 << INTI01_POS);
    hw_reg_write32 (TC9562_REG_BASE_ADRS, INTMCUMASK0, regVal);
    NVIC_EnableIRQ(INT_SRC_NBR_INTI01);
    break;

  case INTI02:
    /*Pin MUX register update*/
    regVal = hw_reg_read32 (TC9562_REG_BASE_ADRS, TC9562_NFUNCEN1_OFFS);
    regVal |= (MUXF2 << MII_RXER_POS);
    hw_reg_write32 (TC9562_REG_BASE_ADRS, TC9562_NFUNCEN1_OFFS, regVal);
    hw_reg_write32 (TC9562_REG_BASE_ADRS, EXTINTCFG, hw_reg_read32(TC9562_REG_BASE_ADRS, EXTINTCFG) | 0x2<<2);
    /*INTMCUMASK register update*/
    regVal = hw_reg_read32 (TC9562_REG_BASE_ADRS, INTMCUMASK0);
    regVal &= ~(0x1 << INTI02_POS);
    hw_reg_write32 (TC9562_REG_BASE_ADRS, INTMCUMASK0, regVal);
    NVIC_EnableIRQ(INT_SRC_NBR_INTI02);
    break;

  default:
    /*INTMCUMASK register update*/
    hw_reg_write32 (TC9562_REG_BASE_ADRS, EXTINTCFG, hw_reg_read32(TC9562_REG_BASE_ADRS, EXTINTCFG) | 0x2<<4);
    tc9562_GPIO_ConfigReg (NSLEEPCTR_OFFS, TC9562_ONE << TC9562_ONE, config);
    regVal = hw_reg_read32 (TC9562_REG_BASE_ADRS, INTMCUMASK0);
    regVal &= ~(0x1 << INTI03_POS);
    hw_reg_write32 (TC9562_REG_BASE_ADRS, INTMCUMASK0, regVal);
    NVIC_EnableIRQ(INT_SRC_NBR_INTI03);
    break;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          void INTI01_IRQHandler(void)
  \brief       IRQ handler
  \return      NULL
*/
void INTI01_IRQHandler(void)
{
  /*TC9562_Ser_Printf("GPIO9_IRQHandler\r\n");*/
  tc9562_GPIO_ConfigReg(EXTINTFLG_OFFS, TC9562_ONE << INTI01_POS, GPIO_HIGH);
  NVIC_ClearPendingIRQ((IRQn_Type)INT_SRC_NBR_INTI01);
}

/**
  \fn          void INTI02_IRQHandler(void)
  \brief       IRQ handler
  \return      NULL
*/
void INTI02_IRQHandler(void)
{
 /* TC9562_Ser_Printf("GPI10_IRQHandler\r\n"); */
  tc9562_GPIO_ConfigReg(EXTINTFLG_OFFS, TC9562_ONE << INTI02_POS, GPIO_HIGH);
  NVIC_ClearPendingIRQ((IRQn_Type)INT_SRC_NBR_INTI02);
}

/**
  \fn          void INTI03_IRQHandler(void)
  \brief       IRQ handler
  \return      NULL
*/
void INTI03_IRQHandler(void)
{
  /*TC9562_Ser_Printf("GPI11_IRQHandler\r\n");*/
  tc9562_GPIO_ConfigReg(EXTINTFLG_OFFS, TC9562_ONE << INTI03_POS, GPIO_HIGH);
  NVIC_ClearPendingIRQ((IRQn_Type)INT_SRC_NBR_INTI03);
}

/**
    \brief Access structure of the GPIO Driver.
*/
TC9562_GPIO_DRIVER tc9562_gpio = {
  tc9562_GPIO_GetVersion,
  tc9562_GPIO_Initialize,
  tc9562_GPIO_Uninitialize,
  tc9562_GPIO_ConfigOutput,
  tc9562_GPIO_ConfigInput,
  tc9562_GPIO_ConfigIntr,
  tc9562_GPIO_OutputData,
  tc9562_GPIO_InputData
};
