/***************************************************************************//**
 * @file main_gg11_xg14.c
 * @brief Use the ADC to take repeated blocking measurements on a single pin
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *******************************************************************************
 * # Evaluation Quality
 * This code has been minimally tested to ensure that it builds and is suitable 
 * as a demonstration for evaluation purposes only. This code will be maintained
 * at the sole discretion of Silicon Labs.
 ******************************************************************************/

#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"

#include "em_gpio.h"
#include "em_usart.h"
//#include "C:\Users\Co-op Shaheeer R\SimplicityStudio\SDKs\gecko_sdk\platform\emdrv\ustimer\inc\ustimer.h"


#define adcFreq   16000000

volatile uint32_t sample;
volatile uint32_t millivolts;
volatile uint8_t message = 30;

/**************************************************************************//**
 * @brief  Initialize ADC function
 *****************************************************************************/
void initADC (void)
{
  // Enable ADC0 clock
  CMU_ClockEnable(cmuClock_ADC0, true);

  // Declare init structs
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  // Modify init structs and initialize
  init.prescale = ADC_PrescaleCalc(adcFreq, 0); // Init to max ADC clock for Series 1

  initSingle.diff       = false;        // single ended
  initSingle.reference  = adcRef2V5;    // internal 2.5V reference
  initSingle.resolution = adcResOVS;  // 12-bit resolution _ADC_SINGLECTRL_RES_OVS, adcOvsRateSel16
  initSingle.acqTime    = adcAcqTime16;  // set acquisition time to meet minimum requirement

  init.ovsRateSel = adcOvsRateSel16;

  // Select ADC input. See README for corresponding EXP header pin.
  initSingle.posSel = adcPosSelAPORT4XCH11;
  init.timebase = ADC_TimebaseCalc(0);

  ADC_Init(ADC0, &init);
  ADC_InitSingle(ADC0, &initSingle);
}

void initUSART (void)
{

  USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
  CMU_ClockEnable(cmuClock_USART1, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 1);
  USART_InitAsync(USART1, &init);
  USART1->ROUTELOC0 = USART_ROUTELOC0_RXLOC_LOC11 | USART_ROUTELOC0_TXLOC_LOC11;
  USART1->ROUTEPEN |= USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  CHIP_Init();

  initADC();
  initUSART();

  // Infinite loop
  while(1)
  {
    // Start ADC conversion
    ADC_Start(ADC0, adcStartSingle);

    // Wait for conversion to be complete
    while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));
    //USART_Tx(USART1, message);

    // Get ADC result
    sample = ADC_DataSingleGet(ADC0);
    //USART_Tx(USART1, sample);
    //USART_Tx(USART0, '\n');
    //sample +=1;
    //USART_Tx(USART1, '\n');
    //sl_udelay_wait(1000000);
// Calculate input voltage in mV
    millivolts = (sample * 2500) / 4096;
    //USART_Tx(USART1, millivolts);
    //USART_Tx(USART1, '\n');
    int r = sample & 0xff;
    int g = (sample >> 8) & 0xff;
    int b = (sample >> 16) & 0xff;
    int a = (sample >> 24) & 0xff;

    USART_Tx(USART1, r);
    USART_Tx(USART1, g);

  }
}
