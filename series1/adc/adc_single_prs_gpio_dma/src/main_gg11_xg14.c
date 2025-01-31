/***************************************************************************//**
 * @file main_gg11_xg14.c
 * @brief Use the ADC to take a nonblocking measurements in EM2. The PRS 
 * redirects GPIO signals to start ADC single conversions. The LDMA moves
 * completed conversions to a SW buffer
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
#include "em_emu.h"
#include "em_adc.h"
#include "em_prs.h"
#include "em_gpio.h"
#include "em_ldma.h"
#include "bsp.h"

// Change this to set number of samples.
#define ADC_BUFFER_SIZE   4

// Init to max ADC clock for Series 1 with AUXHFRCO
#define ADC_FREQ        4000000

#define LDMA_CHANNEL      0
#define PRS_CHANNEL       0

// Buffer for ADC single conversion
uint32_t adcBuffer[ADC_BUFFER_SIZE];

LDMA_TransferCfg_t trans;
LDMA_Descriptor_t descr;
volatile uint32_t counter = 0;
/**************************************************************************//**
 * @brief LDMA Handler
 *****************************************************************************/
void LDMA_IRQHandler(void)
{
  // Clear interrupt flag
  LDMA_IntClear(LDMA_IntGet());
  counter++;
  //ldmaLoopCnt = 5;
  // Insert transfer complete functionality here
  LDMA->SWREQ |= 1 << 0;

}

/**************************************************************************//**
 * @brief GPIO initialization
 *****************************************************************************/
/*void initGpio(void)
{
  // Enable clock for GPIO
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Set Push Button 0 to input
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInput, 0);

  // Configure Push Button 0 to create PRS interrupt signals only
  GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, false, false, true);

  // Use GPIO PB0 as async PRS to trigger ADC in EM2
  CMU_ClockEnable(cmuClock_PRS, true);

  if (BSP_GPIO_PB0_PIN > 7){
    PRS_SourceAsyncSignalSet(PRS_CHANNEL, PRS_CH_CTRL_SOURCESEL_GPIOH, (uint32_t)(BSP_GPIO_PB0_PIN - 8));
   } else {
    PRS_SourceAsyncSignalSet(PRS_CHANNEL, PRS_CH_CTRL_SOURCESEL_GPIOL, BSP_GPIO_PB0_PIN);
  }
}*/

/**************************************************************************//**
 * @brief LDMA initialization
 *****************************************************************************/
void initLdma(void)
{
//  LDMA_Init_t init = LDMA_INIT_DEFAULT;
//  LDMA_Init( &init );
//
//  /* Writes directly to the LDMA channel registers */
//  LDMA->CH[LDMA_CHANNEL].CTRL =
//      LDMA_CH_CTRL_SIZE_HALFWORD
//      + LDMA_CH_CTRL_REQMODE_ALL
//      + LDMA_CH_CTRL_BLOCKSIZE_UNIT4
//      + (ADC_BUFFER_SIZE -1 << _LDMA_CH_CTRL_XFERCNT_SHIFT);
//  LDMA->CH[LDMA_CHANNEL].SRC = (uint32_t)&(ADC0->SINGLEDATA);
//  LDMA->CH[LDMA_CHANNEL].DST = (uint32_t)&adcBuffer;
//
//  /* Enable interrupt and use software to start transfer */
//  LDMA->CH[LDMA_CHANNEL].REQSEL = ldmaPeripheralSignal_NONE;
//  LDMA->IFC = 1 << 0;
//  LDMA->IEN = 1 << 0;
//
//  /* Enable LDMA Channel */
//  LDMA->CHEN = 1 << 0;
//
//  /* Request transfer */
//  LDMA->SWREQ |= 1 << 0;

  // Enable LDMA clock
  CMU_ClockEnable(cmuClock_LDMA, true);

  // Basic LDMA configuration
  LDMA_Init_t ldmaInit = LDMA_INIT_DEFAULT;

  ldmaInit.ldmaInitCtrlNumFixed = 1;
  //ldmaInit.ldmaInitCtrlSyncPrsClrEn = 1;
  LDMA_Init(&ldmaInit);

  // Transfers trigger off ADC single conversion complete
  trans = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL_LOOP(ldmaPeripheralSignal_ADC0_SINGLE,5);

  descr = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(
      &(ADC0->SINGLEDATA),  // source
      adcBuffer,            // destination
      ADC_BUFFER_SIZE,      // data transfer size
      0);                   // link relative offset (links to self)
  descr.xfer.link        = 0;                        // End of linked list
  descr.xfer.linkMode = 0;
  descr.xfer.linkAddr = 0;
  descr.xfer.decLoopCnt = true;
  descr.xfer.dstAddrMode = ldmaCtrlSrcAddrModeRel;   // Each consecutive transfer uses the previous destination
  descr.xfer.doneIfs = 1;
  descr.xfer.structReq = 0;
  //counter = trans.ldmaLoopCnt;
  descr.xfer.ignoreSrec = true;       // ignore single requests to reduce time spent out of EM2

  // Initialize LDMA transfer
  LDMA_StartTransfer(LDMA_CHANNEL, &trans, &descr);

  // Send software request
  LDMA->SWREQ |= 1 << 0;


  // Clear pending and enable interrupts for channel
  //NVIC_ClearPendingIRQ(LDMA_IRQn);
  //NVIC_EnableIRQ(LDMA_IRQn);

  //new addition - Shaheer
  //NVIC_SetPriority (LDMA_IRQn, 0);
}

/**************************************************************************//**
 * @brief ADC initialization
 *****************************************************************************/
void initAdc(void)
{
/*
  CMU_ClockEnable(cmuClock_ADC0, true);

   // Declare init structs
   ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
   ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

   // Modify init structs and initialize
   init.prescale = ADC_PrescaleCalc(16000000, 0); // Init to max ADC clock for Series 1

   initSingle.diff       = false;        // single ended
   initSingle.reference  = adcRef2V5;    // internal 2.5V reference
   initSingle.resolution = adcRes12Bit;  // 12-bit resolution
   initSingle.acqTime    = adcAcqTime4;  // set acquisition time to meet minimum requirement

   // Select ADC input. See README for corresponding EXP header pin.
   initSingle.posSel = adcPosSelAPORT4XCH11;
   init.timebase = ADC_TimebaseCalc(0);

   initSingle.singleDmaEm2Wu = 1;
   initSingle.prsEnable = true;
   initSingle.prsSel = (ADC_PRSSEL_TypeDef) PRS_CHANNEL;

   ADC_Init(ADC0, &init);
   ADC_InitSingle(ADC0, &initSingle);
*/

  // Declare init structs
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  // Enable ADC clock
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_ADC0, true);

  // Select AUXHFRCO for ADC ASYNC mode so it can run in EM2
  CMU->ADCCTRL = CMU_ADCCTRL_ADC0CLKSEL_AUXHFRCO;

  // Set AUXHFRCO frequency and use it to setup the ADC
  CMU_AUXHFRCOFreqSet(cmuAUXHFRCOFreq_4M0Hz);
  init.timebase = ADC_TimebaseCalc(CMU_AUXHFRCOBandGet());
  init.prescale = ADC_PrescaleCalc(ADC_FREQ, CMU_AUXHFRCOBandGet());

  // Let the ADC enable its clock in EM2 when necessary
  init.em2ClockConfig = adcEm2ClockOnDemand;
  // DMA is available in EM2 for processing SINGLEFIFO DVL request
  initSingle.singleDmaEm2Wu = 1;

  // Add external ADC input. See README for corresponding EXP header pin.
  initSingle.posSel = adcPosSelAPORT4XCH11;

  // Basic ADC single configuration
  initSingle.diff = false;              // single-ended
  initSingle.reference  = adcRef2V5;    // 2.5V reference
  initSingle.resolution = adcRes12Bit;  // 12-bit resolution
  initSingle.acqTime    = adcAcqTime4;  // set acquisition time to meet minimum requirements

  // Enable PRS trigger and select channel 0
  initSingle.prsEnable = true;
  initSingle.prsSel = (ADC_PRSSEL_TypeDef) PRS_CHANNEL;

  // Initialize ADC
  ADC_Init(ADC0, &init);
  ADC_InitSingle(ADC0, &initSingle);

  // Clear the Single FIFO
  ADC0->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;

}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
//new features
volatile uint32_t sample;
volatile uint32_t millivolts;

int main(void)
{
  CHIP_Init();

  // Set up GPIO to trigger ADC via PRS
  //initGpio();
  // Setup ADC to perform conversions
  initAdc();
  // Setup DMA to move ADC results to user memory
  initLdma();

  // Infinite loop
/*(  while(1)
  {
    // Enter EM2 until next interrupt
    EMU_EnterEM2(false);
  */
  while(1)
  {
    // Start ADC conversion
    ADC_Start(ADC0, adcStartSingle);

    // Wait for conversion to be complete
    while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));

    // Get ADC result
    sample = ADC_DataSingleGet(ADC0);

    // Calculate input voltage in mV
    millivolts = (sample * 2500) / 4096;
  }

}
