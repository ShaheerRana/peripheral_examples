/***************************************************************************//**
 * @file main_gg11_xg14.c
 * @brief Trigger single ADC conversions periodically via LETIMER in EM2. PRS
 * routes LETIMER signals to ADC Start Conversion, then LDMA routes ADC data
 * to SW buffer on Conversion Complete.
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
#include "em_ldma.h"
#include "em_letimer.h"

// Change this to set number of samples per interrupt
#define ADC_BUFFER_SIZE 8

// Change this to set how many samples get sent at once
#define ADC_DVL         2

// Init to max ADC clock for Series 1 with AUXHFRCO
#define ADC_FREQ        4000000

// Desired letimer interrupt frequency (in Hz)
#define letimerDesired  16000

#define LDMA_CHANNEL    0
#define PRS_CHANNEL     0

// Buffer for ADC single and scan conversion
uint32_t adcBuffer[ADC_BUFFER_SIZE];
uint32_t topValue;
LDMA_TransferCfg_t trans;
LDMA_Descriptor_t descr;

/**************************************************************************//**
 * @brief LDMA Handler
 *****************************************************************************/
void LDMA_IRQHandler(void)
{
  // Clear interrupt flag
  LDMA_IntClear((1 << LDMA_CHANNEL) << _LDMA_IFC_DONE_SHIFT);
}

/**************************************************************************//**
 * @brief LETIMER initialization
 *****************************************************************************/
void initLetimer(void)
{
  // Start LFRCO and wait until it is stable
  CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

  // Enable clock to the interface of the low energy modules
  CMU_ClockEnable(cmuClock_HFLE, true);

  // Route the LFRCO clock to LFA (TIMER0)
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);

  // Enable clock for LETIMER0
  CMU_ClockEnable(cmuClock_LETIMER0, true);

  LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;

  // Reload COMP0 on underflow, pulse output, and run in repeat mode
  letimerInit.comp0Top  = true;
  letimerInit.ufoa0     = letimerUFOAPulse;
  letimerInit.repMode   = letimerRepeatFree;

  // Initialize LETIMER
  LETIMER_Init(LETIMER0, &letimerInit);

  // Need REP0 != 0 to pulse on underflow
  LETIMER_RepeatSet(LETIMER0, 0, 1);

  // calculate the topValue
   topValue = CMU_ClockFreqGet(cmuClock_LETIMER0) / letimerDesired;

  // Compare on wake-up interval count
  LETIMER_CompareSet(LETIMER0, 0, topValue);

  // Use LETIMER0 as async PRS to trigger ADC in EM2
  CMU_ClockEnable(cmuClock_PRS, true);

  PRS_SourceAsyncSignalSet(PRS_CHANNEL, PRS_CH_CTRL_SOURCESEL_LETIMER0,
      PRS_CH_CTRL_SIGSEL_LETIMER0CH0);
}

/**************************************************************************//**
 * @brief LDMA initialization
 *****************************************************************************/
void initLdma(void)
{
  // Enable CMU clock
  CMU_ClockEnable(cmuClock_LDMA, true);

  // Basic LDMA configuration
  LDMA_Init_t ldmaInit = LDMA_INIT_DEFAULT;

  LDMA_Init(&ldmaInit);

  // Transfers trigger off ADC single conversion complete
  trans = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_ADC0_SINGLE);

  descr = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(
      &(ADC0->SINGLEDATA),  // source
      adcBuffer,            // destination
      ADC_BUFFER_SIZE,      // data transfer size
      0);                   // link relative offset (links to self)

  descr.xfer.blockSize =ADC_DVL-1;    // transfers ADC_DVL number of units per arbitration cycle
  descr.xfer.ignoreSrec = true;       // ignores single requests to save energy

  // Initialize transfer
  LDMA_StartTransfer(LDMA_CHANNEL, &trans, &descr);

  // Clear pending and enable interrupts for LDMA
  NVIC_ClearPendingIRQ(LDMA_IRQn);
  NVIC_EnableIRQ(LDMA_IRQn);
}

/**************************************************************************//**
 * @brief ADC initialization
 *****************************************************************************/
void initAdc(void)
{
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
  
  // Set single data valid level (DVL)
  ADC0->SINGLECTRLX = (ADC0->SINGLECTRLX & ~_ADC_SINGLECTRLX_DVL_MASK) | (((ADC_DVL - 1) << _ADC_SINGLECTRLX_DVL_SHIFT) & _ADC_SINGLECTRLX_DVL_MASK); 
  
  // Clear the Single FIFO
  ADC0->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  CHIP_Init();

  // Setup ADC to perform conversions via PRS
  initAdc();
  // Setup DMA to move ADC results to user memory
  initLdma();
  // Set up LETIMER to trigger ADC via PRS in 500ms intervals
  initLetimer();

  // Infinite loop
  while(1)
  {
    // Enter EM2 until next ADC interrupt
    EMU_EnterEM2(false);
  }
}
