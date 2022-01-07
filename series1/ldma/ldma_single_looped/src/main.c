/***************************************************************************//**
 * @file main.c
 * @brief LDMA Single Looped Example
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


// DMA channel used for the examples
#define LDMA_CHANNEL      0
#define LDMA_CH_MASK      1 << LDMA_CHANNEL

// Memory to memory transfer buffer size
#define BUFFER_SIZE       4

// Number of sets of BUFFER_SIZE elements to send.
// Total words sent = BUFFER_SIZE * NUM_SETS
#define NUM_ITERATIONS    4

// Constant for loop transfer
// NUM_SETS - 1 (for first descriptor) - 1 (for first iteration)
#define LOOP_COUNT        NUM_ITERATIONS - 1

// Descriptor linked list for LDMA transfer
LDMA_Descriptor_t descLink;

// Buffer for memory to memory transfer
uint32_t srcBuffer[BUFFER_SIZE];
uint32_t dstBuffer[NUM_ITERATIONS][BUFFER_SIZE];
uint32_t adcBuffer[BUFFER_SIZE];

/***************************************************************************//**
 * @brief
 *   LDMA IRQ handler.
 ******************************************************************************/
void LDMA_IRQHandler( void )
{
  uint32_t pending;

  // Read interrupt source
  pending = LDMA_IntGet();

  // Clear interrupts
  LDMA_IntClear(pending);

  // Check for LDMA error
  if ( pending & LDMA_IF_ERROR ){
    // Loop here to enable the debugger to see what has happened */
    while (1);
  }

  // Start next Transfer
  LDMA->SWREQ |= LDMA_CH_MASK;
}

/***************************************************************************//**
 * @brief
 *   Initialize the LDMA controller for single looped transfer
 ******************************************************************************/
void initLdma(void)
{
  LDMA_Init_t init = LDMA_INIT_DEFAULT;
  LDMA_Init( &init );

  // Use looped peripheral transfer configuration macro
  //DMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_MEMORY_LOOP(LOOP_COUNT);
  LDMA_TransferCfg_t periTransferTx = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL_LOOP(ldmaPeripheralSignal_ADC0_SINGLE,LOOP_COUNT);

  // Use LINK descriptor macro for initialization and looping
  descLink = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_M2M_WORD(&(ADC0->SINGLEDATA), 0, BUFFER_SIZE, 0);

  descLink.xfer.blockSize   = ldmaCtrlBlockSizeUnit4;   // Set block sizes to 4
  descLink.xfer.reqMode     = ldmaCtrlReqModeBlock;     // Set request mode to Block instead of all
  descLink.xfer.doneIfs     = true;                     // Enable interrupts
  descLink.xfer.structReq   = false;                    // Disable auto-requests
  descLink.xfer.decLoopCnt  = 1;                        // Enable loops
  descLink.xfer.dstAddrMode = ldmaCtrlSrcAddrModeRel;   // Each consecutive transfer uses the previous destination
  descLink.xfer.link        = 0;                        // End of linked list

  // Start Transfer
  LDMA_StartTransfer(LDMA_CHANNEL, (void*)&periTransferTx, (void*)&descLink);

  // Start transfers at dstBuffer
  LDMA->CH[LDMA_CHANNEL].DST = (uint32_t)&adcBuffer;

  // Send software request
  LDMA->SWREQ |= LDMA_CH_MASK;
}


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
  CMU_AUXHFRCOFreqSet(cmuAUXHFRCOFreq_1M0Hz);
  init.timebase = ADC_TimebaseCalc(CMU_AUXHFRCOBandGet());
  init.prescale = ADC_PrescaleCalc(1000000, CMU_AUXHFRCOBandGet());

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
  initSingle.prsSel = (ADC_PRSSEL_TypeDef) 0;

  // Initialize ADC
  ADC_Init(ADC0, &init);
  ADC_InitSingle(ADC0, &initSingle);

  // Clear the Single FIFO
  ADC0->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;

}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  // Chip errata
  CHIP_Init();

  // Init DCDC regulator if available
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
  EMU_DCDCInit(&dcdcInit);
  initAdc();

  // Initialize LDMA
  initLdma();

  while (1)
  {
    EMU_EnterEM1();
  }
}
