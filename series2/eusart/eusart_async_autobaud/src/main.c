/***************************************************************************//**
 * @file main.c
 *
 * @brief This project demonstrates use of auto baud detection in asynchronous
 * mode of the EUSART with interrupts.
 *
 * After initialization, the MCU goes into EM1 and waits to see a sync word
 * (0x55 or ASCII character'U') on the RX pin to set the baud rate. Once the 
 * baud rate is set, firmware disables the  Auto Baud Complete interrupt and 
 * enables the RX FIFO Level (RXFL) Interrupt. The RXFL IRQ handler buffers 
 * incoming data until the reception of 80 characters or a CR (carriage return, 
 * ASCII 0x0D).
 *
 * Once the CR or 80 characters is hit, firmware disables the RXFL interrupt
 * and enables the transmit FIFO level (TXFL) interrupt, which is set by default
 * after power-on. Each entry into the transmit interrupt
 * handler causes a character to be sent until all the characters originally
 * received have been echoed.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
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

#include <stddef.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_eusart.h"
#include "em_gpio.h"

// BSP for board controller pin macros
#include "bsp.h"

// Size of the buffer for received data
#define BUFLEN  80

// Receive data buffer
uint8_t buffer[BUFLEN];

// Current position ins buffer
uint32_t inpos = 0;
uint32_t outpos = 0;

// True while receiving data (waiting for CR or BUFLEN characters)
bool receive = true;

#define RETARGET_TXPORT      BSP_BCC_TXPORT      /* EUSART transmission port */
#define RETARGET_TXPIN       BSP_BCC_TXPIN       /* EUSART transmission pin */
#define RETARGET_RXPORT      BSP_BCC_RXPORT      /* EUSART reception port */
#define RETARGET_RXPIN       BSP_BCC_RXPIN       /* EUSART reception pin */

/**************************************************************************//**
 * @brief
 *    GPIO initialization
 *****************************************************************************/
void initGpio(void)
{
  // Enable GPIO Clock
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure the EUSART TX pin to the board controller as an output
  GPIO_PinModeSet(RETARGET_TXPORT, RETARGET_TXPIN, gpioModePushPull, 1);

  // Configure the EUSART RX pin to the board controller as an input
  GPIO_PinModeSet(RETARGET_RXPORT, RETARGET_RXPIN, gpioModeInput, 0);

  // Route EUSART1 TX and RX to the board controller TX and RX pins
  GPIO->EUSARTROUTE[1].TXROUTE = (RETARGET_TXPORT << _GPIO_EUSART_TXROUTE_PORT_SHIFT)
      | (RETARGET_TXPIN << _GPIO_EUSART_TXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[1].RXROUTE = (RETARGET_RXPORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
      | (RETARGET_RXPIN << _GPIO_EUSART_RXROUTE_PIN_SHIFT);

  // Enable RX and TX signals now that they have been routed
  GPIO->EUSARTROUTE[1].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN | GPIO_EUSART_ROUTEEN_TXPEN;

  /*
   * Configure the BCC_ENABLE pin as output and set high. This enables
   * the virtual COM port (VCOM) connection to the board controller and
   * permits serial port traffic over the debug connection to the host
   * PC.
   *
   * To disable the VCOM connection and use the pins on the kit
   * expansion (EXP) header, comment out the following line.
   */
  GPIO_PinModeSet(BSP_BCC_ENABLE_PORT, BSP_BCC_ENABLE_PIN, gpioModePushPull, 1);
}

/**************************************************************************//**
 * @brief
 *    EUSART1 initialization
 *****************************************************************************/
void initEusart1(void)
{
  // Enable EUSART0 Clock
  CMU_ClockEnable(cmuClock_EUSART1, true);

  // Default asynchronous initializer (8N1, no flow control)
  EUSART_UartInit_TypeDef init = EUSART_UART_INIT_DEFAULT_HF;
  init.baudrate = 0;

  // Configure and enable EUSART1 for high-frequency (EM0/1) operation
  EUSART_UartInitHf(EUSART1, &init);

  // Enable NVIC EUSART sources
  NVIC_ClearPendingIRQ(EUSART1_RX_IRQn);
  NVIC_EnableIRQ(EUSART1_RX_IRQn);
  NVIC_ClearPendingIRQ(EUSART1_TX_IRQn);
  NVIC_EnableIRQ(EUSART1_TX_IRQn);
}

/**************************************************************************//**
 * @brief
 *    The EUSART1 receive interrupt saves incoming characters.
 *****************************************************************************/
void EUSART1_RX_IRQHandler(void)
{
  // Get the character just received
  buffer[inpos] = EUSART1->RXDATA;

  if(EUSART_IntGet(EUSART1) & EUSART_IF_AUTOBAUDDONE) {
    // Disable Auto Baud Complete interrupt
    EUSART_IntDisable(EUSART1, EUSART_IEN_AUTOBAUDDONE);

    // Enable receive FIFO level interrupt
    EUSART_IntEnable(EUSART1, EUSART_IEN_RXFL);
  }
  else {
    // Exit loop on new line or buffer full
    if ((buffer[inpos] != '\r') && (inpos < BUFLEN)) {
      inpos++;
    }
    else {
      receive = false;   // Stop receiving on CR
    }
  }

  /*
   * The EUSART differs from the USART in that explicit clearing of
   * RX interrupt flags is required even after emptying the RX FIFO.
   */
  EUSART_IntClear(EUSART1, EUSART_IF_RXFL|EUSART_IF_AUTOBAUDDONE);
}

/**************************************************************************//**
 * @brief
 *    The EUSART1 transmit interrupt outputs characters.
 *****************************************************************************/
void EUSART1_TX_IRQHandler(void)
{
  // Send a previously received character
  if (outpos < inpos) {
    EUSART1->TXDATA = buffer[outpos++];

    /*
     * The EUSART differs from the USART in that the TX FIFO interrupt
     * flag must be explicitly cleared even after a write to the FIFO.
     */
    EUSART_IntClear(EUSART1, EUSART_IF_TXFL);
  }
  else {
  /*
   * Need to disable the transmit FIFO level interrupt in this IRQ
   * handler when done or it will immediately trigger again upon exit
   * even though there is no data left to send.
   */
    receive = true;   // Go back into receive when all is sent

    // Disable transmit FIFO level interrupt
    EUSART_IntDisable(EUSART1, EUSART_IEN_TXFL);
  }
}

/**************************************************************************//**
 * @brief
 *    Main function
 *****************************************************************************/
int main(void)
{
  uint32_t i;

  // Chip errata
  CHIP_Init();

  // Initialize GPIO and EUSART1
  initGpio();
  initEusart1();

  // Enable Auto Baud Complete interrupt
  EUSART_IntEnable(EUSART1, EUSART_IEN_AUTOBAUDDONE);

  while (1) {
    // Zero out buffer
    for (i = 0; i < BUFLEN; i++) {
      buffer[i] = 0;
    }

    // Wait in EM1 while receiving to reduce current draw
    while (receive) {
      EMU_EnterEM1();
    }

    // Disable receive FIFO level interrupt
    EUSART_IntDisable(EUSART1, EUSART_IEN_RXFL);

    // Enable transmit FIFO level interrupt (defaults to one frame)
    EUSART_IntEnable(EUSART1, EUSART_IEN_TXFL);

    // Wait in EM1 while transmitting to reduce current draw
    while (!receive) {
      EMU_EnterEM1();
    }

    // Enable receive FIFO level interrupt
    EUSART_IntEnable(EUSART1, EUSART_IEN_RXFL);

    // Reset buffer indices
    inpos = outpos = 0;
  }
}
