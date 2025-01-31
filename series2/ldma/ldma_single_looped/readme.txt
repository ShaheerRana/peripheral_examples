ldma_single_looped

This example is based on the EFR32 Series 2 Reference Manual:
Chapter: LDMA
Section: Examples
Subsection: Example #3

In this example, the LDMA transfers 16 words from one software array to another
in 4 sets of 4 words. The transfer is requested by software at the end of the
LDMA initialization.

If you wanted to have the transfers run automatically without interrupts,
you can remove from initLdma() the following lines:

  descLink.xfer.doneIfs     = true;                     // Enable interrupts
  descLink.xfer.structReq   = false;                    // Disable auto-requests
	
  ... (Do not remove the lines between these)
	
  // Send software request
  LDMA->SWREQ |= LDMA_CH_MASK;
  
Also remove from LDMA_IRQHandler() the following lines:

  // Start next Transfer
  LDMA->SWREQ |= LDMA_CH_MASK;

================================================================================

How To Test:
1. Update the kit's firmware from the Simplicity Launcher (if necessary)
2. Build the project and download to the Starter Kit
3. Open Simplicity Debugger and add "dstBuffer" to the Expressions window
4. Add a breakpoint at the beginning of LDMA_IRQHandler()
5. Run the debugger. It should halt inside the interrupt subroutine with the
   first descriptor complete (this can be seen in the Expressions window)
6. Resume the program. The debugger should halt inside the interrupt subroutine
   again, after the next descriptor has completed. You can do this 2 more times,
   then the LDMA tranfer will be completed.

================================================================================

Peripherals Used:
HFRCODPLL - 19 MHz
LDMA      - Channel 0

================================================================================

Board:  Silicon Labs EFR32xG21 2.4 GHz 10 dBm Radio Board (BRD4181A) + 
        Wireless Starter Kit Mainboard (BRD4001A)
Device: EFM32MG21A010F1024IM32

Board:  Silicon Labs EFR32xG22 Radio Board (BRD4182A) + 
        Wireless Starter Kit Mainboard
Device: EFR32MG22C224F512IM40

Board:  Silicon Labs EFR32FG23 Radio Board (BRD4263B) + 
        Wireless Starter Kit Mainboard
Device: EFR32FG23A010F512GM48