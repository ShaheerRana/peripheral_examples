ldma_ping_pong

This example is based on the EFR32 Series 2 Reference Manual:
Chapter: LDMA
Section: Examples
Subsection: Example #7

Software requests LDMA ping-pong transfers. A universal source array is
transfered first to the "pingBuffer" array, then to the "pongBuffer", then back
to ping, etc. After each transfer, there is an interrupt that increments the
elements of the source buffer, then requests the next transfer.  
In this way, you should see "11111111" transfered to ping, then "22222222" 
transfered to pong, etc.

================================================================================

How To Test:
1. Update the kit's firmware from the Simplicity Launcher (if necessary)
2. Build the project and download to the Starter Kit
3. Open Simplicity Debugger and add "pingBuffer" and "pongBuffer" to the
   Expressions window
4. Add a breakpoint at the beginning of LDMA_IRQHandler()
5. Run the debugger. It should halt inside the interrupt subroutine with the
   first descriptor complete (this can be seen in the Expressions window)
6. Resume the program. The debugger should halt inside the interrupt subroutine
   again, after the next descriptor has completed.

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