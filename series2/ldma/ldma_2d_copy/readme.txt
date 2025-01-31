ldma_2d_copy

This example is based on the EFR32 Series 2 Reference Manual:
Chapter: LDMA
Section: Examples
Subsection: Example #6

In this example, the LDMA transfers a submatrix from one software matrix to
another. The source buffer is numbered from 0 to 
(BUFFER_2D_WIDTH * BUFFER_2D_HEIGHT - 1) in row major order. Row major order was
chosen so that the debugger will show the matrix as expected.

The submatrix moved will start at (SRC_COL_INDEX, SRC_ROW_INDEX) in the source
matrix, and be moved to (DST_COL_INDEX, DST_ROW_INDEX) in the destination
matrix. The width and height of this submatrix is defined by TRANSFER_HEIGHT
and TRANSFER_WIDTH.

With the sample values given, the LDMA will transfer
10 11 12
20 21 22
30 31 32
40 41 42
from the source matrix to index (2, 1) of the destination matrix.

Note: For EFR32xG21 radio devices, library function calls to CMU_ClockEnable() 
have no effect as oscillators are automatically turned on/off based on demand 
from the peripherals; CMU_ClockEnable() is a dummy function for EFR32xG21 for 
library consistency/compatibility.

================================================================================

How To Test:
1. Update the kit's firmware from the Simplicity Launcher (if necessary)
2. Build the project and download to the Starter Kit
3. Open Simplicity Debugger and add "dst2d" to the Expressions window
4. Add a breakpoint to the beginning of LDMA_IRQHandler()
5. Run the debugger. It should halt inside the interrupt handler
6. You can expand the "dst2d" variable in the Expressions window to see that the
   submatrix has been successfully transfered

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