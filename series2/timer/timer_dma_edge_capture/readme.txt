timer_dma_edge_capture

This project demonstrates DMA driven edge capture from a TIMER Compare/Capture
channel. TIMER0 CC0 is configured to capture rising and falling edges. GPIO Pin
PA6 (see board specific pinout) is to be connected to a periodic signal, and 
edges captured from PA6 are stored in CC0. The LDMA is configured to transfer
the first 512 edges to a fixed length buffer. The buffer is stored globally
for possible future processing.

Note: For EFR32xG21 radio devices, library function calls to CMU_ClockEnable() 
have no effect as oscillators are automatically turned on/off based on demand 
from the peripherals; CMU_ClockEnable() is a dummy function for EFR32xG21 for 
library consistency/compatibility.

================================================================================

Peripherals Used:
HFRCO    - 19 MHz
TIMER0   - CC0
LDMA     - Single Byte P2M

================================================================================

How To Test:
1. Build the project and download to the Starter Kit
2. Connect a periodic signal to GPIO Pin PA6 (see board specific pinout below)
3. View the buffer[] global array in the debugger

================================================================================

Listed below are the port and pin mappings for working with this example.

Board: Silicon Labs EFR32xG21 2.4 GHz 10 dBm Board (BRD4181A) 
       + Wireless Starter Kit Mainboard (BRD4001A)
Device: EFR32MG21A010F1024IM32
PA6 - TIM0_CC0 (Expansion Header Pin 14)

Board:  Silicon Labs EFR32xG22 Radio Board (BRD4182A) + 
        Wireless Starter Kit Mainboard
Device: EFR32MG22C224F512IM40
PA6 - TIM0_CC0 (Expansion Header Pin 14)

Board:  Silicon Labs EFR32xG23 Radio Board (BRD4263B) + 
        Wireless Starter Kit Mainboard
Device: EFR32FG23A010F512GM48
PA6 - TIM0_CC0 (Expansion Header Pin 11)