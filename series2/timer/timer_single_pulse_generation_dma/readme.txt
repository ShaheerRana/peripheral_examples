timer_single_pulse_generation_dma

This project demonstrates the generation of a single pulse using output
compare and the LDMA. TIMER0 is initialized for output compare on Compare/Capture 
channel 0 which is routed to GPIO Pin specified below. The timer is configured 
in one-shot mode and an interrupt triggers after the first compare to load the 
second, falling edge to the OC. The LDMA is configured to transfer a single 
word to the OC register once the first compare event occurs. The values loaded 
into the OC are such that a 1ms pulse is generated after a 3 second delay.

Note: For EFR32xG21 radio devices, library function calls to CMU_ClockEnable() 
have no effect as oscillators are automatically turned on/off based on demand 
from the peripherals; CMU_ClockEnable() is a dummy function for EFR32xG21 for 
library consistency/compatibility.

================================================================================

Peripherals Used:
HFRCO  - 19 MHz
TIMER0 - CC0
LDMA   - SINGLE_M2P_BYTE

================================================================================

How To Test:
1. Build the project and download it to the Starter Kit
2. Use an oscilloscope to measure the GPIO pin specified below
3. A 1 ms pulse should be generated after 3 second

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