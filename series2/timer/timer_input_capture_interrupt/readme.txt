timer_input_capture_interrupt

This project demonstrates the use of the TIMER module for interrupt
based input capture. After button 0 is pressed, the PRS routes this signal
to the timer to indicate that an input capture needs to occur. An interrupt
then occurs because the Compare/Capture channel interrupt flag is set.
The captured timer value is then stored in the user's buffer.

Note: This project triggers a capture on a falling edge because the buttons
are negative logic (i.e. the pin value goes low when the button is pressed).

The example also has comments that show how polled mode can be used for input 
capture.

Note: For EFR32xG21 radio devices, library function calls to CMU_ClockEnable() 
have no effect as oscillators are automatically turned on/off based on demand 
from the peripherals; CMU_ClockEnable() is a dummy function for EFR32xG21 for 
library consistency/compatibility.

================================================================================

Peripherals Used:
HFRCO  - 19 MHz
TIMER0 - HFPERCLK (19 MHz for series 2 boards)
PRS    - (route GPIO signal to the timer)

================================================================================

How To Test:
1. Build the project and download it to the Starter Kit
2. Go into debug mode and click run
3. Press button 0 to trigger the input capture and have the value be recorded
4. Pause the debugger to check if the value was recorded in the global
   `buffer` variable

================================================================================

Listed below are the port and pin mappings for working with this example.

Board: Silicon Labs EFR32xG21 2.4 GHz 10 dBm Board (BRD4181A) 
       + Wireless Starter Kit Mainboard (BRD4001A)
Device: EFR32MG21A010F1024IM32
PD02 - Push Button 0

Board:  Silicon Labs EFR32xG22 Radio Board (BRD4182A) + 
        Wireless Starter Kit Mainboard
Device: EFR32MG22C224F512IM40
PB00 - Push Button 0

Board:  Silicon Labs EFR32xG23 Radio Board (BRD4263B) + 
        Wireless Starter Kit Mainboard
Device: EFR32FG23A010F512GM48
PB01 - Push Button 0