vdac_timer_dma_waveform

This project uses the DAC/VDAC and TIMER0 to output a 32 point sine wave at a
particular frequency (10 kHz by default). The project uses the LDMA to write to 
the CH0F buffer. This project operates in EM1 because the timer can't operate in 
EM2/EM3.

This example used approximately 950 uA. After commenting out the line of code 
that puts the board in EM1, this example used approximately 1.2 mA.
Note: this energy measurement was done using Simplicity Studio's
built-in energy profiler for BRD4263B with a Debug build configuration and
optimization gcc -g3.

Note: For EFR32xG21 radio devices, library function calls to CMU_ClockEnable() 
have no effect as oscillators are automatically turned on/off based on demand 
from the peripherals; CMU_ClockEnable() is a dummy function for EFR32xG21 for 
library consistency/compatibility.

================================================================================

How To Test:
1. Build the project and download to the Starter Kit
2. Measure the VDAC output pin with respect to ground

================================================================================

Peripherals Used:
HFRCODPLL - 19 MHz
VDAC  - internal 1.25V reference, continuous mode

================================================================================

Listed below are the devices that do not have a VDAC module
 - EFR32xG21
 - EFR32xG22

================================================================================

Listed below are the port and pin mappings for working with this example.

Board:  Silicon Labs EFR32FG23 Starter Kit (BRD4263B)
Device: EFR32FG23A010F512GM48
PB0 - VDAC0 CH0 Main Output (Pin 15 of breakout pads)
