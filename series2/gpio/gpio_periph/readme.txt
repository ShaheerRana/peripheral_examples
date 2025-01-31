gpio_periph

This project demonstrates how to configure a GPIO pin to drive the output of
the LFRCO, and sets the slew rate and drive strength.

Note for EFR32xG21 devices, clock enabling is not required.

How To Test:
1. Build the project and download to the Starter Kit
2. Connect an oscilloscope to pin PC3 and observe the LFRCO signal

Peripherals Used:
FSRCO  - 20 MHz

Board:  Silicon Labs EFR32xG21 Radio Board (BRD4181A) + 
        Wireless Starter Kit Mainboard
Device: EFR32MG21A010F1024IM32
PC3 - LFRCO output (Expansion Header Pin 10)

Board:  Silicon Labs EFR32xG22 Radio Board (BRD4182A) + 
        Wireless Starter Kit Mainboard
Device: EFR32MG22C224F512IM40
PC3 - LFRCO output (Expansion Header Pin 10)

Board:  Silicon Labs EFR32xG23 Radio Board (BRD4263B) + 
        Wireless Starter Kit Mainboard
Device: EFR32FG23A010F512GM48
PC3 - LFRCO output (Expansion Header Pin 8)
