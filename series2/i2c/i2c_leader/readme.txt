i2c_leader

This project demonstrates the leader configuration of the EFx32xG2x I2C
peripheral. Two EFx32xG2x devices are connected together, one running the leader
project, the other running the follower project. The leader starts up and enters
a while loop waiting for a button press on push button 0. When push button 0 is
pressed, the program performs an I2C test. This routine reads the follower
device's current buffer values, increments each value by 1, and writes the new
values back to the follower device. The leader then reads back the follower
values again and verifies the new values match what was previously written. Upon
a successful write, LED0 is toggled and the device re-enters the while loop,
waiting again for user input through push button 0. This project runs in a
continuous loop, re-running the I2C test with every PB0 button press and
toggling LED0 with each successful iteration. If there is an I2C transmission
error, or if the verification step of the I2C test fails, LED1 is turned on and
the leader sits and remains in an infinite while loop. Connecting to the device
via debugger while in the infinite loop, the I2C error code can be retrieved.

Note: For EFR32xG21 radio devices, library function calls to CMU_ClockEnable() 
have no effect as oscillators are automatically turned on/off based on demand 
from the peripherals; CMU_ClockEnable() is a dummy function for EFR32xG21 for 
library consistency/compatibility.

Note: This example uses inclusive lexicon wherever possible. For more
information, visit https://www.silabs.com/about-us/inclusive-lexicon-project

================================================================================

How To Test:
1.  Connect the SDA, SCL and GND lines between two kits via the EXP header
2.  Jumper 4.7kOhm pull-up resistors from VMCU to the SDA and SCL lines (only
    one pull-up needed for each line).
3.  Open Simplicity Studio and update each kit's firmware from the Simplicity
    Launcher (if necessary)
4.  Build both the leader and follower projects and download to two Starter Kits
5.  On leader kit, project will compile, load and start-up, proceeding to the
    main loop 
6.  Press PB0 on the leader kit to trigger an I2C test (read follower buffer,
    increment, write, read, verify)
7.  Observe LED0 toggle with a successful I2C test
8.  Suspend (pause) the leader kit via debugger session
9.  Observe that received values received from the follower (rxBuffer) match
    transmitted values sent by leader (txBuffer) by viewing "rxBuffer" and
    txBuffer respectively in the Expressions window of Simplicity Studio.

================================================================================

Peripherals Used:
FSRCO - 20 MHz
I2C0  - 100 kHz

================================================================================

Board:  Silicon Labs EFR32xG21 Radio Board (BRD4181A) + 
        Wireless Starter Kit Mainboard
Device: EFR32MG21A010F1024IM32
PD02 - Push button PB0, Expansion Header Pin 7, WSTK Pin 4
PB00 - LED0, Expansion Header Pin 11, WSTK Pin 8
PB01 - LED1, Expansion Header Pin 13, WSTK Pin 10
PA05 - I2C_SDA, Expansion Header Pin 12, WSTK Pin 9
PA06 - I2C_SCL, Expansion Header Pin 14, WSTK Pin 11

Board:  Silicon Labs EFR32xG22 Radio Board (BRD4182A) + 
        Wireless Starter Kit Mainboard
Device: EFR32MG22C224F512IM40
PB00 - Push button PB0, Expansion Header Pin 7, WSTK Pin 4
PD02 - LED0, Expansion Header Pin 11, WSTK Pin 8
PD03 - LED1, Expansion Header Pin 13, WSTK Pin 10
PA05 - I2C_SDA, Expansion Header Pin 12, WSTK Pin 9
PA06 - I2C_SCL, Expansion Header Pin 14, WSTK Pin 11

Board:  Silicon Labs EFR32FG23 Radio Board (BRD4263B) + 
        Wireless Starter Kit Mainboard
Device: EFR32FG23A010F512GM48
PB01 - Push button PB0, WSTK Pin 17
PB02 - LED0, WSTK Pin 19
PD03 - LED1, WSTK Pin 26
PA05 - I2C_SDA, Expansion Header Pin 7, WSTK Pin 4
PA06 - I2C_SCL, Expansion Header Pin 11, WSTK Pin 8
