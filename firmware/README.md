# Firmware objects

Use these files to program the FPGA and the Pico 2. No need to install the GOWIN IDE or the Arduino IDE.

## FPGA File Stream
Use openFPGALoader to program the FPGA

* start MSYS2
* openFPGALoader --detect
* openFPGALoader -b tangnano9k -f uDST.fs

If successful, the scrolling pattern should appear on the 6 LEDs.

## Pico UF2

* Put the Pico 2 into bootloader mode (press the *Boot* button while plugging in the USB cable).
* Copy the UF2 file into the directory that pops up.

If successful, the LED should flash the error code for FPGA missing.
