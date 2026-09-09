## Prerequisites
This is an Arduino project for the Pi Pico 2. You will need the Pico Arduino core by Earle Philhower available here:

https://github.com/earlephilhower/arduino-pico

## Libraries
 * Install the TFT_eSPI2 library provided in the TFT_eSPI2 folder
 
## Build
 * Pi Pico 2
 * CPU Speed: 250Mhz
 * Optimize: -O3
 * USB Stack: No USB
 * Flash Size: 4MB (no FS)

## Some history
 * 0.01.240 start with MBPTRX code
 * 0.02.240 adjust S meter values
 * 0.03.240 set CW to max gain
 * 0.04.240 improved CESSB
 * 0.05.250 set 250 (not using PCM1808 now)
 * 0.06.250 improved spectrum DC removal
 * 0.07.250 improved AM detection
 * 0.08.250 simplify SSB/DGL/FT8 demod
 * 0.09.250 improved noise blanker
 * 0.10.250 improved s-meter
 * 0.11.250 improved sensitivity (AGC)
 * 0.12.250 notch filter
 * 1.0.250 move DSP to core 1
 * 1.1.250 AM s-meter adjust
 * 1.2.250 s-meter colour
 * 1.3.250 SWR/power meter enhanced
 * 1.4.250 fix spectrum bleed
 * 1.5.250 reduce spectrum stack usage
 * 1.6.250 move mode Auto
 * 1.7.250 add 10000 to quick step
 * 1.8.250 fix frequency step
 * 1.9.250 FT8 hashtable bounds
 * 2.0.250 include TFT_eSPI2 library
 * 2.1.250 add popups
 * 2.2.250 about menu
 * 2.3.250 FT8 AGC display
 * 2.4.250 set all defaults
 * 2.5.250 FT8 auto calibration
 * 2.6.250 FT8 S9 at 80%
