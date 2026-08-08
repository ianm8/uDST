## Libraries
 * Install the TFT_eSPI2 library provided in the **release** files - coming soon
 
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
