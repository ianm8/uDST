# Important Note

I replaced the stock 27MHz clock on the dev board with a 24MHz TCXO. See the uDST81 folder for the stock 27MHz clock version (**but it is untested in production**). The part I used was from Mouser *FT3MHUPM24.0-T1*. Also, DigiKey has the exact same part.

# FPGA Programming

## Project

In GOWIN, just create a new project with device GW1NR-LV9QN88PC6/I5, add these files, and build.

**Note: need to set dual function pin as follows:**
* goto Project, Configuration
* Place & Route, Dual-Purpose Pin
* Check: Use SSPI as regular IO

## Device Programming
* start MSYS2
* openFPGALoader --detect
* openFPGALoader -b tangnano9k -f uDST.fs
