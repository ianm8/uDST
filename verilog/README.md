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
