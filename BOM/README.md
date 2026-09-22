# BOM Notes

## Crystal Oscillator
Optional but highly recommended, use a FT3MHUPM24.0-T1 24MHz TCXO to replace the 27MHz oscillator on the BRS-100.

## Resistors
All SM resistors are 1% tolerance, 1206 thick film of at least 125mW except R59 which should be at least 2.4W.

## Capacitors
All SM capacitors are 1206 NP0 or C0G except for the 22uF (25V) and 100uF capacitors. Try to get capacitors of at least 100V for the low-pass filters. Because the audio amp is powered from it's own 5V regulator, the 100uF cap could probably be substituted with a 22uF (but I have not tried it).

## Inductors
All SM inductors are 1206 (or 0806) for as high SRF as possible (ie, greater than 30MHz), except for L20 and L26 which has a specific part number (1206F-100K-01).

## Transformers
T3 and T4 can be SHO-301 if you can get them, they work well, I have tested them. T5 can be T-622-KK81+ from Mini Circuits, I have tried it and it works well.

# PA Bias Adjustment
1. Make sure you have a dummy load connected
2. Set the bias trimpots to mid position
3. Apply power using a power supply set to 13.8V and current max 1.5A
4. Make sure the mode is set for SSB
5. Press PTT and adjust the trimpots for less than 1V on the gate of the MOSFETs
6. Note the current
7. Adjust one trimpot for an additional 250ma of current draw
8. Note the current
9. Adjust the other trimpot for another 250ma of current draw
10. No need to be exact
