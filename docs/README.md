# FPGA Details

## DDC Description

**DDC / receive.** The 12-bit ADC gives an ideal SNR of 6.02×12 + 1.76 = 74.0 dB, but that noise is spread across the full 40 MHz Nyquist band — a density of −74 − 10·log₁₀(40 MHz) = −150 dBFS/Hz. The DDC doesn't remove any noise; it just discards the 39.97 MHz of spectrum you don't want. After mixing to complex baseband and decimating by 2560, the output holds a 31.25 kHz slice, so the retained noise is −150 + 10·log₁₀(31 250) = −105 dBFS. The processing gain is therefore 10·log₁₀(40 MHz / 31.25 kHz) = 10·log₁₀(1280) ≈ **31.1 dB**, split as 24.1 dB from the CIC (÷512 → exactly 4.0 bits) and 7.0 dB from the FIR (÷5 → 1.2 bits). That's 5.2 bits of genuine extra resolution: the 12-bit converter behaves like a ~17-bit converter in the 31.25 kHz channel.

The word-growth bookkeeping matches this exactly. The 3rd-order CIC has a DC gain of 512³ = 2²⁷ (why the 48-bit integrators: worst case 16 + 27 = 43 bits). The ≫22 trim keeps 2⁷⁻²² wait — keeps 2²⁷⁄2²² = ×32, i.e. 5 bits of that gain — and those 5 bits are precisely the 5.2 bits the decimation earned, so the 16-bit I/Q word is filled with real information rather than padded LSBs. It's also why ≫22 calibrated so well: quadrature mixing halves a real carrier's amplitude (A·cosωt → A/2 complex envelope), so the net carrier gain is ×16, and a full-scale ADC input (±2047) lands at ±32 752 — 16-bit full scale within 0.004 dB. The 16-bit interface floor (−98 dBFS) sits ~7 dB above the −105 dBFS theoretical floor, so the pipe, not the ADC, is the final limit — and 98 dB in 31.25 kHz is far beyond anything HF propagation delivers. Firmware channel filtering to a ~2.4 kHz SSB bandwidth adds a further 10·log₁₀(31 250⁄2400) ≈ 11 dB on top.

## DDC Block Diagram
![DDC Block Diagrem](https://github.com/ianm8/uDST/blob/main/docs/uDST_DDC25_block_diagram.png?raw=true)

## DUC Description

**DUC / transmit.** The same bookkeeping runs in reverse, arranged to be unity gain end to end: zero-stuffing ↑5 spreads each sample's energy over five outputs (÷5), which the 63-tap FIR's ≈×5 passband gain restores; the interpolating CIC has DC gain R^(N−1) = 512² = 2¹⁸, cancelled exactly by SCALE_SHIFT = 18; and the Q15 mixer's ≫15 returns to integer scale — so 10-bit TX samples map cleanly onto the 10-bit DAC. Oversampling pays off here too: the DAC's 62 dB SNR is spread over 40 MHz, a quantization-noise density of −138 dBFS/Hz, keeping the digital contribution to close-in transmit noise well below what the analog chain sets.

## DUC Block Diagram
![DUC Block Diagrem](https://github.com/ianm8/uDST/blob/main/docs/uDST_DUC18_block_diagram.png?raw=true)
