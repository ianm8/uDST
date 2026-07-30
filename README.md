# uDST
A direct sampling SDR QRP transceiver.

## Some Specifications
 * SSB, CW, FT8, Other digital modes using vox
 * CESSB
 * 80m - 10m, nominally 5W
 * 3.5MHz - 30MHz SWL
 * 30 KHz spectrum display
 * Noise reduction
 * Noise blanker

Here is a block diagram:

![alt text](https://github.com/ianm8/uDST/blob/main/docs/uDST.png?raw=true)


## Advantages of Direct Sampling
**The signal path collapses to almost nothing.** The uDST's entire analog chain is a transformer, one MMIC, and an anti-alias filter ahead of the ADC — after that, everything is arithmetic. A Tayloe rig needs a quadrature LO chain (usually a synthesizer plus divide-by-4), the switching detector itself, matched integrator/op-amp stages on both I and Q, and an audio codec. A superhet needs one or more mixers, LO synthesizers, IF amplifier strips, and crystal or roofing filters, all of which must be aligned and gain-distributed. Every analog stage removed is a stage that can't drift, mismatch, intermodulate, or pick up hum.

**Quadrature is perfect by construction.** This is arguably the single biggest win over the Tayloe architecture. In a phasing rig, opposite-sideband rejection is set by how well two analog channels match in gain and phase across the audio passband — typically 40–50 dB from component tolerances, more only with calibration, and it wanders with temperature. In the uDST, the "mixer" is an NCO driving sine and cosine that are exactly 90° apart to arithmetic precision, and I and Q travel through literally identical CIC and FIR logic. Image rejection is effectively unlimited; there is no calibration and nothing to drift.

**The zero-IF pathologies vanish.** The Tayloe detector mixes the signal of interest down to DC, where it collides with op-amp offsets, 1/f flicker noise, LO leakage, mains hum, and second-order intermod products that all land in the passband. That's why the MBPTRX's AM detector had to run at FS/4 rather than at baseband. The uDST's DDC also produces a zero-IF result, but the mixing happens numerically at 80 MSPS — there's no analog circuitry at baseband to inject any of it.

**Spurious responses are few and predictable.** A switching detector responds at odd harmonics of the LO — a strong signal at 3× the tuned frequency gets in only ~9.5 dB down, which is why Tayloe rigs need decent band-pass preselection. Superhets add image responses, half-IF spurs, IF breakthrough, and birdies wherever LO harmonics beat together. A direct-sampling receiver's spurious responses are aliases of the one fixed Nyquist boundary, handled once by one fixed anti-alias filter, plus ADC spurs that are characterised on the datasheet. Nothing moves as you tune.

**One clean fixed clock instead of an agile LO.** Close-in dynamic range in a superhet is often limited by reciprocal mixing from synthesizer phase noise, and a wide-tuning PLL inevitably carries spurs. The direct-sampling equivalent is sample-clock jitter — but a single fixed-frequency crystal clock is about the easiest low-phase-noise source there is. Tuning happens in the NCO with sub-millihertz resolution, instant settling, and zero phase-noise penalty for agility.

**Filtering is done properly, once, in numbers.** The 255-tap FIR gives near-brick-wall selectivity with linear phase — no crystal-filter ripple, blow-by, group-delay distortion at the edges, aging, or cost. Bandwidth is a coefficient set, not a component. Even DSP-at-the-back superhets can't escape this: their DSP only sees what survives the analog roofing filter, so close-in strong-signal performance is still hostage to that one expensive crystal filter.

**Processing gain rescues the "only 12 bits" objection.** Decimating from 40 MHz of Nyquist bandwidth down to a 31.25 kHz channel spreads the quantisation noise and buys roughly 10·log₁₀(40 MHz / 31.25 kHz) ≈ 31 dB — about five extra effective bits, so the 12-bit converter behaves like a ~17-bit one in-channel.

**The whole band is available at once.** Full-span waterfall, multiple simultaneous receive slices, wideband noise blanking, diversity — all free once the band is digitised. A Tayloe rig's panadapter is capped at the codec sample rate; a superhet sees one roofing-filter slice at a time.

**Transmit is symmetric.** The DUC synthesises SSB or FT8 directly at RF with mathematically perfect carrier and sideband suppression — no balanced-modulator nulling, no TX filter alignment.

**And it's repeatable.** Performance lives in the Verilog, so unit two behaves exactly like unit one. No trimmers, no alignment procedure.

