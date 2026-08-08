#ifndef DSP_H
#define DSP_H

#include "filter.h"
#include "notch.h"
#include "util.h"

namespace DSP
{
  static notch_t notch;
  volatile static float agc_peak = 0.0f;

  static const int32_t __not_in_flash_func(get_agc)(void)
  {
    return (int32_t)agc_peak;
  }

  static const int16_t __not_in_flash_func(agc)(const float in)
  {
    // limit gain to max of 200 (46db)
    static const float max_gain = 200.0f;
    // decay about 10dB per second
    static const float k = 0.99996f;

    const float magnitude = fabsf(in);
    if (magnitude>agc_peak)
    {
      agc_peak = magnitude;
    }
    else
    {
      agc_peak *= k;
    }

    // trap issues with low values
    if (agc_peak<1.0f) return (int16_t)(in * max_gain);

    // set maximum gain possible for 12 bit DAC
    const float m = 2047.0f/agc_peak;
    return (int16_t)(in*fminf(m,max_gain));
  }

  static const uint8_t __not_in_flash_func(smeter)(const uint32_t f)
  {
    // S9 = -73dBm = 141uV PP
    // need to return 10 for S9
    volatile const float comp_agc_peak = agc_peak;
    if (f < 10'000'000ul)
    {
      static constexpr float S0_sig = 10.0f;
      static constexpr float S9_sig = 200.0f;
      static constexpr float S9p_sig = 4000.0f;
      static constexpr uint32_t S9_from_min = (uint32_t)(log10f(S0_sig) * 1024.0f);
      static constexpr uint32_t S9_from_max = (uint32_t)(log10f(S9_sig) * 1024.0f);
      static constexpr uint32_t S9_min = 1ul;
      static constexpr uint32_t S9_max = 10ul;
      static constexpr uint32_t S9p_from_min = (uint32_t)(log10f(S9_sig) * 1024.0f);
      static constexpr uint32_t S9p_from_max = (uint32_t)(log10f(S9p_sig) * 1024.0f);
      static constexpr uint32_t S9p_min = 11ul;
      static constexpr uint32_t S9p_max = 15ul;
      if (comp_agc_peak < 1.0f)
      {
        return 0u;
      }
      const uint32_t log_peak = (uint32_t)(log10f(comp_agc_peak) * 1024.0f);
      if (comp_agc_peak>S9_sig)
      {
        return (uint8_t)UTIL::map(log_peak,S9p_from_min,S9p_from_max,S9p_min,S9p_max);
      }
      return (uint8_t)UTIL::map(log_peak,S9_from_min,S9_from_max,S9_min,S9_max);
    }
    static constexpr float S0_sig = 2.0f;
    static constexpr float S9_sig = 200.0f;
    static constexpr float S9p_sig = 4000.0f;
    static constexpr uint32_t S9_from_min = 0;
    static constexpr uint32_t S9_from_max = (uint32_t)(log10f(S9_sig  / S0_sig) * 1024.0f);
    static constexpr uint32_t S9_min = 1ul;
    static constexpr uint32_t S9_max = 10ul;
    static constexpr uint32_t S9p_from_min = S9_from_max;
    static constexpr uint32_t S9p_from_max = (uint32_t)(log10f(S9p_sig / S0_sig) * 1024.0f);
    static constexpr uint32_t S9p_min = 11ul;
    static constexpr uint32_t S9p_max = 15ul;
    if (comp_agc_peak < S0_sig)
    {
      return 0u;
    }
    const uint32_t log_peak = (uint32_t)(log10f(comp_agc_peak * (1.0f / S0_sig)) * 1024.0f);
    if (comp_agc_peak>S9_sig)
    {
      return (uint8_t)UTIL::map(log_peak,S9p_from_min,S9p_from_max,S9p_min,S9p_max);
    }
    return (uint8_t)UTIL::map(log_peak,S9_from_min,S9_from_max,S9_min,S9_max);
  }

  static void __not_in_flash_func(noise_blanker)(float &I, float &Q, const uint8_t level)
  {
    // Wideband detection with pulse-width qualification.
    //
    // Raw (unfiltered) I/Q amplitude is compared against a slow envelope
    // average. Over-threshold runs no longer than MAX_BLANK_RUN samples are
    // qualified as impulses and removed by linear interpolation across the
    // event (plus GUARD_PRE samples of pre-ring) inside a delay line.
    // Longer runs are declared signal (e.g. voice peaks of a nearby strong
    // station) and pass untouched while the envelope average adapts to them.
    //
    // Adds NB_LEN samples (64 / 31250 = ~2 ms) of RX latency while enabled;
    // zero latency when off. FILTER::lpf_nb1/lpf_nb2 are no longer needed.

    static constexpr uint32_t NB_LEN = 64u;          // power of two, >= MAX_BLANK_RUN + GUARD_PRE + 2
    static constexpr uint32_t NB_MASK = NB_LEN - 1u; // buffer index control
    static constexpr uint32_t MAX_BLANK_RUN = 32u;   // ~1ms at 31250 Hz; longer = signal, not impulse
    static constexpr uint32_t GUARD_PRE = 2u;        // pre-ring guard (DDC FIR smears impulses slightly)
    static constexpr float alpha = 0.002f;           // ~16ms at 31250 Hz

    // thresholds, number of times greater than
    // average to trigger a blanking event
    static const float thresholds[] =
    {
      0.0f, // level 0 — unused (returns early)
      9.0f, // level 1 — only very large spikes
      7.0f, // level 2 — conservative
      5.0f, // level 3 — moderate
      3.0f, // level 4 — fairly aggressive
      2.0f  // level 5 — maximum blanking
    };

    static struct
    {
      float bufI[NB_LEN];
      float bufQ[NB_LEN];
      float avg_amp;
      uint32_t w;   // delay line write index
      uint32_t run; // length of current over-threshold run
      uint32_t active;
    } s = {0};

    if ((level == 0u) || (level > 5u))
    {
      s.active = 0u;     // bypass; state and delay line refill on re-enable
      return;
    }
    const float threshold = thresholds[level];

    if (s.active == 0u)
    {
      for (uint32_t i = 0u; i < NB_LEN; i++)
      {
        s.bufI[i] = I;
        s.bufQ[i] = Q;
      }
      s.avg_amp = 0.01f;
      s.w = 0u;
      s.run = 0u;
      s.active  = 1u;
    }

    // wideband instantaneous amplitude (fast L1 estimate)
    const float amp = fabsf(I) + fabsf(Q);

    if (amp > threshold * s.avg_amp)
    {
      if (s.run <= MAX_BLANK_RUN)
      {
        s.run++;   // clamps at MAX_BLANK_RUN + 1
      }
      if (s.run > MAX_BLANK_RUN)
      {
        // too long for an impulse — it's a signal, track it
        s.avg_amp += alpha * (amp - s.avg_amp);
      }
    }
    else
    {
      if ((s.run > 0u) && (s.run <= MAX_BLANK_RUN))
      {
        // qualified impulse: rewrite the buffered event by linear
        // interpolation between the last good sample before it and
        // the current (below threshold) sample
        uint32_t gap = s.run + GUARD_PRE;
        if (gap > NB_LEN - 2u)
        {
          gap = NB_LEN - 2u;
        }
        const uint32_t ia = (s.w - gap - 1u) & NB_MASK;  // last good sample
        const float aI  = s.bufI[ia];
        const float aQ  = s.bufQ[ia];
        const float stepI = (I - aI) / (float)(gap + 1u);
        const float stepQ = (Q - aQ) / (float)(gap + 1u);
        for (uint32_t p = 1u; p <= gap; p++)
        {
          const uint32_t idx = (s.w - (gap + 1u - p)) & NB_MASK;
          s.bufI[idx] = aI + stepI * (float)p;
          s.bufQ[idx] = aQ + stepQ * (float)p;
        }
      }

      // track signal envelope
      s.avg_amp += alpha * (amp - s.avg_amp);
      s.run = 0u;
    }

    // delay line: emit the sample from NB_LEN calls ago, store the current one
    const float outI = s.bufI[s.w];
    const float outQ = s.bufQ[s.w];
    s.bufI[s.w] = I;
    s.bufQ[s.w] = Q;
    s.w = (s.w + 1u) & NB_MASK;
    I = outI;
    Q = outQ;
  }

  static const int16_t __not_in_flash_func(process_ssb)(
      const int16_t in_i,
      const int16_t in_q,
      const uint32_t jnr_level,
      const uint8_t bw,
      const uint8_t nb_level)
  {
    // remove DC
    float ii = FILTER::dc1f((float)in_i / 32768.0f);
    float qq = FILTER::dc2f((float)in_q / 32768.0f);

    // noise blanker
    noise_blanker(ii,qq,nb_level);

    // phase shift IQ +/- 45
    const float p45 = FILTER::fap1f(ii);
    const float n45 = FILTER::fap2f(qq);

    // reject image
    const float ssb = p45 - n45;

    // LPF
    const float audio_lpf = FILTER::bwf[bw](ssb);

    // HPF
    const float audio_raw = FILTER::hpf_200f(audio_lpf);

    // notch
    const float audio_notch = notch.process(audio_raw);

    // JNR
    const float audio_out = FILTER::jnr(audio_notch,jnr_level);

    // AGC returns 12 bit value
    return agc(audio_out * 32768.0f);
  }

  static const int16_t __not_in_flash_func(process_dig)(const int16_t in_i,const int16_t in_q)
  {
    // remove DC
    float ii = FILTER::dc1f((float)in_i / 32768.0f);
    float qq = FILTER::dc2f((float)in_q / 32768.0f);

    // phase shift IQ +/- 45
    const float p45 = FILTER::fap1f(ii);
    const float n45 = FILTER::fap2f(qq);

    // reject image
    const float ssb = p45 - n45;

    // LPF
    const float audio_out = FILTER::lpf_3000f_rx(ssb);

    // AGC returns 12 bit value
    return agc(audio_out * 32768.0f);
  }

  static const float __not_in_flash_func(process_ft8)(const int16_t in_i,const int16_t in_q)
  {
    // remove DC
    float ii = FILTER::dc1f((float)in_i / 32768.0f);
    float qq = FILTER::dc2f((float)in_q / 32768.0f);

    // phase shift IQ +/- 45
    const float p45 = FILTER::fap1f(ii);
    const float n45 = FILTER::fap2f(qq);

    // reject image
    const float ssb = p45 - n45;

    // LPF
    const float audio_out = FILTER::lpf_3000f_rx(ssb);

    // AGC for s-meter
    agc(audio_out * 32768.0f);

    // return raw float value
    return audio_out;
  }

  static const int16_t __not_in_flash_func(process_cw)(const int16_t in_i,const int16_t in_q,float &sig)
  {
    // remove DC
    const float ii = FILTER::dc1f((float)in_i / 32768.0f);
    const float qq = FILTER::dc2f((float)in_q / 32768.0f);

    // phase shift IQ +/- 45
    const float p45 = FILTER::fap1f(ii);
    const float n45 = FILTER::fap2f(qq);

    // reject image
    const float ssb = p45 - n45;

    // BPF for CW
    sig = FILTER::bpf_700f(ssb);

    // AGC returns 12 bit value
    return agc(sig * 32768.0f);
  }

  static const int16_t __not_in_flash_func(process_am)(const int16_t in_i, const int16_t in_q, const uint32_t jnr_level)
  {
    // AM envelope detector at zero IF with carrier-referenced AGC.
    // The DC component of the envelope IS the carrier level A, so a slow
    // LPF of the envelope gives a gain reference that ignores modulation
    // entirely - no syllabic pumping - while still tracking QSB.

    // AGC settings
    static constexpr float max_gain = 100.0f;

    // one-pole corner ~ alpha*fs/(2*pi)
    // = ~5 Hz at fs = 31.25 kHz;
    // scale for your RX sample rate
    static constexpr float agc_alpha = 0.001f; 

    // DAC amplitude at m = 1.0; leaves
    // ~+28% positive-peak headroom
    // below the 12-bit limit of 2047
    static constexpr float target = 1600.0f;

    // carrier level
    volatile static float agc_carrier = 0.0f;

    // channel filter, pre-detection: two instances of the SAME design
    // (identical coefficients, separate state per instance)
    const float ii = FILTER::lpf_fs8f_rx1((float)in_i / 32768.0f);
    const float qq = FILTER::lpf_fs8f_rx2((float)in_q / 32768.0f);

    // exact envelope at zero IF (single VSQRT.F32 on the RP2350 FPU)
    const float envelope = sqrtf(ii * ii + qq * qq);

    // carrier tracker: slow one-pole LPF of the envelope. The fmaxf
    // floor at envelope/2 is a fast-attack guard: a strong signal
    // appearing suddenly is caught to within 6 dB immediately instead
    // of blasting through for the LPF settling time. For m <= 1 the
    // floor never exceeds the true carrier, so it cannot cause pumping;
    // the slow tracker then converges the last few dB smoothly.
    const float agc_envelope = envelope * 32768.0f;
    agc_carrier += (agc_envelope - agc_carrier) * agc_alpha;
    if (agc_envelope > 4.0f * agc_carrier)
    {
      agc_carrier = agc_envelope * 0.5f;
    }
    agc_peak = agc_carrier * 2.0f;

    // gain from carrier level: m = 1.0 maps to 'target' at the DAC
    float gain = max_gain;
    if (agc_carrier > 1.0f)
    {
      gain = target / agc_carrier;
    }
    gain = fminf(gain, max_gain);

    // carrier removal, audio shaping, noise reduction as before
    const float audio_raw = FILTER::lpf_3000f(FILTER::dcf(envelope));
    const float audio_out = FILTER::jnr(audio_raw, jnr_level);
    return (int16_t)(audio_out * 32768.0f * gain);
  }

  static const uint32_t __not_in_flash_func(get_mic_peak_level)(const int16_t mic_in)
  {
    static const uint32_t MIC_LEVEL_DECAY_RATE = 50ul;
    static const uint32_t MIC_LEVEL_HANG_TIME = 500ul;
    static uint32_t mic_peak_level = 0;
    static uint32_t mic_level_update = 0;
    static uint32_t mic_hangtime_update = 0;
    const uint32_t now = millis();
    const uint32_t mic_level = abs(mic_in)>>5;
    if (mic_level>mic_peak_level)
    {
      mic_peak_level = mic_level;
      mic_level_update = now + MIC_LEVEL_DECAY_RATE;
      mic_hangtime_update = now + MIC_LEVEL_HANG_TIME;
    }
    else
    {
      if (now>mic_hangtime_update)
      {
        if (now>mic_level_update)
        {
          if (mic_peak_level) mic_peak_level--;
          mic_level_update = now + MIC_LEVEL_DECAY_RATE;
        }
      }
    }
    return mic_peak_level;
  }

  static float __not_in_flash_func(mic_process)(const float sample, const uint8_t level)
  {
    // use tanh for soft clipping

    if (level == 0) return sample;
    if (level > 5) return sample;

    // drive values mapped to processing levels 1–5
    // level 0 = bypass (filter state is left untouched)
    static const float DRIVE_TABLE[6] =
    {
      0.0f,   // 0 — bypass
      1.5f,   // 1 — subtle warmth
      3.0f,   // 2 — light SSB processing
      5.0f,   // 3 — typical SSB (good starting point)
      7.5f,   // 4 — aggressive
      12.0f,  // 5 — contest/heavy
    };
  
    const float drive = DRIVE_TABLE[level];
    const float clipped = tanhf(sample * drive) / tanhf(drive);
    return clipped;
  }

  static void __not_in_flash_func(cessb)(float& ii, float& qq)
  {
    ii *= 1.5f;
    qq *= 1.5f;
    const float m2 = ii*ii + qq*qq;
    if (m2 > 1.0f)
    {
      const float inv = 1.0f / sqrtf(m2);
      ii *= inv;
      qq *= inv;
    }
    ii = FILTER::lpf_2600if_tx(ii);
    qq = FILTER::lpf_2600qf_tx(qq);
  }

  static const void __not_in_flash_func(process_mic)(
    const int16_t s,
    int16_t &out_i,
    int16_t &out_q,
    const float mic_gain,
    const uint8_t proc_level,
    const bool cessb_on)
  {
    // input is 12 bits
    // convert to float
    // remove Mic DC
    // mic processing
    // 2400 LPF 
    // phase shift I
    // phase shift Q
    // convert to int
    // output is 10 bits
    const float ac_sig = FILTER::dcf(((float)s)*(1.0f/2048.0f)) * mic_gain;
    const float mic_proc = mic_process(ac_sig,proc_level);
    const float mic_sig = FILTER::lpf_2600f_tx(mic_proc);
    float ii = FILTER::fap1f(mic_sig);
    float qq = FILTER::fap2f(mic_sig);
    if (cessb_on) cessb(ii,qq);
    out_i = (int16_t)(ii * 512.0f);
    out_q = (int16_t)(qq * 512.0f);
  }

  static const void __not_in_flash_func(process_dig)(const int16_t s,int16_t &out_i,int16_t &out_q)
  {
    // input is 12 bits
    // convert to float
    // remove DC
    // 3000 Hz LPF 
    // phase shift I
    // phase shift Q
    // convert to int
    // output is 10 bits
    const float ac_sig = FILTER::dcf(((float)s)*(1.0f/2048.0f));
    const float lp_sig = FILTER::lpf_3000f_tx(ac_sig);
    float ii = FILTER::fap1f(lp_sig);
    float qq = FILTER::fap2f(lp_sig);
    out_i = (int16_t)(ii * 512.0f);
    out_q = (int16_t)(qq * 512.0f);
  }
}

#endif