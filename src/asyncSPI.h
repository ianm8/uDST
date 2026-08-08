// asyncSPI.h
//
// 32-bit SPI master on the RP2350 PIO, mode 0, MSB-first, CS framed.
// Designed to match the FPGA spi_iq_slave / spi_tw_slave:
//   - mode 0: MOSI driven while SCK low, both sides sample on SCK rising
//   - MSB first, 32-bit frames, CS active low
//
// The FPGA oversamples SCK in its 80 MHz domain, so keep SCK conservative
// (~6 MHz). This driver sets the PIO clock divider from sck_hz so you get the
// rate you ask for instead of sys_clk/4.
//
// Usage: one transfer per data_ready strobe. doTransfer() is pipelined --
// the value it returns is the result of the PREVIOUS transfer (one sample
// of latency), which matches the FPGA's RX hold register.
#pragma once
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

// ---------------------------------------------------------------------------
// PIO program. 5 instructions per bit loop (3..6 plus the side-set settle nop)
// so SCK period = 5 PIO cycles. An extra "nop side 1" is inserted before the
// "in" to give the FPGA's oversampled MISO time to settle after SCK falling
// before we sample it.
//
//   0: pull   block      side 0   ; wait for a 32-bit word, CS high (idle)
//   1: set    pins,0     side 0   ; assert CS (low), SCK low
//   2: set    x,31       side 0   ; 32-bit counter
//   3: out    pins,1     side 0   ; drive MOSI bit, SCK low
//   4: nop               side 1   ; SCK rising edge
//   5: nop               side 1   ; settle: hold SCK high one extra cycle
//   6: in     pins,1     side 1   ; sample MISO, SCK high
//   7: jmp    x--,3      side 0   ; SCK falling edge, next bit
//   8: set    pins,1     side 0   ; deassert CS (high)
//   9: jmp    0          side 0   ; wrap
// ---------------------------------------------------------------------------
#define spi_master_32_cs_wrap_target 0
#define spi_master_32_cs_wrap        9

static const uint16_t spi_master_32_cs_program_instructions[] =
{
          //     .wrap_target
  0x80a0, //  0: pull   block           side 0
  0xe000, //  1: set    pins, 0         side 0
  0xe03f, //  2: set    x, 31           side 0
  0x6001, //  3: out    pins, 1         side 0
  0xb042, //  4: nop                    side 1
  0xb042, //  5: nop                    side 1   (extra MISO settle)
  0x5001, //  6: in     pins, 1         side 1
  0x0043, //  7: jmp    x--, 3          side 0
  0xe001, //  8: set    pins, 1         side 0
  0x0000, //  9: jmp    0               side 0
          //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program spi_master_32_cs_program =
{
  .instructions = spi_master_32_cs_program_instructions,
  .length = 10,
  .origin = -1,
};

static inline pio_sm_config spi_master_32_cs_program_get_default_config(uint offset)
{
  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, offset + spi_master_32_cs_wrap_target, offset + spi_master_32_cs_wrap);
  sm_config_set_sideset(&c, 1, false, false);
  return c;
}
#endif

class AsyncSPI
{
  public:
    AsyncSPI(PIO pio, uint sm, uint cs_pin, uint sck_pin, uint mosi_pin, uint miso_pin, float sck_hz);
    uint32_t doTransfer(const uint32_t tx);

  private:
    PIO  pio_;
    uint sm_;
    uint cs_pin_;
    uint sck_pin_;
    uint mosi_pin_;
    uint miso_pin_;
};

inline AsyncSPI::AsyncSPI(PIO pio, uint sm, uint cs_pin, uint sck_pin,
                          uint mosi_pin, uint miso_pin, float sck_hz)
    : pio_(pio), sm_(sm), cs_pin_(cs_pin), sck_pin_(sck_pin),
      mosi_pin_(mosi_pin), miso_pin_(miso_pin)
{
  // Load program and build the config (this 'c' was missing before).
  uint offset = pio_add_program(pio_, &spi_master_32_cs_program);
  pio_sm_config c = spi_master_32_cs_program_get_default_config(offset);

  // CS as a normal GPIO, deasserted, before PIO drives it.
  gpio_init(cs_pin_);
  gpio_set_dir(cs_pin_, GPIO_OUT);
  gpio_put(cs_pin_, 1);                 // CS idle high
  pio_gpio_init(pio_, cs_pin_);

  // Hand the other pins to the PIO.
  pio_gpio_init(pio_, sck_pin_);
  pio_gpio_init(pio_, mosi_pin_);
  pio_gpio_init(pio_, miso_pin_);

  // Pin groups: SET drives CS, OUT drives MOSI, IN reads MISO, side-set is SCK.
  sm_config_set_set_pins(&c, cs_pin_, 1);
  sm_config_set_out_pins(&c, mosi_pin_, 1);
  sm_config_set_in_pins(&c, miso_pin_);
  sm_config_set_sideset_pins(&c, sck_pin_);

  // MSB-first 32-bit frames: shift LEFT, auto pull/push at 32 bits.
  // (PIO default is shift-right / LSB-first, which would bit-reverse the word.)
  sm_config_set_out_shift(&c, /*shift_right=*/false, /*autopull=*/true,  /*pull_thresh=*/32);
  sm_config_set_in_shift (&c, /*shift_right=*/false, /*autopush=*/true,  /*push_thresh=*/32);

  // SCK rate. The bit loop is 5 PIO cycles per SCK period, so divide the
  // system clock by sck_hz * 5 to get the requested SCK frequency.
  float div = (float)clock_get_hz(clk_sys) / (sck_hz * 5.0f);
  if (div < 1.0f) div = 1.0f;
  sm_config_set_clkdiv(&c, div);

  // Pin directions: outputs = CS, MOSI, SCK; input = MISO.
  pio_sm_set_consecutive_pindirs(pio_, sm_, cs_pin_,   1, true);
  pio_sm_set_consecutive_pindirs(pio_, sm_, mosi_pin_, 1, true);
  pio_sm_set_consecutive_pindirs(pio_, sm_, sck_pin_,  1, true);
  pio_sm_set_consecutive_pindirs(pio_, sm_, miso_pin_, 1, false);

  // Init and enable last.
  pio_sm_init(pio_, sm_, offset, &c);
  pio_sm_set_enabled(pio_, sm_, true);
}

// Non-blocking, pipelined transfer:
//   - queues tx if the TX FIFO has room
//   - returns the previous frame's rx if the RX FIFO has data, else 0
// Call once per data_ready strobe. The returned rx lags the pushed tx by one
// transfer, matching the FPGA's one-sample RX hold.
inline uint32_t AsyncSPI::doTransfer(const uint32_t tx)
{
  if (!pio_sm_is_tx_fifo_full(pio_, sm_))
  {
    pio_sm_put(pio_, sm_, tx);
  }

  if (!pio_sm_is_rx_fifo_empty(pio_, sm_))
  {
    return pio_sm_get(pio_, sm_);
  }

  return 0u;
}
