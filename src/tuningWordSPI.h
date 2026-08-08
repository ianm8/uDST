// tuningWordSPI.h
//
// Write-only 32-bit tuning-word writer for the FPGA's spi_tw_slave (mode 0,
// MSB-first, CS active low). Uses the RP2350's hardware SPI -- no PIO needed,
// since there's no MISO round trip to worry about and writes are infrequent
// (tens to low hundreds per second even with GFSK shaping).
//
// The SPI instance is passed in by reference, so this works on EITHER hardware
// SPI block:
//   - SPI  == SPI0
//   - SPI1 == SPI1
// On the RP2350 the GPIO chosen for SCK/MOSI fixes which block is used -- e.g.
// GP10/GP11 are SPI1, so you must pass SPI1 here and call SPI1.setSCK()/setTX().
// CS is a plain GPIO (any pin), toggled manually around each transfer, because
// the FPGA slave has only sck/mosi/cs_n (no MISO); the peripheral's RX pin can
// be left unconnected and its returned bytes ignored.
//
// Order of operations in setup():
//   spiBus.setSCK(sck_pin);
//   spiBus.setTX(mosi_pin);
//   tw.begin();            // calls spiBus.begin() internally
#pragma once
#include <SPI.h>

class TuningWordSPI
{
  public:
    // spi:    which hardware SPI instance to use (SPI or SPI1)
    // cs_pin: any free GPIO; CS is bit-banged around the transfer
    // sck_hz: write-only, so no MISO settle constraint like the IQ bus.
    //         8 MHz is a comfortable default; this path has lots of margin.
    TuningWordSPI(SPIClassRP2040 &spi, uint8_t cs_pin, uint32_t sck_hz = 8000000)
      : spi_(spi), cs_pin_(cs_pin), settings_(sck_hz, MSBFIRST, SPI_MODE0) {}

    void begin()
    {
      pinMode(cs_pin_, OUTPUT);
      digitalWrite(cs_pin_, HIGH);   // CS idle high (deasserted)
      spi_.begin();
    }

    // Blocking write of a 32-bit tuning word, MSB-first, framed by CS.
    void write(uint32_t tuning_word)
    {
      uint8_t buf[4] =
      {
        (uint8_t)(tuning_word >> 24),
        (uint8_t)(tuning_word >> 16),
        (uint8_t)(tuning_word >> 8),
        (uint8_t)(tuning_word)
      };
      spi_.beginTransaction(settings_);
      digitalWrite(cs_pin_, LOW);
      spi_.transfer(buf, 4); // full-duplex; buf[] overwritten with
                             // (unused) MISO data on return
      digitalWrite(cs_pin_, HIGH);
      spi_.endTransaction();
    }

  private:
    SPIClassRP2040 &spi_;
    uint8_t         cs_pin_;
    SPISettings     settings_;
};