// ============================================================================
// radio_top  -- direct-sampling HF SSB/FT8 transceiver, top level
//
// programming
//   start MSYS2
//   openFPGALoader --detect
//   openFPGALoader -b tangnano9k -f C:/GOWIN/uDST-24.fs
//
// Clocking + reset + converter I/O + DDC/DUC cores + two SPI slaves.
// Both SPI slaves OVERSAMPLE the SPI clock in the 80 MHz domain (sck/mosi/cs_n
// are 2-flop synchronised and edge-detected), so there is no second clock
// domain and no async-clock constraint. The trade-off: keep the Pico's SPI
// clock modest -- a few MHz -- so a bit period spans several 80 MHz cycles and
// MISO is stable before the master samples it. At 31.25 kHz x 32 bits the data
// rate is ~1 Mbit/s, so a 2-6 MHz SPI clock is ample.
//
// SPI format (both buses): mode 0 (CPOL=0, CPHA=0), 32-bit frames, MSB first,
// CS active low. Tuning bus is write-only (Pico -> FPGA). IQ bus carries one
// {I[15:0], Q[15:0]} sample per frame: RX shifts the DDC sample out on MISO,
// TX shifts the Pico sample in on MOSI. The Pico should leave a few 80 MHz
// cycles between asserting CS and the first SCK edge.
//
// Pin budget (32 GPIO; sys_clk on the crystal/oscillator pin):
//   adc 12 + dac 10 + clk80 1 + tx_rx 1 + spi_tw 3 + spi_iq 4 + data_ready 1
// leds_n[1:6] are on the board's separate dedicated LED header (its own
// 1.8V VCCIO bank) -- they are NOT part of the 32 GPIO above.
//
// Project sources: this file, DDC20.v, DUC18.v (delete its mac16_reg -- DDC20
// provides it), pll_20_80.v, sin_lut.hex, fir_rx255_coeffs.hex. ODDR is a Gowin
// primitive.
// ============================================================================
module radio_top (
    input  wire        sys_clk,        // 20 MHz crystal/TCXO (clock pin)

    input  wire        tx_rx,          // 0 = RX, 1 = TX

    input  wire signed [11:0] adc_in,  // AD9236, two's complement
    output wire        [9:0]  dac_out,
    output wire        clk80,          // 80 MHz forwarded to converters

    input  wire        spi_tw_sck,
    input  wire        spi_tw_mosi,
    input  wire        spi_tw_cs_n,

    input  wire        spi_iq_sck,
    input  wire        spi_iq_mosi,
    output wire        spi_iq_miso,
    input  wire        spi_iq_cs_n,

    output reg          data_ready,
    output wire [1:6]   leds_n          // active low, 1.8V bank; status scroller
);

    // ---- clocking: 20 MHz -> rPLL -> 80 MHz ----
    wire clk80_int, pll_lock;
    pll_20_80 u_pll (.clkin(sys_clk), .clkout(clk80_int), .lock(pll_lock));

    // ---- internal reset (no pin): hold until lock, sync de-assert ----
    reg [1:0] rst_pipe = 2'b11;
    always @(posedge clk80_int or negedge pll_lock) begin
        if (!pll_lock) rst_pipe <= 2'b11;
        else           rst_pipe <= {rst_pipe[0], 1'b0};
    end
    wire rst = rst_pipe[1];

    // ---- forward clk80 to the converters via DDR output buffer ----
    ODDR u_clk_fwd (.D0(1'b1), .D1(1'b0), .TX(1'b0), .CLK(clk80_int), .Q0(clk80), .Q1());

    // ---- synchronise the static mode select ----
    reg [1:0] txrx_s;
    always @(posedge clk80_int) txrx_s <= {txrx_s[0], tx_rx};
    wire mode_tx = txrx_s[1];

    // ---- ADC capture (IOB register) ----
    reg signed [11:0] adc_r;
    always @(posedge clk80_int) adc_r <= adc_in;

    // ---- tuning word (SPI write-only slave); same dial drives RX and TX ----
    wire [31:0] tuning_word;
    wire        tw_valid;
    spi_tw_slave u_tw (
        .clk(clk80_int), .rst(rst),
        .sck(spi_tw_sck), .mosi(spi_tw_mosi), .cs_n(spi_tw_cs_n),
        .tuning_word(tuning_word), .tw_valid(tw_valid)
    );

    // ---- RX chain: ADC -> DDC -> IQ ----
    wire               rx_iq_valid;
    wire signed [15:0] rx_iq_i, rx_iq_q;
    ddc_top u_ddc (
        .clk80(clk80_int), .rst(rst), .adc_in(adc_r),
        .tw_valid(tw_valid), .tuning_word(tuning_word),
        .iq_valid(rx_iq_valid), .iq_i(rx_iq_i), .iq_q(rx_iq_q)
    );

    // ---- IQ SPI slave: RX shifts DDC IQ out, TX shifts Pico IQ in ----
    wire signed [15:0] tx_iq_i, tx_iq_q;
    wire               tx_iq_valid;
    spi_iq_slave u_iq (
        .clk(clk80_int), .rst(rst),
        .sck(spi_iq_sck), .mosi(spi_iq_mosi), .cs_n(spi_iq_cs_n), .miso(spi_iq_miso),
        .mode_tx(mode_tx),
        .rx_i(rx_iq_i), .rx_q(rx_iq_q), .rx_valid(rx_iq_valid),
        .tx_i(tx_iq_i), .tx_q(tx_iq_q), .tx_valid(tx_iq_valid)
    );

    // ---- TX chain: Pico IQ -> DUC -> DAC. DUC steps on each TX sample. ----
    wire [9:0] duc_dac;
    duc_top u_duc (
        .clk(clk80_int), .rst(rst),
        .in_valid(tx_iq_valid),
        .inI(tx_iq_i), .inQ(tx_iq_q),
        .tuning_word(tuning_word),
        .dac_out(duc_dac)
    );

    // DAC: drive the DUC output in TX, idle (0) in RX. IOB output register.
    reg [9:0] dac_r;
    always @(posedge clk80_int) dac_r <= mode_tx ? duc_dac : 10'd0;
    assign dac_out = dac_r;

    // ---- data_ready: 50% square wave phase-locked to the DDC sample tick ----
    //   high on iq_valid, low after 1280 cycles, until the next iq_valid.
    //   Pico waits high, runs DSP, waits low, repeats.
    localparam [11:0] HB_HALF = 12'd1280;
    reg [11:0] dr_cnt = 12'd0;
    always @(posedge clk80_int) begin
        if (rst) begin
            data_ready <= 1'b0; dr_cnt <= 12'd0;
        end else if (rx_iq_valid) begin
            data_ready <= 1'b1; dr_cnt <= 12'd0;
        end else if (data_ready) begin
            if (dr_cnt == HB_HALF-1) data_ready <= 1'b0;
            else                     dr_cnt <= dr_cnt + 12'd1;
        end
    end

    // ---- status LEDs: single LED scrolling around the ring of 6, wrapping.
    //   RX: step every 2^25 clk80 cycles  = 2.384 Hz  (~419 ms/step)
    //   TX: step every 2^23 clk80 cycles  = 9.537 Hz  (~105 ms/step)
    //   2^25 / 2^23 = 4, matching the 8/2 Hz target ratio exactly, so one
    //   free-running counter serves both rates -- no separate TX divider.
    reg [24:0] led_div;
    always @(posedge clk80_int) begin
        if (rst) led_div <= 25'd0;
        else     led_div <= led_div + 25'd1;
    end
    wire led_tick = mode_tx ? (led_div[22:0] == 23'd0)   // every 2^23
                             : (led_div[24:0] == 25'd0);  // every 2^25

    reg [2:0] led_pos;     // 0..5, wraps
    always @(posedge clk80_int) begin
        if (rst)            led_pos <= 3'd0;
        else if (led_tick)  led_pos <= (led_pos == 3'd5) ? 3'd0 : led_pos + 3'd1;
    end

    // one-hot decode, registered, active low (1.8V bank, common anode)
    reg [1:6] leds_n_r;
    always @(posedge clk80_int) begin
        if (rst) leds_n_r <= 6'b111111;
        else case (led_pos)
            3'd0: leds_n_r <= 6'b111110;
            3'd1: leds_n_r <= 6'b111101;
            3'd2: leds_n_r <= 6'b111011;
            3'd3: leds_n_r <= 6'b110111;
            3'd4: leds_n_r <= 6'b101111;
            default: leds_n_r <= 6'b011111;   // 3'd5
        endcase
    end
    assign leds_n = leds_n_r;

endmodule


// ============================================================================
// spi_tw_slave -- write-only SPI slave (32-bit tuning word, mode 0, MSB first).
// Oversampled in the clk domain; latches the word and pulses tw_valid on the
// CS rising edge.
// ============================================================================
module spi_tw_slave (
    input  wire clk, input wire rst,
    input  wire sck, input wire mosi, input wire cs_n,
    output reg  [31:0] tuning_word,
    output reg         tw_valid
);
    reg [2:0] sck_s; reg [2:0] cs_s; reg [1:0] mosi_s;
    always @(posedge clk) begin
        sck_s  <= {sck_s[1:0],  sck};
        cs_s   <= {cs_s[1:0],   cs_n};
        mosi_s <= {mosi_s[0],   mosi};
    end
    wire sck_rise = (sck_s[2:1] == 2'b01);
    wire cs_low   = ~cs_s[1];
    wire cs_rise  = (cs_s[2:1] == 2'b01);

    reg [31:0] sr;
    always @(posedge clk) begin
        if (rst) begin sr <= 0; tuning_word <= 0; tw_valid <= 1'b0; end
        else begin
            tw_valid <= 1'b0;
            if (cs_low && sck_rise) sr <= {sr[30:0], mosi_s[1]};
            if (cs_rise) begin tuning_word <= sr; tw_valid <= 1'b1; end
        end
    end
endmodule


// ============================================================================
// spi_iq_slave -- bidirectional SPI slave (mode 0, MSB first, 32-bit frame =
// {I[15:0], Q[15:0]}). RX (mode_tx=0): on rx_valid latch the DDC sample; on CS
// falling load it into the shifter and stream it out on MISO. TX (mode_tx=1):
// shift MOSI in; on CS rising present the sample to the DUC with tx_valid.
// ============================================================================
module spi_iq_slave (
    input  wire clk, input wire rst,
    input  wire sck, input wire mosi, input wire cs_n,
    output reg  miso,
    input  wire mode_tx,
    input  wire signed [15:0] rx_i, input wire signed [15:0] rx_q,
    input  wire rx_valid,
    output reg  signed [15:0] tx_i, output reg signed [15:0] tx_q,
    output reg  tx_valid
);
    reg [2:0] sck_s; reg [2:0] cs_s; reg [1:0] mosi_s;
    always @(posedge clk) begin
        sck_s  <= {sck_s[1:0],  sck};
        cs_s   <= {cs_s[1:0],   cs_n};
        mosi_s <= {mosi_s[0],   mosi};
    end
    wire sck_rise = (sck_s[2:1] == 2'b01);
    wire sck_fall = (sck_s[2:1] == 2'b10);
    wire cs_low   = ~cs_s[1];
    wire cs_fall  = (cs_s[2:1] == 2'b10);   // frame start
    wire cs_rise  = (cs_s[2:1] == 2'b01);   // frame end

    // latest DDC sample, held for the next RX read
    reg [31:0] rx_hold;
    always @(posedge clk) begin
        if (rst) rx_hold <= 0;
        else if (rx_valid) rx_hold <= {rx_i, rx_q};
    end

    reg [31:0] sr;
    always @(posedge clk) begin
        if (rst) begin
            sr <= 0; miso <= 1'b0; tx_i <= 0; tx_q <= 0; tx_valid <= 1'b0;
        end else begin
            tx_valid <= 1'b0;
            if (cs_fall) begin
                sr   <= mode_tx ? 32'd0 : rx_hold;   // RX: load word to send
                miso <= mode_tx ? 1'b0  : rx_hold[31];
            end
            if (cs_low && sck_rise && mode_tx) begin
                sr <= {sr[30:0], mosi_s[1]};         // TX: shift MOSI in
            end
            if (cs_low && sck_fall && !mode_tx) begin
                sr   <= {sr[30:0], 1'b0};            // RX: advance, present next bit
                miso <= sr[30];
            end
            if (cs_rise && mode_tx) begin
                tx_i <= sr[31:16]; tx_q <= sr[15:0]; tx_valid <= 1'b1;
            end
        end
    end
endmodule
