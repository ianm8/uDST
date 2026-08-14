`timescale 1ns/1ps

// Registered 16x16 signed multiplier (32-bit product). Latency = 3 clocks.
//
// The INPUTS are registered (a_r, b_r) so the multiply runs register-to-
// register and packs into the iCE40 SB_MAC16 DSP (input regs + multiplier
// pipeline reg + output reg). That register placement is what lets it close
// timing at ~81 MHz; registering only the output would leave the full
// combinational multiply in one clock. Synthesize with `synth_ice40 -dsp`,
// otherwise the multiply falls back to slow LUT logic.
//
// Latency is unchanged from the previous output-only version, so the FIR's
// MUL_LATENCY = 3 still holds.
/*
module mac16_reg (
    input  wire               clk,
    input  wire               rst,
    input  wire signed [15:0] a,        // multiplicand
    input  wire signed [15:0] b,        // multiplier (coefficient)
    output reg  signed [31:0] prod_out
);
    reg signed [15:0] a_r, b_r;         // input registers   -> SB_MAC16 A/B regs
    reg signed [31:0] mult_r;           // multiplier output -> SB_MAC16 pipeline reg

    always @(posedge clk) begin
        if (rst) begin
            a_r      <= 16'sd0;
            b_r      <= 16'sd0;
            mult_r   <= 32'sd0;
            prod_out <= 32'sd0;
        end else begin
            // stage 0 -> 1: register inputs
            a_r <= a;
            b_r <= b;

            // stage 1 -> 2: multiply (register-to-register)
            mult_r <= $signed(a_r) * $signed(b_r);

            // stage 2 -> output
            prod_out <= mult_r;
        end
    end
endmodule
*/

module upsampler_zero_stuff6_tx (
    input  wire clk,
    input  wire rst,

    input  wire in_valid,
    input  wire signed [9:0] inI,
    input  wire signed [9:0] inQ,

    output reg  out_valid,
    output reg  signed [9:0] outI,
    output reg  signed [9:0] outQ
);

    // constants
    localparam integer OUT_DIV = 432;   // 81 MHz / 187.5 kHz
    localparam integer R       = 6;

    reg signed [9:0] latI, latQ;
    reg [2:0] phase;                   // 0..5
    integer out_cnt;

    always @(posedge clk) begin
        if (rst) begin
            latI      <= 0;
            latQ      <= 0;
            phase     <= 0;
            out_cnt   <= 0;
            out_valid <= 0;
            outI      <= 0;
            outQ      <= 0;
        end else begin
            // latch incoming sample
            if (in_valid) begin
                latI <= inI;
                latQ <= inQ;
            end

            out_valid <= 0;

            if (out_cnt == OUT_DIV-1) begin
                out_cnt   <= 0;
                out_valid <= 1;

                if (phase == 0) begin
                    outI <= latI;
                    outQ <= latQ;
                end else begin
                    outI <= 0;
                    outQ <= 0;
                end

                if (phase == R-1)
                    phase <= 0;
                else
                    phase <= phase + 1;
            end else begin
                out_cnt <= out_cnt + 1;
            end
        end
    end

endmodule


module fir_serial_tx #(
    parameter integer NTAPS=63, parameter integer OUT_SHIFT=15,
    parameter integer MUL_LATENCY=3, parameter integer READ_LAT=1
)(
    input wire clk, input wire rst, input wire in_valid,
    input wire signed [9:0] inI, input wire signed [9:0] inQ,
    output reg out_valid, output reg signed [31:0] outI, output reg signed [31:0] outQ);

    localparam integer BUF_LEN=64;
    localparam integer ACCW=40;
    localparam [6:0] CNT_LAST = NTAPS + READ_LAT + MUL_LATENCY;   // 63+1+3 = 67
    localparam [5:0] ZERO_TAP = 6'd63;                            // Hmem[63] = 0 pad

    // coefficient ROM (64 entries; [59..63]=0 pad), Q15, ~3 kHz LP @187.5 kHz.
    // DC gain x4.214; combined with OUT_SHIFT=14 the effective FIR gain is x8.43,
    // which cancels the /6 zero-stuff and the CIC interpolator's x0.712 to net
    // ~x1.0 TX interpolation gain (same drive level as the 80 MHz prototype).
    // Stopband (>=28 kHz) ~ -72 dB. Synchronous read.
    reg signed [15:0] Hmem [0:BUF_LEN-1];
    initial begin
        Hmem[0]=0;Hmem[1]=0;Hmem[2]=0;Hmem[3]=0;Hmem[4]=-2;Hmem[5]=-6;Hmem[6]=-15;Hmem[7]=-29;Hmem[8]=-46;Hmem[9]=-60;
        Hmem[10]=-56;Hmem[11]=-19;Hmem[12]=67;Hmem[13]=206;Hmem[14]=376;Hmem[15]=525;Hmem[16]=572;Hmem[17]=425;Hmem[18]=12;Hmem[19]=-672;
        Hmem[20]=-1531;Hmem[21]=-2348;Hmem[22]=-2801;Hmem[23]=-2528;Hmem[24]=-1216;Hmem[25]=1290;Hmem[26]=4906;Hmem[27]=9272;Hmem[28]=13794;Hmem[29]=17753;
        Hmem[30]=20460;Hmem[31]=21426;Hmem[32]=20460;Hmem[33]=17753;Hmem[34]=13794;Hmem[35]=9272;Hmem[36]=4906;Hmem[37]=1290;Hmem[38]=-1216;Hmem[39]=-2528;
        Hmem[40]=-2801;Hmem[41]=-2348;Hmem[42]=-1531;Hmem[43]=-672;Hmem[44]=12;Hmem[45]=425;Hmem[46]=572;Hmem[47]=525;Hmem[48]=376;Hmem[49]=206;
        Hmem[50]=67;Hmem[51]=-19;Hmem[52]=-56;Hmem[53]=-60;Hmem[54]=-46;Hmem[55]=-29;Hmem[56]=-15;Hmem[57]=-6;Hmem[58]=-2;Hmem[59]=0;
        Hmem[60]=0;Hmem[61]=0;Hmem[62]=0;Hmem[63]=0;
    end

    // sample buffer in BRAM (synchronous read). Forced to BSRAM so the write
    // uses a dedicated write-enable (no 64-way register CE fanout) and the read
    // is a RAM port rather than a LUT mux -- fixes both FIR timing path families.
    (* syn_ramstyle = "block_ram" *) reg signed [9:0] bufI [0:BUF_LEN-1];
    (* syn_ramstyle = "block_ram" *) reg signed [9:0] bufQ [0:BUF_LEN-1];
    integer bi;
    initial for (bi=0;bi<BUF_LEN;bi=bi+1) begin bufI[bi]=0; bufQ[bi]=0; end

    reg [5:0] wr, base; reg [6:0] cnt;
    localparam S_IDLE=1'b0,S_MAC=1'b1; reg state;

    wire issuing = (state==S_MAC) && (cnt<NTAPS);
    wire [5:0] coeff_addr = issuing ? cnt[5:0] : ZERO_TAP;   // zero coeff when idle/draining
    wire [5:0] rd_addr    = base - cnt[5:0];                 // don't-care unless issuing
    wire       wr_en      = (state==S_IDLE) && in_valid;

    // BRAM write port (separate process; no reset, BRAM-friendly)
    always @(posedge clk) if (wr_en) begin bufI[wr] <= inI; bufQ[wr] <= inQ; end

    // registered reads (isolate the read mux/address from the multiplier).
    // Reset the read-output regs so the first MAC after reset is defined.
    reg signed [9:0]  sampI, sampQ;
    reg signed [15:0] coeff;
    always @(posedge clk) begin
        if (rst) begin
            sampI <= 0; sampQ <= 0; coeff <= 0;
        end else begin
            sampI <= bufI[rd_addr];
            sampQ <= bufQ[rd_addr];
            coeff <= Hmem[coeff_addr];
        end
    end

    wire signed [15:0] mac_aI = {{6{sampI[9]}}, sampI};
    wire signed [15:0] mac_aQ = {{6{sampQ[9]}}, sampQ};
    wire signed [31:0] macI_out, macQ_out;
    mac16_reg macI (.clk(clk),.rst(rst),.a(mac_aI),.b(coeff),.prod_out(macI_out));
    mac16_reg macQ (.clk(clk),.rst(rst),.a(mac_aQ),.b(coeff),.prod_out(macQ_out));

    // accumulator: own block, unconditional add; clear after latch
    reg signed [ACCW-1:0] accI, accQ;
    wire acc_clr = (state==S_IDLE) && in_valid;
    always @(posedge clk) begin
        if (rst || acc_clr) begin accI<=0; accQ<=0; end
        else begin accI <= accI + macI_out; accQ <= accQ + macQ_out; end
    end

    // control FSM
    always @(posedge clk) begin
        if (rst) begin
            wr<=0; base<=0; cnt<=0; outI<=0; outQ<=0; out_valid<=0; state<=S_IDLE;
        end else begin
            out_valid <= 1'b0;
            case (state)
            S_IDLE: if (in_valid) begin base<=wr; cnt<=0; wr<=wr+1'b1; state<=S_MAC; end
            S_MAC:  if (cnt==CNT_LAST) begin
                        outI<=accI>>>OUT_SHIFT; outQ<=accQ>>>OUT_SHIFT; out_valid<=1'b1; state<=S_IDLE;
                    end else cnt<=cnt+1'b1;
            endcase
        end
    end
endmodule


// ============================================================
// 3rd-order CIC interpolator
// - Input rate  : 187.5 kHz
// - Output rate : 81 MHz
// - Interp      : R = 432
// - Order       : N = 3
// - Integer I/Q, 10-bit in / 10-bit out
module cic_interp_3rd_tx (
    input  wire clk, input wire rst, input wire in_valid,
    input  wire signed [9:0] inI, input wire signed [9:0] inQ,
    output reg  signed [9:0] outI, output reg signed [9:0] outQ
);
    // R is documentation here (rate change is set by the in_valid cadence from the
    // upsampler); only SCALE_SHIFT feeds the logic. 432^2/2^18 = x0.712 CIC gain.
    localparam integer N=3, R=432, SCALE_SHIFT=18, AW=40;
    localparam integer CW=16;                 // comb width (>=13 needed; 16 = 3 bits margin)
    wire signed [CW-1:0] xI = {{(CW-10){inI[9]}}, inI};
    wire signed [CW-1:0] xQ = {{(CW-10){inQ[9]}}, inQ};

    reg signed [CW-1:0] c0d_I,c1d_I,c2d_I, c0y_I,c1y_I,c2y_I;
    reg signed [CW-1:0] c0d_Q,c1d_Q,c2d_Q, c0y_Q,c1y_Q,c2y_Q;
    reg signed [AW-1:0] inj_I, inj_Q;
    reg signed [AW-1:0] i0_I,i1_I,i2_I, i0_Q,i1_Q,i2_Q;

    always @(posedge clk) begin
        if (rst) begin
            c0d_I<=0;c1d_I<=0;c2d_I<=0; c0y_I<=0;c1y_I<=0;c2y_I<=0;
            c0d_Q<=0;c1d_Q<=0;c2d_Q<=0; c0y_Q<=0;c1y_Q<=0;c2y_Q<=0;
            inj_I<=0; inj_Q<=0;
            i0_I<=0;i1_I<=0;i2_I<=0; i0_Q<=0;i1_Q<=0;i2_Q<=0;
            outI<=0; outQ<=0;
        end else begin
            if (in_valid) begin
                c0y_I <= xI    - c0d_I;  c0d_I <= xI;
                c1y_I <= c0y_I - c1d_I;  c1d_I <= c0y_I;
                c2y_I <= c1y_I - c2d_I;  c2d_I <= c1y_I;
                c0y_Q <= xQ    - c0d_Q;  c0d_Q <= xQ;
                c1y_Q <= c0y_Q - c1d_Q;  c1d_Q <= c0y_Q;
                c2y_Q <= c1y_Q - c2d_Q;  c2d_Q <= c1y_Q;
                inj_I <= {{(AW-CW){c2y_I[CW-1]}}, c2y_I};   // widen CW -> AW
                inj_Q <= {{(AW-CW){c2y_Q[CW-1]}}, c2y_Q};
            end else begin
                inj_I <= {AW{1'b0}};  inj_Q <= {AW{1'b0}};
            end
            i0_I <= i0_I + inj_I;  i1_I <= i1_I + i0_I;  i2_I <= i2_I + i1_I;
            i0_Q <= i0_Q + inj_Q;  i1_Q <= i1_Q + i0_Q;  i2_Q <= i2_Q + i1_Q;
            outI <= i2_I >>> SCALE_SHIFT;
            outQ <= i2_Q >>> SCALE_SHIFT;
        end
    end
endmodule


module nco_iq_mix_lut_tx (
    input  wire        clk,
    input  wire        rst,

    // Phase control
    input  wire [31:0] phase_inc,

    // IQ input from CIC (signed 10-bit)
    input  wire signed [9:0]  inI,
    input  wire signed [9:0]  inQ,

    // 10-bit unsigned DAC output
    output reg  [9:0]  dac_out
);

    // -----------------------------
    // Phase accumulator
    // -----------------------------
    reg [31:0] phase_acc;
    always @(posedge clk) begin
        if (rst) phase_acc <= 32'd0;
        else     phase_acc <= phase_acc + phase_inc;
    end

    // -----------------------------
    // LUT addressing (1024 entries)
    // -----------------------------
    wire [9:0] sin_idx = phase_acc[31:22];
    wire [9:0] cos_idx = sin_idx + 10'd256;          // +90 deg, wraps mod 1024

    // -----------------------------
    // Sine LUT (Q15) — two copies, REGISTERED reads so the synthesiser maps
    // them to iCE40 block RAM (an async ROM read is built from logic). Two
    // copies give the two independent read ports we need (sin and cos).
    // -----------------------------
    reg signed [15:0] sin_lut [0:1023];
    reg signed [15:0] cos_lut [0:1023];
    initial begin
        $readmemh("sin_lut.hex", sin_lut);
        $readmemh("sin_lut.hex", cos_lut);
    end

    reg signed [15:0] sin_val, cos_val;
    always @(posedge clk) begin
        sin_val <= sin_lut[sin_idx];
        cos_val <= cos_lut[cos_idx];
    end

    // Delay the IQ inputs by one clock so they line up with the now-registered
    // LUT outputs (same phase instant feeds both the multiplier operands).
    reg signed [9:0] inI_d, inQ_d;
    always @(posedge clk) begin
        if (rst) begin
            inI_d <= 10'sd0;
            inQ_d <= 10'sd0;
        end else begin
            inI_d <= inI;
            inQ_d <= inQ;
        end
    end

    // -----------------------------
    // Registered multipliers (SB_MAC16 equivalent)
    // -----------------------------
    wire signed [31:0] mul_i;
    wire signed [31:0] mul_q;

    mac16_reg u_mac_i (
        .clk      (clk),
        .rst      (rst),
        .a        ({{6{inI_d[9]}}, inI_d}),   // sign-extend 10 -> 16
        .b        (cos_val),
        .prod_out (mul_i)
    );

    mac16_reg u_mac_q (
        .clk      (clk),
        .rst      (rst),
        .a        ({{6{inQ_d[9]}}, inQ_d}),   // sign-extend 10 -> 16
        .b        (sin_val),
        .prod_out (mul_q)
    );

    // -----------------------------
    // Subtract and scale (Q15 -> integer)
    // -----------------------------
    reg signed [31:0] mix_q15;
    reg signed [15:0] mix_int;
    always @(posedge clk) begin
        if (rst) begin
            mix_q15 <= 32'sd0;
            mix_int <= 16'sd0;
        end else begin
            mix_q15 <= mul_i - mul_q;      // still Q15 (upper sideband)
            mix_int <= mix_q15 >>> 15;     // Q15 -> integer
        end
    end

    // -----------------------------
    // Convert to unsigned DAC range (with saturation)
    // -----------------------------
    always @(posedge clk) begin
        if (rst) begin
            dac_out <= 10'd512;
        end else begin
            if (mix_int >  16'sd511)
                dac_out <= 10'd1023;
            else if (mix_int < -16'sd512)
                dac_out <= 10'd0;
            else
                dac_out <= mix_int[9:0] + 10'd512;
        end
    end

endmodule


module duc_top (
    input  wire        clk,
    input  wire        rst,
    input  wire        in_valid,
    input  wire signed [15:0] inI,
    input  wire signed [15:0] inQ,
    input  wire [31:0] tuning_word,
    output wire [9:0]  dac_out
);

    // -----------------------------
    // Internal signals
    // -----------------------------
    wire              up_valid;
    wire signed [9:0] upI, upQ;

    wire              fir_valid;
    wire signed [31:0] firI, firQ;

    wire signed [9:0] cicI, cicQ;

    // FIR output saturated to the 10-bit datapath. The FIR has ~x8.4 effective
    // gain (compensating the /6 zero-stuffing and the CIC's x0.712), so firI/firQ
    // can exceed the 10-bit range; a bare [9:0] slice would wrap. Saturate instead.
    wire signed [9:0] firI_sat = (firI >  32'sd511) ?  10'sd511 :
                                 (firI < -32'sd512) ? -10'sd512 :
                                 firI[9:0];
    wire signed [9:0] firQ_sat = (firQ >  32'sd511) ?  10'sd511 :
                                 (firQ < -32'sd512) ? -10'sd512 :
                                 firQ[9:0];

    // Register the saturated CIC input: the 32-bit saturation compares above are
    // the DUC critical path, so put them in their own clock instead of letting
    // them fall through into the CIC comb subtract. +1 cycle latency (TX: don't care).
    reg signed [9:0] cic_inI, cic_inQ;
    reg              cic_inv;
    always @(posedge clk) begin
        if (rst) begin cic_inI <= 10'sd0; cic_inQ <= 10'sd0; cic_inv <= 1'b0; end
        else begin     cic_inI <= firI_sat; cic_inQ <= firQ_sat; cic_inv <= fir_valid; end
    end

    // -----------------------------
    // Upsampler (31.25 kHz → 187.5 kHz)
    // -----------------------------
    upsampler_zero_stuff6_tx u_upsampler (
        .clk       (clk),
        .rst       (rst),
        .in_valid  (in_valid),
        .inI       (inI[9:0]),   // explicit truncate
        .inQ       (inQ[9:0]),
        .out_valid (up_valid),
        .outI      (upI),
        .outQ      (upQ)
    );

    // -----------------------------
    // FIR (187.5 kHz)
    // -----------------------------
    fir_serial_tx #(
        .NTAPS(63),
        .OUT_SHIFT(14)
    ) u_fir (
        .clk       (clk),
        .rst       (rst),
        .in_valid  (up_valid),
        .inI       (upI),
        .inQ       (upQ),
        .out_valid (fir_valid),
        .outI      (firI),
        .outQ      (firQ)
    );

    // -----------------------------
    // CIC interpolator (→ 81 MHz)
    // -----------------------------
    cic_interp_3rd_tx u_cic (
        .clk      (clk),
        .rst      (rst),
        .in_valid (cic_inv),
        .inI      (cic_inI),
        .inQ      (cic_inQ),
        .outI     (cicI),
        .outQ     (cicQ)
    );

    // -----------------------------
    // NCO + IQ Mixer + DAC
    // -----------------------------
    nco_iq_mix_lut_tx u_nco (
        .clk       (clk),
        .rst       (rst),
        .phase_inc (tuning_word),
        .inI       (cicI),
        .inQ       (cicQ),
        .dac_out   (dac_out)
    );

endmodule
