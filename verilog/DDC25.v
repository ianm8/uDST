module nco_sincos_rx #(
    parameter LUT_BITS = 10,          // 1024 entries
    parameter PHASE_BITS = 32
)(
    input  wire                   clk,
    input  wire                   rst,
    input  wire [PHASE_BITS-1:0]  tuning_word,
    output reg  signed [15:0]     sin_out,
    output reg  signed [15:0]     cos_out
);

    localparam LUT_SIZE = 1 << LUT_BITS;
    localparam COS_OFFSET = LUT_SIZE >> 2;  // +π/2

    // ----------------------------
    // Phase accumulator
    // ----------------------------
    reg [PHASE_BITS-1:0] phase;

    always @(posedge clk) begin
        if (rst) begin
            phase <= 0;
        end else begin
            phase <= phase + tuning_word;
        end
    end

    // ----------------------------
    // LUT indexing
    // ----------------------------
    // :- is starting bits and width eg, 31 :- 10
    wire [LUT_BITS-1:0] sin_idx = phase[PHASE_BITS-1 -: LUT_BITS];
    wire [LUT_BITS-1:0] cos_idx = sin_idx + COS_OFFSET;   // wraps mod LUT_SIZE

    // ----------------------------
    // Sine LUT (Q15)
    // ----------------------------
    // Two copies of the table so sine and cosine can be read in the same cycle.
    // The reads are REGISTERED: on the iCE40 a block RAM read is synchronous,
    // so a registered read maps cleanly to BRAM. An asynchronous (combinational)
    // read of a 1024x16 ROM does NOT infer BRAM and is built from a large, slow
    // mux of logic cells instead.
    reg signed [15:0] sin_lut [0:LUT_SIZE-1];
    reg signed [15:0] cos_lut [0:LUT_SIZE-1];
	initial begin
		$readmemh("sin_lut.hex", sin_lut);
		$readmemh("sin_lut.hex", cos_lut);
	end

    always @(posedge clk) begin
        if (rst) begin
            sin_out <= 0;
            cos_out <= 0;
        end else begin
            sin_out <= sin_lut[sin_idx];
            cos_out <= cos_lut[cos_idx];
        end
    end

endmodule

// Registered 16x16 signed multiplier (32-bit product). Latency = 3 clocks.
//
// The INPUTS are registered (a_r, b_r) so the multiply runs register-to-
// register and packs into the iCE40 SB_MAC16 DSP (input regs + multiplier
// pipeline reg + output reg). That register placement is what lets it close
// timing at 80 MHz; registering only the output would leave the full
// combinational multiply in one clock. Synthesize with `synth_ice40 -dsp`,
// otherwise the multiply falls back to slow LUT logic.
//
// Latency is unchanged from the previous output-only version, so the FIR's
// MUL_LATENCY = 3 still holds.
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

// 255-tap, decimate-by-5, RX interpolation/anti-alias FIR.
//
// Unity passband gain (coefficients sum to 32768 in Q15 -> shift back by 15).
// One serial multiplier per channel; the sample histories and the coefficient
// table live in BLOCK RAM (synchronous read), because 255-deep x 16-bit x 2
// histories will not fit in flip-flops on a small device.
//
// Pipeline latency accounting (this is the part that must be exact):
//   - present a tap address on MAC clock k
//   - BRAM registered read  -> operands valid on clock k+READ_LAT
//   - mac16_reg (3 stages)  -> product valid on clock k+READ_LAT+MUL_LATENCY
//   We accumulate the product every clock and latch the result once the last
//   real tap (NTAPS-1) has come out, i.e. CNT_LAST = NTAPS+READ_LAT+MUL_LATENCY.
//
// The coefficient address is forced to the zero-padded ROM entry whenever we
// are not issuing a valid tap, so every spurious product in the pipeline is 0
// and accumulating on every clock is safe.
module fir255_dec5_rx (
    input  wire               clk,
    input  wire               rst,
    input  wire               din_valid,
    input  wire signed [15:0] xi,
    input  wire signed [15:0] xq,
    output reg                dout_valid,
    output reg signed [15:0]  yoI,
    output reg signed [15:0]  yoQ
);
    localparam integer NTAPS       = 255;
    localparam integer BUF_LEN     = 256;   // power of two; >= NTAPS
    localparam integer READ_LAT    = 1;     // BRAM synchronous-read latency
    localparam integer MUL_LATENCY = 3;     // mac16_reg latency
    localparam integer ROUND_LATENCY = 1;
    localparam [9:0]   CNT_LAST    = NTAPS + READ_LAT + MUL_LATENCY + ROUND_LATENCY;  // = 260
    localparam [7:0]   ZERO_TAP    = 8'd255; // ROM entry that holds 0

    // ---- coefficient ROM (256 entries; [255] = 0 pad), synchronous read ----
    reg signed [15:0] Hmem [0:255];
    initial $readmemh("fir_rx255_coeffs.hex", Hmem);

    // ---- sample histories in BRAM (synchronous read) ----
    reg signed [15:0] dI [0:BUF_LEN-1];
    reg signed [15:0] dQ [0:BUF_LEN-1];
    integer bi;
    initial for (bi = 0; bi < BUF_LEN; bi = bi + 1) begin
        dI[bi] = 0;
        dQ[bi] = 0;   // BRAM powers up to 0 on Gowin/iCE40; matched here for sim
    end

    reg [7:0] wr;        // next write slot
    reg [7:0] base;      // newest-sample index for the current MAC run
    reg [2:0] decim;     // 0..4 decimate-by-5 counter
    reg [9:0] cnt;       // MAC step counter

    localparam S_IDLE = 1'b0, S_MAC = 1'b1;
    reg state;

    // address generation
    wire issuing = (state == S_MAC) && (cnt < NTAPS);
    wire [7:0] coeff_addr = issuing ? cnt[7:0] : ZERO_TAP;  // 0 coeff when idle/draining
    wire [7:0] rd_addr    = base - cnt[7:0];                // don't-care unless issuing
    wire       wr_en      = (state == S_IDLE) && din_valid;

    // registered reads (infer BRAM). Reset the read-output regs so they
    // can't briefly go X right after reset and poison the (self-accumulating)
    // FIR accumulator for an entire decimation period -- same class of fix
    // applied to the TX FIR (DUC) earlier in this project.
    reg signed [15:0] sampI, sampQ, coeff;
    always @(posedge clk) begin
        if (wr_en) begin
            dI[wr] <= xi;
            dQ[wr] <= xq;
        end
        if (rst) begin
            sampI <= 0; sampQ <= 0; coeff <= 0;
        end else begin
            sampI <= dI[rd_addr];
            sampQ <= dQ[rd_addr];
            coeff <= Hmem[coeff_addr];
        end
    end

    // one multiplier per channel; same coefficient feeds both
    wire signed [31:0] prodI, prodQ;
    mac16_reg macI (.clk(clk), .rst(rst), .a(sampI), .b(coeff), .prod_out(prodI));
    mac16_reg macQ (.clk(clk), .rst(rst), .a(sampQ), .b(coeff), .prod_out(prodQ));

    // ---- accumulator: its own block, UNCONDITIONAL add so the FSM state
    //      never enters the carry chain; the clear is a post-add mux (1 LUT). ----
    localparam integer ACCW = 40;            // was 48; 33 bits is the worst-case need
    reg signed [ACCW-1:0] accI, accQ;
    wire acc_clr = (state == S_IDLE) && din_valid && (decim == 3'd4);  // same clear timing as before
    always @(posedge clk) begin
        if (rst || acc_clr) begin
            accI <= 0; accQ <= 0;
        end else begin
            accI <= accI + prodI;   // clean add; clr selects 0 vs sum AFTER the adder
            accQ <= accQ + prodQ;
        end
    end

    // registered saturate instead of wrap
    reg signed [ACCW-1:0] accI_rnd;
    reg signed [ACCW-1:0] accQ_rnd;
    always @(posedge clk) begin
        if (rst) begin
            accI_rnd <= 0;
            accQ_rnd <= 0;
        end else begin
            accI_rnd <= accI + (1 <<< 14);
            accQ_rnd <= accQ + (1 <<< 14);
        end
    end

    wire signed [ACCW-1:0] yI_sh = accI_rnd >>> 15;
    wire signed [ACCW-1:0] yQ_sh = accQ_rnd >>> 15;
    wire signed [15:0] yI_sat = (yI_sh >  40'sd32767) ?  16'sd32767 :
                                (yI_sh < -40'sd32768) ? -16'sd32768 : yI_sh[15:0];
    wire signed [15:0] yQ_sat = (yQ_sh >  40'sd32767) ?  16'sd32767 :
                                (yQ_sh < -40'sd32768) ? -16'sd32768 : yQ_sh[15:0];

    // ---- control FSM: no longer touches accI/accQ ----
    always @(posedge clk) begin
        if (rst) begin
            wr <= 0; base <= 0; decim <= 0; cnt <= 0;
            yoI <= 0; yoQ <= 0; dout_valid <= 0; state <= S_IDLE;
        end else begin
            dout_valid <= 1'b0;
            case (state)
            S_IDLE: begin
                if (din_valid) begin
                    if (decim == 3'd4) begin
                        decim <= 0;
                        base <= wr;
                        cnt <= 0;
                        state <= S_MAC;
                    end else decim <= decim + 1'b1;
                    wr <= wr + 1'b1;
                end
            end
            S_MAC: begin
                if (cnt == CNT_LAST) begin
                    yoI <= yI_sat;
					yoQ <= yQ_sat;
                    dout_valid <= 1'b1;
                    state <= S_IDLE;
                end else cnt <= cnt + 1'b1;
            end
            endcase
        end
    end
endmodule

module ddc_top (
    input  wire        clk80,
    input  wire        rst,
    input  wire signed [11:0] adc_in,
    // tuning word interface
    input  wire        tw_valid,
    input  wire [31:0] tuning_word,
    output wire        iq_valid,
    output wire signed [15:0] iq_i,
    output wire signed [15:0] iq_q
);

    // ============================================================
    // TUNING WORD SHADOW
    // ============================================================
    reg [31:0] tuning_active;
    always @(posedge clk80) begin
        if (rst)
            tuning_active <= 32'd0;
        else if (tw_valid)
            tuning_active <= tuning_word;
    end

    // ============================================================
    // NCO
    // ============================================================
    wire signed [15:0] nco_sin;
    wire signed [15:0] nco_cos;

    nco_sincos_rx NCO (
        .clk        (clk80),
        .rst        (rst),
        .tuning_word(tuning_active),
        .sin_out    (nco_sin),
        .cos_out    (nco_cos)
    );

    // ============================================================
    // INLINE REGISTERED MULTIPLY (replacing SB_MAC16)
    // ============================================================
    
    // Stage 1: Register inputs (optional - like INPUT_REG=1)
    reg signed [15:0] adc_r, sin_r, cos_r;
    always @(posedge clk80) begin
        if (rst) begin
            adc_r <= 0;
            sin_r <= 0;
            cos_r <= 0;
        end else begin
            adc_r <= {{4{adc_in[11]}}, adc_in};  // Sign-extend to 16 bits
            sin_r <= nco_sin;
            cos_r <= nco_cos;
        end
    end
    
    // Stage 2: Multiplication (combinatorial)
    wire signed [31:0] mult_i_raw = $signed(adc_r) * $signed(cos_r);
    wire signed [31:0] mult_q_raw = $signed(adc_r) * $signed(sin_r);
    
    // Stage 3: Register outputs (like OUTPUT_REG=1)
    reg signed [31:0] mult_i_reg, mult_q_reg;
    always @(posedge clk80) begin
        if (rst) begin
            mult_i_reg <= 0;
            mult_q_reg <= 0;
        end else begin
            mult_i_reg <= mult_i_raw;
            mult_q_reg <= mult_q_raw;
        end
    end
    
    // ============================================================
    // SCALE MIXER OUTPUTS (Q15 → integer)
    // ============================================================
    reg signed [31:0] mixI, mixQ;
    always @(posedge clk80) begin
        if (rst) begin
            mixI <= 0;
            mixQ <= 0;
        end else begin
            mixI <= (mult_i_reg + 32'sd16384) >>> 15; // round and scale from Q15
            mixQ <= (mult_q_reg + 32'sd16384) >>> 15; // round and scale from Q15
        end
    end

    // ============================================================
    // REST OF THE PIPELINE (unchanged)
    // ============================================================
    // CIC Integrators
    reg signed [47:0] i1I, i2I, i3I;
    reg signed [47:0] i1Q, i2Q, i3Q;
    always @(posedge clk80) begin
        if (rst) begin
            i1I <= 0; i2I <= 0; i3I <= 0;
            i1Q <= 0; i2Q <= 0; i3Q <= 0;
        end else begin
            i1I <= i1I + mixI;
            i2I <= i2I + i1I;
            i3I <= i3I + i2I;
            i1Q <= i1Q + mixQ;
            i2Q <= i2Q + i1Q;
            i3Q <= i3Q + i2Q;
        end
    end

    // ============================================================
    // Decimation control (unchanged)
    // ============================================================
    reg [8:0] dec_cnt;
    always @(posedge clk80)
        if (rst) dec_cnt <= 0;
        else     dec_cnt <= dec_cnt + 1;

    // --- new: registered cic_valid and registered outputs
    reg        cic_valid_reg;
    reg signed [31:0] cicI_reg, cicQ_reg;

    // ============================================================
    // CIC Combs (pipelined to make comb result stable before valid)
    // ============================================================
    // Sampled integrator outputs at decimation instant
    reg signed [47:0] i3I_s, i3Q_s;

    // Delay registers for comb stages and intermediate results
    reg signed [47:0] comb_d1I, comb_d2I, comb_d3I;
    reg signed [47:0] comb_d1Q, comb_d2Q, comb_d3Q;

    reg signed [47:0] y0I, y1I, y2I;
    reg signed [47:0] y0Q, y1Q, y2Q;

    always @(posedge clk80) begin
        if (rst) begin
            // clear sampled integrator and comb state
            i3I_s <= 0; i3Q_s <= 0;
            comb_d1I <= 0; comb_d2I <= 0; comb_d3I <= 0;
            comb_d1Q <= 0; comb_d2Q <= 0; comb_d3Q <= 0;
            y0I <= 0; y1I <= 0; y2I <= 0;
            y0Q <= 0; y1Q <= 0; y2Q <= 0;
            cicI_reg <= 0; cicQ_reg <= 0;
            cic_valid_reg <= 1'b0;
        end else begin
            // default: deassert valid
            cic_valid_reg <= 1'b0;

            if (dec_cnt == 9'd511) begin
                // Stage A: sample integrator outputs at decimation instant
                // (this captures i3I/i3Q for comb processing)
                i3I_s <= i3I;
                i3Q_s <= i3Q;

                // Stage B: compute first comb difference using previous delay
                // y0 = x[n] - x[n-1]
                y0I <= i3I_s - comb_d1I;
                y0Q <= i3Q_s - comb_d1Q;

                // update first delay with sampled integrator
                comb_d1I <= i3I_s;
                comb_d1Q <= i3Q_s;

                // Stage C: compute second comb difference using previous y0 delay
                y1I <= y0I - comb_d2I;
                y1Q <= y0Q - comb_d2Q;

                // update second delay with current y0
                comb_d2I <= y0I;
                comb_d2Q <= y0Q;

                // Stage D: compute third comb difference using previous y1 delay
                y2I <= y1I - comb_d3I;
                y2Q <= y1Q - comb_d3Q;

                // update third delay with current y1
                comb_d3I <= y1I;
                comb_d3Q <= y1Q;

                // Stage E: scale and register final output, then assert valid
                //cicI_reg <= (y2I + 48'sd4194304) >>> 23; // ADC clips first
                //cicQ_reg <= (y2Q + 48'sd4194304) >>> 23; // ADC clips first
                cicI_reg <= (y2I + 48'sd2097152) >>> 22; // S9 + 55
                cicQ_reg <= (y2Q + 48'sd2097152) >>> 22; // S9 + 55
                //cicI_reg <= (y2I + 48'sd1048576) >>> 21; // S9 + 49
                //cicQ_reg <= (y2Q + 48'sd1048576) >>> 21; // S9 + 49
                //cicI_reg <= (y2I + 48'sd524288) >>> 20; // S9 + 43
                //cicQ_reg <= (y2Q + 48'sd524288) >>> 20; // S9 + 43

                // Indicate the registered outputs are ready for downstream FIR
                cic_valid_reg <= 1'b1;
            end
        end
    end

    // Expose the registered valid and outputs to the FIR
    wire cic_valid = cic_valid_reg;
    // keep cicI/cicQ names used by FIR (drive from registered values)
    wire signed [15:0] cicI_sat = (cicI_reg >  32'sd32767) ?  16'sd32767 :
                                  (cicI_reg < -32'sd32768) ? -16'sd32768 : $signed(cicI_reg[15:0]);
    wire signed [15:0] cicQ_sat = (cicQ_reg >  32'sd32767) ?  16'sd32767 :
                                  (cicQ_reg < -32'sd32768) ? -16'sd32768 : $signed(cicQ_reg[15:0]);

    reg signed [15:0] cicI;
    reg signed [15:0] cicQ;
    always @(posedge clk80) begin
        if (rst) begin
            cicI <= 0;
            cicQ <= 0;
        end else begin
            cicI <= cicI_sat;
            cicQ <= cicQ_sat;
        end
    end

    // FIR Decimator
    wire fir_valid;
    wire signed [15:0] firI, firQ;
    fir255_dec5_rx FIR (
        .clk        (clk80),
        .rst        (rst),
        .din_valid  (cic_valid),
        .xi         (cicI),
        .xq         (cicQ),
        .yoI        (firI),
        .yoQ        (firQ),
        .dout_valid (fir_valid)
    );

    assign iq_valid = fir_valid;
    assign iq_i     = firI;
    assign iq_q     = firQ;

endmodule