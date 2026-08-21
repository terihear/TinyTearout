/*
 * Copyright (c) 2026 terihear
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_terihear_tinytearout (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // =========================================================
    // PARAMETERS
    // =========================================================
    localparam MCAND_W = 16;
    localparam COEFF_W = 8;
    localparam ACC_W   = 24; // 16 int + 7 frac + 1 sign guard
    localparam FRAC_B  = 7;  // Q0.7 format

    // =========================================================
    // INPUT SERDES: 3-cycle protocol
    //   Cycle 0 (LOAD_COEFF):  ui_in = coefficient (Q0.7 signed)
    //   Cycle 1 (LOAD_MCLO):   ui_in = multiplicand[7:0]
    //   Cycle 2 (LOAD_MCHI):   ui_in = multiplicand[15:8]
    // =========================================================
    localparam [1:0] S_IDLE     = 2'd0,
                     S_LOAD_COEFF = 2'd1,
                     S_LOAD_MCLO  = 2'd2,
                     S_LOAD_MCHI  = 2'd3;

    reg [1:0]             in_state;
    reg signed [COEFF_W-1:0] coeff_latch;
    reg [7:0]             mcand_lo;
    reg signed [MCAND_W-1:0] multiplicand;
    reg                   operands_valid; // Single-cycle pulse

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_state       <= S_IDLE;
            coeff_latch    <= 8'sd0;
            mcand_lo       <= 8'd0;
            multiplicand   <= 16'sd0;
            operands_valid <= 1'b0;
        end else if (ena) begin
            operands_valid <= 1'b0; // Default: single-cycle pulse
            case (in_state)
                S_IDLE: begin
                    // Wait for first valid input word (coefficient)
                    // External controller asserts ena and presents coeff
                    in_state <= S_LOAD_COEFF;
                end

                S_LOAD_COEFF: begin
                    coeff_latch <= $signed(ui_in);
                    in_state    <= S_LOAD_MCLO;
                end

                S_LOAD_MCLO: begin
                    mcand_lo <= ui_in;
                    in_state <= S_LOAD_MCHI;
                end

                S_LOAD_MCHI: begin
                    multiplicand   <= $signed({ui_in, mcand_lo});
                    operands_valid <= 1'b1; // Pulse: start multiply next cycle
                    in_state       <= S_IDLE;
                end

                default: in_state <= S_IDLE;
            end
        end else begin
            // Hold state when not enabled
            operands_valid <= 1'b0;
        end
    end

    // =========================================================
    // BIT-SERIAL MULTIPLIER
    // 8 cycles accumulation + 7 cycles alignment = 15 cycles
    // Rounding on cycle 16 → total 16-cycle latency from
    // operands_valid assertion to rounded result available
    // =========================================================
    reg signed [ACC_W-1:0] accumulator;
    reg [3:0]              bit_cnt;       // 0..14 during active multiply
    reg                    mult_active;
    reg                    mult_done;     // Single-cycle pulse

    wire coeff_bit   = coeff_latch[bit_cnt];
    wire is_last_bit = (bit_cnt == COEFF_W - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accumulator <= {ACC_W{1'sb0}};
            bit_cnt     <= 4'd0;
            mult_active <= 1'b0;
            mult_done   <= 1'b0;
        end else if (ena) begin
            mult_done <= 1'b0;

            if (operands_valid && !mult_active) begin
                // Launch new multiplication
                accumulator <= {ACC_W{1'sb0}};
                bit_cnt     <= 4'd0;
                mult_active <= 1'b1;
            end else if (mult_active) begin
                if (bit_cnt < COEFF_W) begin
                    // Accumulation phase: one coeff bit per cycle
                    if (is_last_bit) begin
                        // MSB of 2's complement has negative weight
                        if (coeff_bit)
                            accumulator <= accumulator -
                                ($signed(multiplicand) <<< FRAC_B);
                    end else begin
                        if (coeff_bit)
                            accumulator <= accumulator +
                                ($signed(multiplicand) <<< bit_cnt);
                    end
                    bit_cnt <= bit_cnt + 4'd1;
                end else if (bit_cnt == 4'd14) begin
                    // End of alignment padding (cycles 8..14)
                    mult_done   <= 1'b1;
                    mult_active <= 1'b0;
                    bit_cnt     <= 4'd0;
                end else begin
                    // Alignment/pipeline delay (no-op accumulation)
                    bit_cnt <= bit_cnt + 4'd1;
                end
            end
        end
    end

    // =========================================================
    // ROUNDING: Round-half-up on fractional residue
    // =========================================================
    reg signed [MCAND_W-1:0] rounded_result;
    reg                       result_valid; // Single-cycle pulse

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rounded_result <= 16'sd0;
            result_valid   <= 1'b0;
        end else if (ena) begin
            result_valid <= mult_done; // Propagate done pulse
            if (mult_done) begin
                // Round half-up: if frac[6]==1, add 1 to integer part
                if (accumulator[FRAC_B-1])
                    rounded_result <= accumulator[ACC_W-1:FRAC_B] + 16'sd1;
                else
                    rounded_result <= accumulator[ACC_W-1:FRAC_B];
            end
        end
    end

    // =========================================================
    // OUTPUT SERDES: 16-bit → two 8-bit words, LSB first
    // =========================================================
    reg        out_word_sel; // 0 = emit LO, 1 = emit HI
    reg [15:0] out_shift;
    reg        out_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uo_out     <= 8'd0;
            out_word_sel <= 1'b0;
            out_shift    <= 16'd0;
            out_active   <= 1'b0;
        end else if (ena) begin
            if (result_valid && !out_active) begin
                // Capture result, emit LO byte immediately
                out_shift    <= rounded_result;
                uo_out     <= rounded_result[7:0];
                out_word_sel <= 1'b1;
                out_active   <= 1'b1;
            end else if (out_active && out_word_sel) begin
                // Emit HI byte, then deactivate
                uo_out     <= out_shift[15:8];
                out_word_sel <= 1'b0;
                out_active   <= 1'b0;
            end else begin
                uo_out <= 8'd0;
            end
        end
    end

  // All output pins must be assigned. If not used, assign to 0.
  assign uio_out = 0;
  assign uio_oe  = 0;

  // List all unused inputs to prevent warnings
  wire _unused = &{uio_in, 1'b0};

endmodule
