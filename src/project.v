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
    // INPUT: 2-cycles
    //   Cycle 0 (LOAD_COEFF): uio_in = coefficient (Q0.7 signed)
    //   Cycle 0 (LOAD_MCLO):   ui_in = multiplicand[7:0]
    //   Cycle 1 (LOAD_MCHI):   ui_in = multiplicand[15:8]
    // =========================================================
    localparam [1:0] S_IDLE     = 2'd0,
                     S_LOAD_COEFF = 2'd1,
                     S_LOAD_MCHI  = 2'd3;

    reg [1:0]             in_state;
    reg signed [COEFF_W-1:0] coeff_latch;
    reg [7:0]             mcand_lo;
    reg signed [ACC_W-1:0] multiplicand;
    reg                   operands_valid; // Single-cycle pulse

    assign uio_oe = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_state       <= S_IDLE;
            coeff_latch    <= 8'sd0;
            mcand_lo       <= 8'd0;
            multiplicand   <= {ACC_W{1'sb0}};
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
                    coeff_latch <= $signed(uio_in);
                    mcand_lo <= ui_in;
                    in_state <= S_LOAD_MCHI;
                end

                S_LOAD_MCHI: begin
                    multiplicand   <= $signed({ui_in, mcand_lo});
                    operands_valid <= 1'b1; // Pulse: start multiply next cycle
                    in_state       <= S_IDLE;
                end

                default: in_state <= S_IDLE;
	      endcase
        end else begin
            // Hold state when not enabled
            operands_valid <= 1'b0;
        end
    end

    assign uio_oe = 8'hFF;

    // =========================================================
    // BIT-SERIAL MULTIPLIER
    // 8 cycles accumulation
    // Rounding on cycle 6
    // operands_valid assertion to rounded result available
    // =========================================================
    reg signed [ACC_W-1:0] accumulator;
    reg [2:0]              bit_cnt;       // 0..7 during active multiply
    reg                    mult_active;
    reg signed [MCAND_W-1:0] rounded_result;
    reg                      result_valid;

    wire coeff_bit   = coeff_latch[bit_cnt];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accumulator <= {ACC_W{1'sb0}};
            bit_cnt     <= 3'd0;
            mult_active <= 1'b0;
            rounded_result <= 16'sd0;
            result_valid   <= 1'b0;
        end else if (ena) begin
            if (operands_valid && !mult_active) begin
                // Launch new multiplication
                accumulator <= {ACC_W{1'sb0}};
                bit_cnt     <= 3'd0;
                mult_active <= 1'b1;
            end else if (mult_active) begin
               // Accumulation: one coeff bit per cycle
	       if (bit_cnt == FRAC_B) begin
		  // MSB of coefficient
                  if (coeff_bit) begin
                     // MSB of 2's complement has negative weight
                     accumulator <= accumulator -
				    ($signed(multiplicand) <<< bit_cnt);
		  end
		  rounded_result <= accumulator[ACC_W-1:FRAC_B];
                  mult_active <= 1'b0;
		  result_valid <= 1'b1;
                  bit_cnt     <= 3'd0;
	       end else begin
                  if (coeff_bit) begin
                     accumulator <= accumulator +
                                    ($signed(multiplicand) <<< bit_cnt);
		  end
		  if (bit_cnt == (FRAC_B-1)) begin
		     // round half-up: if frac[6]==1
		     if (accumulator[FRAC_B-1])
		       accumulator <= accumulator +
				      ((16'sd1) <<< bit_cnt);
		  end
		  bit_cnt <= bit_cnt + 3'd1;
	       end // else: !if(bit_cnt == FRAC_B)
            end // if (mult_active)
	end // if (ena)
    end

    // =========================================================
    // OUTPUT: 16-bit → two 8-bit words, LSB first
    // =========================================================
    reg        out_word_sel; // 0 = emit LO, 1 = emit HI
    reg [15:0] out_shift;
    reg        out_active;
    reg [7:0]  out_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_data     <= 8'd0;
            out_word_sel <= 1'b0;
            out_shift    <= 16'd0;
            out_active   <= 1'b0;
      end else if (ena) begin
         if (result_valid && !out_active) begin
            // Capture result, emit LO byte immediately
            out_shift    <= rounded_result;
            out_data     <= rounded_result[7:0];
            out_word_sel <= 1'b0;
            out_active   <= 1'b1;
         end else if (out_active && !out_word_sel) begin
            // Emit HI byte
            out_data     <= out_shift[15:8];
            out_word_sel <= 1'b1;
            out_active   <= 1'b1;
         end else begin
	    out_word_sel <= 1'b0;
	    out_active <= 1'b0;
            out_data <= 8'd0;
         end
      end // if (ena)
    end // always @ (posedge clk or negedge rst_n)

   // uio_out[0] as indicating output LO available
   // uio_out[1] as indicating LO or HI
   assign uio_out = {6'b0, out_word_sel, out_active};
   assign uo_out = out_data;
   
   // All output pins must be assigned. If not used, assign to 0.
 
  // List all unused inputs to prevent warnings

endmodule
