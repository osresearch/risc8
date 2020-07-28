/*
 * Combinatorial 8-bit ALU
 *
 * This implements a simple 8-bit ALU with status register flags,
 * as definied in the AVR instruction datasheet.
 *
 * Ra can be 8 or 16-bits, Rb is only 8 bits.
 */
`ifndef _risc8_alu_v_
`define _risc8_alu_v_

`define OP_MOVE	4'h0 // Copy the input Ra word to the output
`define OP_MOVR	4'h1 // Copy the input Rb byte to the output
`define OP_ADD	4'h2
`define OP_SUB	4'h3
`define OP_ADW	4'h4
`define OP_SBW	4'h5 // must be OP_ADW | 1
`define OP_NEG	4'h6
`define OP_SWAP	4'h7
`define OP_ASR	4'h8
`define OP_ROR	4'h9 // must be OP_ASR | 1
`define OP_LSR	4'hA
`define OP_AND	4'hB
`define OP_EOR	4'hC
`define OP_OR	4'hD
`define OP_SREG	4'hE // Update the SREG flags, uses carry input for set/clear
`define OP_MUL  4'hF // optional

// If you don't use SREG half-carry flag, turn it off to save a few LC
`define SREG_H

module risc8_alu(
	input clk,
	input reset,

	input [15:0] Rd_in, // Might be a word register
	input [7:0] Rr_in,
	input [7:0] sreg_in, // carry in
	input [3:0] op,
	input use_carry,

	output [15:0] R_out, // full word output register
	output [7:0] sreg_out
);
	reg [7:0] R;
	reg [7:0] Rh;
	assign R_out = { Rh, R };

	wire [7:0] Rr = Rr_in;
	wire [7:0] Rd = Rd_in[7:0]; // default is operate on only the bottom byte
	reg SI, ST, SH, SS, SV, SN, SZ, SC;
	assign sreg_out = { SI, ST, SH, SS, SV, SN, SZ, SC };

	// helpers for computing sreg updates
	wire Rd3 = Rd[3];
	wire Rd7 = Rd[7];
	wire Rdh7 = Rd_in[15];
	wire Rr3 = Rr[3];
	wire Rr7 = Rr[7];
	wire R3 = R_out[3];
	wire R7 = R_out[7];
	wire R15 = R_out[15];
	wire C = sreg_in[0];
	wire opt_C = use_carry ? C : 0; // Optional Carry
	wire R_zero = R == 0;
	reg sreg_default_snz;

	always @(*) begin
		{Rh, R} = Rd_in;
		{ SI, ST, SH, SS, SV, SN, SZ, SC } = sreg_in;
		sreg_default_snz = 1;

		(* fullcase *)
		case(op)
		`OP_MOVE: begin
			// Default will copy {R,Rh} <= Rd
			// Do not modify any SREG
			sreg_default_snz = 0;
		end
		`OP_MOVR: begin
			// Copy the Rb input byte to the output
			// Do not modify any SREG
			Rh = 0;
			R = Rr;
			sreg_default_snz = 0;
		end
		`OP_ADD: begin
			R = Rd + Rr + opt_C;
`ifdef SREG_H
			SH = (Rd3 & Rr3) | (Rr3 & !R3) | (!R3 & Rd3);
`endif
			SV = (Rd7 & Rr7 & !R7) | (!Rd7 & !Rr7 & R7);
			SC = (Rd7 & R7) | (Rr7 & !R7) | (!R7 & Rd7);
		end
		`OP_SUB: begin
			R = Rd - Rr - opt_C;
`ifdef SREG_H
			SH = (!Rd3 & Rr3) | (Rr3 & R3) | (R3 & !Rd3);
`endif
			SV = (Rd7 & !Rr7 & !R7) | (!Rd7 & Rr7 & R7);
			SC = (!Rd7 & Rr7) | (Rr7 & R7) | (R7 & !Rd7);
		end
		`OP_ADW, `OP_SBW: begin
			if (op[0]) begin
				// SBW
				{Rh,R} = Rd_in - Rr;
				SC = Rd7 & Rr7 | Rr7 & !R7 | Rd7 & !R7;
			end else begin
				// ADW
				{Rh,R} = Rd_in + Rr;
				SC = R15 & !Rdh7;
			end
			SS = SN ^ SV;
			SV = !Rdh7 & R15;
			SN = R15;
			SZ = { Rh, R } == 0;
			sreg_default_snz = 0;
		end
		`OP_NEG: begin
			R = ~Rd;
`ifdef SREG_H
			SH = R3 | !Rd3;
`endif
			SV = R == 8'h80;
			SC = R != 0;
		end
		`OP_SWAP: begin
			R = { Rd[3:0], Rd[7:4] };
			// no sreg update
			sreg_default_snz = 0;
		end
		`OP_ASR, `OP_ROR: begin
			R = { op[0] ? C : Rd[7], Rd[7:1] };
			SS = SN^SV;
			SC = Rd[0];
		end
		`OP_LSR: begin
			R = { 1'b0, Rd[7:1] };
			SS = SN^SV;
			SC = Rd[0];
			// SN = 0; // this breaks negative flag
		end
		`OP_AND: begin
			R = Rd & Rr;
			SV = 0;
		end
		`OP_EOR: begin
			R = Rd ^ Rr;
			SV = 0;
		end
		`OP_OR: begin
			R = Rd | Rr;
			SV = 0;
		end
		`OP_SREG: begin
			(* full_case *)
			case(Rr[3:0])
			3'b000: SC = use_carry;
			3'b001: SZ = use_carry;
			3'b010: SN = use_carry;
			3'b011: SV = use_carry;
			3'b100: SS = use_carry;
			3'b101: SH = use_carry;
			3'b110: ST = use_carry;
			3'b111: SI = use_carry;
			endcase
			sreg_default_snz = 0;
		end
		// need to infer a multiplier
		`OP_MUL: begin
`ifdef CONFIG_MULU
			{ Rh, R } = Rd * Rr;
			SC = R15;
			SZ = { Rh, R } == 0;
`endif
		end
		endcase

		// many operations reuse this calculation
		if (sreg_default_snz) begin
			SS = SN^SV;
			SN = R7;
			SZ = R_zero;
		end
	end

endmodule


`endif
