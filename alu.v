`ifndef _alu_v_
`define _alu_v_

`define OP_MOVE	4'h0
`define OP_MOVR	4'h1
`define OP_ADD	4'h2
`define OP_SUB	4'h3
`define OP_ADW	4'h4
`define OP_SBW	4'h5 // must be OP_ADW | 1
`define OP_LSR	4'h6
`define OP_NEG	4'h7
`define OP_ASR	4'h8
`define OP_ROR	4'h9 // must be OP_ASR | 1
`define OP_SWAP	4'hA
`define OP_AND	4'hB
`define OP_OR	4'hC
`define OP_EOR	4'hD
`define OP_SREG	4'hE

module alu(
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

	always @(*) begin
		{Rh, R} = Rd_in;
		{ SI, ST, SH, SS, SV, SN, SZ, SC } = sreg_in;

		case(op)
		`OP_ADD: begin
			R = Rd + Rr + opt_C;
			SH = (Rd3 & Rr3) | (Rr3 & !R3) | (!R3 & Rd3);
			SS = SN^SV;
			SV = (Rd7 & Rr7 & !R7) | (!Rd7 & !Rr7 & R7);
			SN = R7;
			SZ = R == 0;
			SC = (Rd7 & R7) | (Rr7 & !R7) | (!R7 & Rd7);
		end
		`OP_SUB: begin
			R = Rd - Rr - opt_C;
			SH = (!Rd3 & Rr3) | (Rr3 & R3) | (R3 & !Rd3);
			SS = SN^SV;
			SV = (Rd7 & !Rr7 & !R7) | (!Rd7 & Rr7 & R7);
			SN = R7;
			SZ = R == 0;
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
		end
		`OP_NEG: begin
			R = ~Rd;
			SH = R3 | !Rd3;
			SV = R == 8'h80;
			SN = R7;
			SC = R != 0;
			SZ = R == 0;
			SS = SN^SV;
		end
		`OP_SWAP: begin
			R = { Rd[3:0], Rd[7:4] };
			// no sreg update
		end
		`OP_ASR, `OP_ROR: begin
			R = { op[0] ? C : Rd[7], Rd[7:1] };
			SS = SN^SV;
			SV = SN^SC;
			SN = R7;
			SZ = R == 0;
			SC = Rd[0];
		end
		`OP_LSR: begin
			R = { 1'b0, Rd[7:1] };
			SS = SN^SV;
			SV = SN^SC;
			SN = 0;
			SZ = R == 0;
			SC = Rd[0];
		end
		`OP_AND: begin
			R = Rd & Rr;
			SS = SN^SV;
			SV = 0;
			SN = R7;
			SZ = R == 0;
		end
		`OP_EOR: begin
			R = Rd ^ Rr;
			SS = SN^SV;
			SV = 0;
			SN = R7;
			SZ = R == 0;
		end
		`OP_OR: begin
			R = Rd | Rr;
			SS = SN^SV;
			SV = 0;
			SN = R7;
			SZ = R == 0;
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
		end
		`OP_MOVE: begin
			// Do not modify any SREG
			{Rh,R} = Rd_in;
		end
		`OP_MOVR: begin
			// Do not modify any SREG
			Rh = 0;
			R = Rr;
		end
		endcase
	end

endmodule

`endif
