`ifndef _avr_cpu_v_
`define _avr_cpu_v_

module avr_cpu(
	input clk,
	input reset,

	// the program memory should provide a new opcode
	// every clock cycle
	output [15:0] pc,
	input [15:0] cdata,

	// the data memory is used for LD/ST as well as the stack
	output [15:0] data_addr,
	output data_wen,
	output data_ren,
	input [7:0] data_read,
	output [7:0] data_write
);
	// register file (as flops, not as BRAM)
	localparam BASE_X = 26;
	localparam BASE_Y = 28;
	localparam BASE_Z = 30;
	reg [7:0] regs[31:0];
	wire [15:0] reg_X = { regs[BASE_X+1], regs[BASE_X] };
	wire [15:0] reg_Y = { regs[BASE_Y+1], regs[BASE_Y] };
	wire [15:0] reg_Z = { regs[BASE_Z+1], regs[BASE_Z] };
	reg [15:0] temp;
	reg [15:0] next_temp;
	reg [15:0] reg_PC;
	reg [15:0] reg_SP;
	reg [15:0] next_SP;
	reg [7:0] sreg;
	localparam SREG_I = 7;
	localparam SREG_T = 6;
	localparam SREG_H = 5;
	localparam SREG_S = 4;
	localparam SREG_V = 3;
	localparam SREG_N = 2;
	localparam SREG_Z = 1;
	localparam SREG_C = 0;
	reg SI, ST, SH, SS, SV, SN, SZ, SC;
	wire [7:0] next_sreg = { SI, ST, SH, SS, SV, SN, SZ, SC };

	// the PC output is almost always the actual PC,
	// although sometimes it is the address for a LPM
	assign pc = next_PC; // lpm_active ? addr : reg_PC;
	reg [15:0] next_PC;
	reg force_PC;

	// Some instructions require an extra cycle;
	// they will set cycle and re-use the previous opcode
	reg [1:0] cycle = 0;
	reg [1:0] next_cycle;

	// Some instruction can cause the next instruction to be skipped,
	// which might be multiple words; this still executes the instruction,
	// but doesn't write any results
	reg skip = 0;
	reg next_skip;
	reg [15:0] prev_opcode;
	wire [15:0] opcode = cycle == 0 ? cdata : prev_opcode;
	reg [15:0] addr;
	reg [15:0] next_addr;
	reg [7:0] wdata;
	reg [7:0] next_wdata;
	reg wen;
	reg ren;
	reg next_wen;
	reg next_ren;

	assign data_addr = next_addr;
	assign data_wen = next_wen;
	assign data_ren = next_ren;
	assign data_write = next_wdata;

	reg invalid_op;
	reg [7:0] R;
	reg [7:0] R1;
	reg [1:0] dest;
	reg [5:0] dest_base;
	localparam DEST_NONE = 0;
	localparam DEST_RD = 1;
	localparam DEST_RDI = 2;
	localparam DEST_WORD = 3;

	// ALU registers
	wire [5:0] alu_r = { opcode[9], opcode[3:0] }; // 0-31
	wire [5:0] alu_d = opcode[8:4]; // 0-31
	wire [5:0] alu_di = { 1'b1, opcode[7:4] }; // 16-31
	wire [7:0] alu_Rr = regs[alu_r];
	wire [7:0] alu_Rd = regs[alu_d];
	wire [7:0] alu_Rdi = regs[alu_di];
	wire [7:0] alu_K = { opcode[11:8], opcode[3:0] };
	wire [5:0] alu_Q = { opcode[13], opcode[15:14], opcode[2:0] };

	// helpers for computing sreg updates
	wire Rdi3 = alu_Rdi[3];
	wire Rdi7 = alu_Rdi[7];
	wire Rd3 = alu_Rd[3];
	wire Rd7 = alu_Rd[7];
	wire Rr3 = alu_Rr[3];
	wire Rr7 = alu_Rr[7];
	wire K3 = alu_K[3];
	wire K7 = alu_K[7];
	wire R3 = R[3];
	wire R7 = R[7];

	// sign extended 12-bit value
	wire [15:0] simm12 = {
		opcode[11],
		opcode[11],
		opcode[11],
		opcode[11],
		opcode[11:0]
	};

	// sign extended 7-bit value for branch instructions
	wire [15:0] simm7 = {
		opcode[9],
		opcode[9],
		opcode[9],
		opcode[9],
		opcode[9],
		opcode[9],
		opcode[9],
		opcode[9],
		opcode[9],
		opcode[9:3]
	};

	// immediate word 6-bit values
	wire [5:0] immw6 = { opcode[7:6], opcode[3:0] };

	always @(posedge clk) if (reset) begin
		cycle <= 0;
		skip <= 0;
		reg_PC <= ~0;
		reg_SP <= ~0;
		addr <= 0;
		wen <= 0;
		ren <= 0;
		wdata <= 0;

		regs[0] <= 0;
		regs[1] <= 0;
		regs[2] <= 0;
		regs[3] <= 0;
		regs[4] <= 0;
		regs[5] <= 0;
		regs[6] <= 0;
		regs[7] <= 0;
		regs[8] <= 0;
		regs[9] <= 0;
		regs[10] <= 0;
		regs[11] <= 0;
		regs[12] <= 0;
		regs[13] <= 0;
		regs[14] <= 0;
		regs[15] <= 0;
		regs[16] <= 0;
		regs[17] <= 0;
		regs[18] <= 0;
		regs[19] <= 0;
		regs[20] <= 0;
		regs[21] <= 0;
		regs[22] <= 0;
		regs[23] <= 0;
		regs[24] <= 0;
		regs[25] <= 0;
		regs[26] <= 0;
		regs[27] <= 0;
		regs[28] <= 0;
		regs[29] <= 0;
		regs[30] <= 0;
		regs[31] <= 0;

	end else begin
		// only advance the PC if we are not in
		// a multi-cycle instruction and not a LPM
		if (force_PC || next_cycle == 0)
			reg_PC <= next_PC;

		reg_SP <= next_SP;
		cycle <= next_cycle;
		skip <= next_skip;
		temp <= next_temp;
		prev_opcode <= opcode;
		sreg <= next_sreg;

		addr <= next_addr;
		wen <= next_wen;
		ren <= next_ren;
		wdata <= next_wdata;

		case(dest)
		0: begin /* Nothing */ end
		DEST_RD: regs[alu_d] <= R;
		DEST_RDI: regs[alu_di] <= R;
		DEST_WORD: begin
			regs[dest_base | 0] <= R;
			regs[dest_base | 1] <= R1;
		end
		endcase

		if (invalid_op)
			$display("%04x.%d: %04x%s%s",
				reg_PC * 2,
				cycle,
				opcode,
				skip ? " SKIP" : "",
				invalid_op ? " INVALID": ""
			);
	end

	always @(*)
	begin
		{ SI, ST, SH, SS, SV, SN, SZ, SC } = sreg;
		force_PC = 0;
		next_PC = reg_PC + 1;
		next_SP = reg_SP;
		next_cycle = 0;
		next_skip = 0;
		next_temp = temp;

		next_wdata = wdata;
		next_addr = addr;
		next_wen = 0;
		next_ren = 0;

		invalid_op = 0;
		R = 0;
		R1 = 0;
		dest = 0;
		dest_base = 0;

		if (skip) begin
			// only a few instructions require an extra skip
			casez(opcode)
			16'b1001_010?_????_111?, // CALL abs22
			16'b1001_010?_????_110?, // JMP abs22
			16'b1001_00??_????_0000: // LDS/STS
			begin
				force_PC = 1;
				next_cycle = 1;
				next_skip = 1;
			end
			endcase
		end else
		casez(opcode)
		16'b0000000000000000: begin
			// NOP
		end
		16'b00000001_????_????: begin
			// MOVW Rd,Rr Move register pair
			dest = DEST_WORD;
			dest_base = { opcode[7:4], 1'b0 };
			R1 = regs[{opcode[3:0], 1'b1 }];
			R = regs[{opcode[3:0], 1'b0 }];
		end
		16'b0000_0011_0???_1???: begin
			// MUL and FMUL, unimplemented
		end

		// 2-operand instructions
		16'b000_0_01_?_?????_????: begin
			// CPC Rd,Rr (no dest, only sreg)
			R = alu_Rd - alu_Rr - sreg[SREG_C];
			SH = !Rd3 & Rr3 | Rr3 & R3 | R3 & !Rd3;
			SS = SN ^ SV;
			SV = (Rd7 & !Rr7 & !R7) | (!Rd7 & Rr7 & R7);
			SN = R7;
			SZ = (R == 0) & sreg[SREG_Z];
			SC = !Rd7 & Rr7 | Rr7 & R7 | R7 & !Rd7;
		end
		16'b000_1_01_?_?????_????: begin
			// CP Rd,Rr (no dest, only sreg)
			R = alu_Rd - alu_Rr;
			SH = !Rd3 & Rr3 | Rr3 & R3 | R3 & !Rd3;
			SS = SN ^ SV;
			SV = (Rd7 & !Rr7 & !R7) | (!Rd7 & Rr7 & R7);
			SN = R7;
			SZ = R == 0;
			SC = !Rd7 & Rr7 | Rr7 & R7 | R7 & !Rd7;
		end
		16'b000010_?_?????_????: begin
			// SBC Rd,Rr
			dest = DEST_RD;
			R = alu_Rd - alu_Rr - sreg[SREG_C];
			SH = !Rd3 & Rr3 | Rr3 & R3 | R3 & !Rd3;
			SS = SN ^ SV;
			SV = (Rd7 & !Rr7 & !R7) | (!Rd7 & Rr7 & R7);
			SN = R7;
			SZ = (R == 0) & sreg[SREG_Z];
			SC = !Rd7 & Rr7 | Rr7 & R7 | R7 & !Rd7;
		end
		16'b000110_?_?????_????: begin
			// SUB Rd,Rr
			dest = DEST_RD;
			R = alu_Rd - alu_Rr;
			SH = !Rd3 & Rr3 | Rr3 & R3 | R3 & !Rd3;
			SS = SN ^ SV;
			SV = (Rd7 & !Rr7 & !R7) | (!Rd7 & Rr7 & R7);
			SN = R7;
			SZ = R == 0;
			SC = !Rd7 & Rr7 | Rr7 & R7 | R7 & !Rd7;
		end
		16'b000_0_11_?_?????_????: begin
			dest = DEST_RD;
			if (alu_r == alu_d) begin
				// LSL Rd when Rd=Rr
				R = { alu_Rd[6:0], 1'b0 };
				SH = Rd3;
				SS = SN ^ SV; 
				SV = SN ^ SC;
				SN = R7;
				SZ = R == 0;
				SC = Rd7;
			end else begin
				// ADD Rd,Rr
				R = alu_Rd + alu_Rr;
				SH = (Rd3 & Rr3) | (Rr3 & !R3) | (Rd3 & !R3);
				SS = SN ^ SV;
				SV = SN ^ SC;
				SN = R7;
				SZ = R == 0;
				SC = Rd7 & Rr7 | Rr7 & !R7 | Rd7 & !R7;
			end
		end
		16'b000_1_11_?_?????_????: begin
			dest = DEST_RD;
			if (alu_r == alu_d) begin
				// ROL Rd when Rd=Rr
				R = { alu_Rd[6:0], sreg[SREG_C] };
				SH = Rd3;
				SS = SN ^ SV;
				SV = SN ^ SC;
				SZ = R == 0;
				SC = Rd7;
			end else begin
				// ADC Rd,Rr
				R = alu_Rd + alu_Rr + sreg[SREG_C];
				SH = Rd3 & Rr3 | Rr3 & !R3 | !R3 & Rd3;
				SS = SN ^ SV;
				SV = Rd7 & Rr7 & !R7 | !Rd7 & !Rr7 & R7;
				SN = R7;
				SZ = R == 0;
				SC = Rd7 & Rr7 | Rr7 & !R7 | !R7 & Rd7;
			end
		end
		16'b0001_00??_????_????: begin
			// CPSE Rd,Rr (no sreg updates)
			if (alu_Rd == alu_Rr)
				next_skip = 1;
		end
		16'b0010_00??_????_????: begin
			// AND Rd,Rr
			dest = DEST_RD;
			R = alu_Rd & alu_Rr;
			SS = SN ^ SV;
			SV = 0;
			SN = R7;
			SZ = R == 0;
		end
		16'b0010_01??_????_????: begin
			// EOR Rd,Rr
			dest = DEST_RD;
			R = alu_Rd ^ alu_Rr;
			SS = SN ^ SV;
			SV = 0;
			SN = R7;
			SZ = R == 0;
		end
		16'b0010_10??_????_????: begin
			// OR Rd,Rr
			dest = DEST_RD;
			R = alu_Rd | alu_Rr;
			SS = SN ^ SV;
			SV = 0;
			SN = R7;
			SZ = R == 0;
		end
		16'b0010_11??_????_????: begin
			// MOV Rd,Rr (no sreg updates)
			dest = DEST_RD;
			R = alu_Rr;
		end

		// Register-immediate operations
		16'b0011_????_????_????: begin
			// CPI Rd,K (only updates status register, so no dest)
			R = alu_Rdi - alu_K;
			SH = (K3 & !Rdi3) | (K3 & R3) | (R3 & !Rdi3);
			SS = SN ^ SV;
			SV = (Rdi7 & !K7 & !R7) | (K7 & R7 & !Rdi7);
			SN = R7;
			SZ = R == 0;
			SC = (!Rdi7 & K7) | (K7 & R7) | (R7 & !Rdi7);
		end
		16'b0100_????_????_????: begin
			// SBCI Rd, K
			dest = DEST_RDI;
			R = alu_Rdi - alu_K - sreg[SREG_C];
			SH = !Rdi3 & K3 | K3 & !R3 | R3 & !Rdi3;
			SS = SN ^ SV;
			SV = Rdi7 & !K7 & !R7 | !Rdi7 & K7 & R7;
			SN = R7;
			SZ = (R == 0) & sreg[SREG_Z];
			SC = !Rdi7 & K7 | K7 & R7 | R7 & !Rdi7;
		end
		16'b0101_????_????_????: begin
			// SUBI Rd, K
			dest = DEST_RDI;
			R = alu_Rdi - alu_K;
			SH = (K3 & !Rdi3) | (K3 & R3) | (R3 & !Rdi3);
			SS = SN ^ SV;
			SV = (Rdi7 & !K7 & !R7) | (K7 & R7 & !Rdi7);
			SN = R7;
			SZ = R == 0;
			SC = (!Rdi7 & K7) | (K7 & R7) | (R7 & !Rdi7);
		end
		16'b0110_????_????_????: begin
			// ORI Rd,K or SBR Rd, K
			dest = DEST_RDI;
			R = alu_Rdi | alu_K;
			SS = SN ^ SV;
			SV = 0;
			SN = R7;
			SZ = R == 0;
		end
		16'b0111_????_????_????: begin
			// ANDI Rd,K or CBR Rd, K
			dest = DEST_RDI;
			R = alu_Rdi & alu_K;
			SS = SN ^ SV;
			SV = 0;
			SN = R7;
			SZ = R == 0;
		end

		// LDS rd,i  / STS i,rd
		16'b100100_?_?????_0000:
			// No sreg update
			// 2 cycles
			// Load or store instructions
			// followed by 16-bit immediate SRAM address
			case(cycle)
			2'b00: begin
				// wait for the next read to get the address
				force_PC = 1;
				next_cycle = 1;
			end
			2'b01: begin
				next_addr = cdata;
				if (opcode[9] == 0) begin
					// LDS: request a read of the addr
					// wait at this PC
					next_cycle = 2;
					next_ren = 1;
				end else begin
					// STS: write to that address
					// no extra cycle required
					next_wdata = regs[alu_d];
					next_wen = 1;
				end
			end
			2'b10: begin
				// only LDS, store the data read
				dest = DEST_RD;
				R = data_read;
			end
			endcase

		16'b1001_00??_????_1100,
		16'b1001_00??_????_1101,
		16'b1001_00??_????_1110:
			// ST / LD Rd, X / X- / X+ (no sreg update)
			case(cycle)
			2'b00: begin
				if (opcode[9]) begin
					// STS (no extra cycle needed)
					next_wen = 1;
					next_wdata = alu_Rd;
				end else begin
					// LD
					next_ren = 1;
					next_cycle = 1;
				end

				case(opcode[1:0])
				2'b00: next_addr = reg_X;
				2'b01: begin
					// post-increment
					dest = DEST_WORD;
					dest_base = BASE_X;
					{ R1, R } = reg_X + 1;
					next_addr = reg_X;
				end
				2'b10: begin
					// pre-decrement
					dest = DEST_WORD;
					dest_base = BASE_X;
					{ R1, R } = reg_X - 1;
					next_addr = reg_X - 1;
				end
				endcase
			end
			2'b01: begin
				// extra cycle only for LD
				dest = DEST_RD;
				R = data_read;
			end
			endcase

		// LD Rd, Z+Q (no status update)
		16'b10?0_??0?_????_0???:
			case(cycle)
			2'b00: begin
				next_cycle = 1;
				next_ren = 1;
				next_addr = reg_Z + alu_Q;
			end
			2'b01: begin
				dest = DEST_RD;
				R = data_read;
			end
			endcase

		// LD Rd, -Z / Z+ (no sreg update)
		16'b1001_000?_????_0001,
		16'b1001_000?_????_0010:
			case(cycle)
			2'b00: begin
				next_cycle = 1;
				next_ren = 1;
				case(opcode[1:0])
				2'b01: begin
					// post increment
					dest = DEST_WORD;
					dest_base = BASE_Z;
					{ R1, R } = reg_Z + 1;
					next_addr = reg_Z;
				end
				2'b10: begin
					// predecrement
					dest = DEST_WORD;
					dest_base = BASE_Z;
					{ R1, R } = reg_Z - 1;
					next_addr = reg_X - 1;
				end
				default: invalid_op = 1;
				endcase
			end
			2'b01: begin
				dest = DEST_RD;
				R = data_read;
			end
			endcase

		// LD Rd, Y+Q (no sreg update)
		16'b10?0_??0?_????_1???:
			case(cycle)
			2'b00: begin
				next_cycle = 1;
				next_addr = reg_Y + alu_Q;
				next_ren = 1;
			end
			2'b01: begin
				dest = DEST_RD;
				R = data_read;
			end
			endcase

		// LPM/ELPM Rd, Z / Z+
		16'b1001_000?_????_0100,
		16'b1001_000?_????_0101:
			case(cycle)
			2'b00: begin
				// start a read of the program memory space
				// storing the real next PC into the temp reg
				// PC is in words, not bytes
				next_cycle = 1;
				force_PC = 1;
				next_PC = reg_Z >> 1;
				next_temp = reg_PC;
			end
			2'b01: begin
				// store the correct byte of read data,
				// based on the bottom bit of Z
				dest = DEST_RD;
				R = reg_Z[0] ? cdata[15:8] : cdata[7:0];
				next_cycle = 2;

				// and return to the program flow by
				// reading the temp reg.
				next_PC = temp;
				force_PC = 1;
			end
			2'b10: begin
				if(opcode[1:0] == 2'b01) begin
					// Z+ addressing, 3 cycles
					dest = DEST_WORD;
					dest_base = BASE_Z;
					{ R1, R } = reg_Z + 1;
				end
			end
			endcase

/*
		16'b1001001_?????_0100: begin
			// XCH Z,Rd
			invalid_op = 1;
		end
		16'b1001001_?????_0101: begin
			// LAS Z,Rd
			invalid_op = 1;
		end
		16'b1001001_?????_0110: begin
			// LAC Z,Rd
			invalid_op = 1;
		end
		16'b1001001_?????_0111: begin
			// LAT Z,Rd
			invalid_op = 1;
		end
		16'b1001001_?????_1111: begin
			// POP/PUSH Rd
			invalid_op = 1;
		end
*/

		// One operand instructions

		// COM Rd
		16'b1001010_?????_0000: begin
			dest = DEST_RD;
			R = 8'hFF - alu_Rd;
			SS = SN ^ SV;
			SV = 0;
			SN = R7;
			SZ = R == 0;
			SC = 1;
		end

		// NEG Rd
		16'b1001010_?????_0001: begin
			dest = DEST_RD;
			R = 8'h00 - alu_Rd;
			SH = R3 | !Rd3;
			SS = SN ^ SV;
			SV = R7 && (R[6:0] == 0);
			SN = R7;
			SZ = R == 0;
			SC = R != 0;
		end

		// SWAP Rd, no sreg updates
		16'b1001010_?????_0010: begin
			dest = DEST_RD;
			R = { alu_Rd[3:0], alu_Rd[7:4] };
		end

		// INC Rd
		16'b1001010_?????_0011: begin
			dest = DEST_RD;
			R = alu_Rd + 1;
			SS = SN ^ SV;
			SV = R7 & (R[6:0] == 0);
			SN = R7;
			SZ = R == 0;
		end
/*
		// RESERVED
		16'b1001010_?????_0100: begin
			invalid_op = 1;
		end
*/
		// ASR Rd
		16'b1001010_?????_0101: begin
			dest = DEST_RD;
			R = { alu_Rd[7], alu_Rd[7:1] };
			SS = SN ^ SV;
			SV = SN ^ SC;
			SN = R7;
			SZ = R == 0;
			SC = alu_Rd[0];
		end

		// LSR Rd
		16'b1001010_?????_0110: begin
			dest = DEST_RD;
			R = { 1'b0, alu_Rd[7:1] };
			SS = SN ^ SV;
			SV = SN ^ SC;
			SN = 0;
			SZ = R == 0;
			SC = alu_Rd[0];
		end

		// ROR Rd
		16'b1001010_?????_0111: begin
			dest = DEST_RD;
			R = { sreg[SREG_C], alu_Rd[7:1] };
			SS = SN ^ SV;
			SV = SN ^ SC;
			SN = R7;
			SZ = R == 0;
			SC = alu_Rd[0];
		end
/*
		16'b10010100_?_???_1000: begin
			// SEx/CLx Status register clear/set bit
			invalid_op = 1;
		end
*/

		// Zero-operand instructions

		// RET
		16'b1001010100001000:
			case(cycle)
			2'b00: begin
				next_cycle = 1;
				next_SP = reg_SP + 1;
				next_addr = next_SP;
				next_ren = 1;
			end
			2'b01: begin
				next_cycle = 2;
				next_temp[7:0] = data_read;
				next_SP = reg_SP + 1;
				next_addr = next_SP;
				next_ren = 1;
			end
			2'b10: begin
				next_PC = { temp[7:0], data_read };
			end
			endcase
/*
		16'b1001010100011000: begin
			// RETI
			invalid_op = 1;
		end
		16'b10010101001x1000: begin
			// RESERVED
			invalid_op = 1;
		end
		16'b1001010101??1000: begin
			// RESERVED
			invalid_op = 1;
		end
		16'b1001010110001000: begin
			// SLEEP
			invalid_op = 1;
		end
		16'b1001010110011000: begin
			// BREAK
			invalid_op = 1;
		end
		16'b1001010110101000: begin
			// WDR
			invalid_op = 1;
		end
		16'b1001010110111000: begin
			// RESERVED
			invalid_op = 1;
		end
		16'b10010101110_?_1000: begin
			// LPM/ELPM
			invalid_op = 1;
		end
		16'b1001010111101000: begin
			// SPM
			invalid_op = 1;
		end
		16'b1001010111111000: begin
			// SPM X+
			invalid_op = 1;
		end

		// misc instructions
		16'b1001010_?_000_?_1001: begin
			// Indirect jump/call to Z or EIND:Z
			invalid_op = 1;
		end
*/
		// DEC Rd
		16'b1001010_?????_1010: begin
			dest = DEST_RD;
			R = alu_Rd - 1;
			SS = SN ^ SV;
			SV = !R7 & (R[6:0] != 0);
			SN = R7;
			SZ = R == 0;
		end
/*
		16'b10010100_????_1011: begin
			// DES round k
			invalid_op = 1;
		end
*/

		// JMP abs22, 3 cycles
		16'b1001_010?_????_110?:
			// 16 bits in next word
			case(cycle)
			2'b00: begin
				next_cycle = 1;
				force_PC = 1;
			end
			2'b01: begin
				// cdata now has the destination address
				// start pre-fetch of next_PC
				next_PC = cdata;
				force_PC = 1;
				next_cycle = 2;
			end
			2'b10: begin
				// should be ready
			end
			endcase

		// CALL abs22
		16'b1001_010?_????_111?:
			// 16 bits in next word
			case(cycle)
			2'b00: begin
				next_cycle = 1;
				force_PC = 1;
			end
			2'b01: begin
				// cdata now has the destination address
				// start pushing next_PC
				next_temp = cdata;
				next_addr = reg_SP;
				next_SP = reg_SP + 1;
				next_wdata = next_PC[7:0];
				next_wen = 1;
				next_cycle = 2;
			end
			2'b10: begin
				// write the second half of the return address
				next_addr = reg_SP;
				next_SP = reg_SP + 1;
				next_wdata = next_PC[15:8];
				next_wen = 1;
				next_cycle = 3;
			end
			2'b11: begin
				// 22-bit PC has extra bits in opcode
				// but we are a 16-bit PC CPU, so ignored
				next_PC = temp;
			end
			endcase

		// ADIW/SBIW Rp, uimm6
		16'b1001_011?_????_????: begin
			dest = DEST_WORD;
			dest_base = { opcode[5:4], 3'b000 };

			if (opcode[8]) begin
				// SBIW
				{R1,R} = { regs[dest_base|1], regs[dest_base] } - immw6;
				SC = R1[7] & !regs[dest_base|1][7];
			end else begin
				// ADIW
				{R1,R} = { regs[dest_base|1], regs[dest_base] } + immw6;
				SC = !R1[7] & regs[dest_base|1][7]; // Rdh7
			end

			SS = SN^SV;
			SV = !regs[dest_base|1][7] & R1[7]; // !Rdh7 & R15
			SN = R1[7]; // R15
			SZ = { R1, R } == 0;
		end
/*
		16'b100110_?_0_?????_???: begin
			// CBI/SBI a,b (clear/set IO bit)
			invalid_op = 1;
		end
		16'b100110_?_1_?????_???: begin
			// SBIC/SBIS a,b (IO bit test)
			invalid_op = 1;
		end
		16'b100111_?_?????_????: begin
			// MUL unsigned R1:R0 = Rr*Rd
			invalid_op = 1;
		end
*/
		// OUT to IO space (no sreg update)
		16'b1011_1???_????_????: begin
			next_addr = { 10'b10_0000_0000, opcode[14:13], opcode[3:0] };
			next_wdata = alu_Rd;
			next_wen = 1;
		end

		// RJMP to PC + simm12
		16'b1100_????????????: begin
			next_PC = reg_PC + simm12 + 1;
		end

		// RCALL to PC + simm12
		16'b1101_????????????:
			case(cycle)
			2'b00: begin
				// push the first half of the PC
				next_wen = 1;
				next_addr = reg_SP;
				next_SP = reg_SP - 1;
				next_wdata = next_PC[7:0]; // pc + 1
				next_cycle = 1;
			end
			2'b01: begin
				// push the second half
				next_wen = 1;
				next_addr = reg_SP;
				next_SP = reg_SP - 1;
				next_wdata = next_PC[15:8]; // pc + 1
				next_cycle = 2;
			end
			2'b10: begin
				// and do the jump
				next_PC = reg_PC + simm12 + 1;
			end
			endcase

		// LDI Rd, K (no sreg updates)
		16'b1110_????_????_????: begin
			dest = DEST_RDI;
			R = alu_K;
		end

		// BRBS - Branch if bit in SREG is set
		16'b111100_???????_???: begin
			if (sreg[opcode[2:0]])
				next_PC = reg_PC + simm7 + 1;
		end
		// BRBC - Branch if bit in SREG is clear
		16'b111101_???????_???: begin
			if (!sreg[opcode[2:0]])
				next_PC = reg_PC + simm7 + 1;
		end
/*
		16'b111110_?_?????_0_???: begin
			// BLD/BST register bit to STATUS.T
			invalid_op = 1;
		end
		16'b111111_?_?????_0_???: begin
			// SBRC/SBRS skip if register bit b equals B
			invalid_op = 1;
		end
		16'b11111_??_?????_1_???: begin
			// RESERVED
			invalid_op = 1;
		end
*/
		default: begin
			invalid_op = 1;
		end
		endcase
	end
endmodule

`endif
