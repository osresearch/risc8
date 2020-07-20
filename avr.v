`ifndef _avr_cpu_v_
`define _avr_cpu_v_
`include "alu.v"

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
	//reg [7:0] regs[31:16];
	wire [15:0] reg_X = { regs[BASE_X+1], regs[BASE_X] };
	wire [15:0] reg_Y = { regs[BASE_Y+1], regs[BASE_Y] };
	wire [15:0] reg_Z = { regs[BASE_Z+1], regs[BASE_Z] };
	reg [15:0] temp;
	reg [15:0] next_temp;
	reg [15:0] reg_PC;
	reg [15:0] reg_SP;
	reg [15:0] next_SP;
	reg [7:0] sreg;
	reg SI, ST, SH, SS, SV, SN, SZ, SC;

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
	reg [5:0] dest;
	reg alu_store;
	reg alu_word;

	// opcode registers
	wire [5:0] op_Rr = { opcode[9], opcode[3:0] }; // 0-31
	wire [5:0] op_Rd = opcode[8:4]; // 0-31
	wire [5:0] op_Rdi = { 1'b1, opcode[7:4] }; // 16-31
	wire [7:0] op_K = { opcode[11:8], opcode[3:0] };
	wire [5:0] op_Q = { opcode[13], opcode[15:14], opcode[2:0] };

	// IN and OUT instructions
	wire [5:0] io_addr = { opcode[10:9], opcode[3:0] };

	// LD vs ST is in the 9th bit
	wire is_store = opcode[9];

	// sign extended 12-bit value
	wire [15:0] simm12 = {
		{4{opcode[11]}},
		opcode[11:0]
	};

	// sign extended 7-bit value for branch instructions
	wire [15:0] simm7 = {
		{9{opcode[9]}},
		opcode[9:3]
	};

	// immediate word 6-bit values
	wire [5:0] immw6 = { opcode[7:6], opcode[3:0] };

	// ALU to perform the operations
	wire [7:0] next_sreg;
	reg [15:0] alu_Rd;
	reg [7:0] alu_Rr;
	reg [3:0] alu_op;
	wire [15:0] alu_out;

	alu avr_alu(
		.clk(clk),
		.reset(reset),
		.op(alu_op),
		.Rd_in(alu_Rd),
		.Rr_in(alu_Rr),
		.R_out(alu_out),
		.sreg_in(sreg),
		.sreg_out(next_sreg)
	);

	always @(posedge clk) if (reset) begin
		cycle <= 0;
		skip <= 0;
		reg_PC <= 0;
		reg_SP <= 16'h1000;
		addr <= 0;
		wen <= 0;
		ren <= 0;
		wdata <= 0;

		regs[ 0] <= 0; regs[ 1] <= 0; regs[ 2] <= 0; regs[ 3] <= 0;
		regs[ 4] <= 0; regs[ 5] <= 0; regs[ 6] <= 0; regs[ 7] <= 0;
		regs[ 8] <= 0; regs[ 9] <= 0; regs[10] <= 0; regs[11] <= 0;
		regs[12] <= 0; regs[13] <= 0; regs[14] <= 0; regs[15] <= 0;
		regs[16] <= 0; regs[17] <= 0; regs[18] <= 0; regs[19] <= 0;
		regs[20] <= 0; regs[21] <= 0; regs[22] <= 0; regs[23] <= 0;
		regs[24] <= 0; regs[25] <= 0; regs[26] <= 0; regs[27] <= 0;
		regs[28] <= 0; regs[29] <= 0; regs[30] <= 0; regs[31] <= 0;

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

		if (alu_store) begin
			regs[dest] <= alu_out[7:0];
			if (alu_word)
				regs[dest|1] <= alu_out[15:8];
		end

		//if (invalid_op)
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
		if (reset)
			next_PC = 0;
		else
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

		// Default is to not store, but if commiting to the register
		// file is selected, then to store to the Rd value
		alu_store = 0;
		alu_word = 0;
		dest = op_Rd;

		alu_op = `OP_MOVE;
		alu_Rd = regs[op_Rd];
		alu_Rr = regs[op_Rr];

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
			dest = { opcode[7:4], 1'b0 };
			alu_word = 1;
			alu_store = 1;
			alu_Rd = {
				regs[{opcode[3:0], 1'b1 }],
				regs[{opcode[3:0], 1'b0 }]
			};
		end
		16'b0000_0011_0???_1???: begin
			// MUL and FMUL, unimplemented
			invalid_op = 1;
		end

		// 2-operand instructions
		16'b000_0_01_?_?????_????: begin
			// CPC Rd,Rr (no dest, only sreg)
			alu_op = `OP_SBC;
		end
		16'b000_1_01_?_?????_????: begin
			// CP Rd,Rr (no dest, only sreg)
			alu_op = `OP_SUB;
		end
		16'b000010_?_?????_????: begin
			// SBC Rd,Rr
			alu_op = `OP_SBC;
			alu_store = 1;
		end
		16'b000110_?_?????_????: begin
			// SUB Rd,Rr
			alu_op = `OP_SUB;
			alu_store = 1;
		end
		16'b000_0_11_?_?????_????: begin
			alu_store = 1;
			if (op_Rd == op_Rr)
				alu_op = `OP_LSL; // LSL Rd when Rd=Rr
			else
				alu_op = `OP_ADD; // ADD Rd,Rr
		end
		16'b000_1_11_?_?????_????: begin
			alu_store = 1;
			if (op_Rd == op_Rr)
				alu_op = `OP_ROL; // ROL Rd when Rd=Rr
			else
				alu_op = `OP_ADC; // ADC Rd,Rr
		end

		16'b0010_00??_????_????: begin
			// AND Rd,Rr
			alu_store = 1;
			alu_op = `OP_AND;
		end
		16'b0010_01??_????_????: begin
			// EOR Rd,Rr
			alu_store = 1;
			alu_op = `OP_EOR;
		end
		16'b0010_10??_????_????: begin
			// OR Rd,Rr
			alu_store = 1;
			alu_op = `OP_OR;
		end
		16'b0010_11??_????_????: begin
			// MOV Rd,Rr (no sreg updates)
			alu_store = 1;
		end

		// Register-immediate operations
		16'b0011_????_????_????: begin
			// CPI Rd,K (only updates status register, so no dest)
			alu_op = `OP_SUB;
			alu_Rd = regs[op_Rdi];
			alu_Rr = op_K;
			alu_store = 1;
		end
		16'b0100_????_????_????: begin
			// SBCI Rd, K
			alu_op = `OP_SBC;
			alu_Rd = regs[op_Rdi];
			alu_Rr = op_K;
			alu_store = 1;
		end
		16'b0101_????_????_????: begin
			// SUBI Rd, K
			alu_op = `OP_SUB;
			alu_Rd = regs[op_Rdi];
			alu_Rr = op_K;
			alu_store = 1;
		end
		16'b0110_????_????_????: begin
			// ORI Rd,K or SBR Rd, K
			alu_op = `OP_OR;
			alu_Rd = regs[op_Rdi];
			alu_Rr = op_K;
			alu_store = 1;
		end
		16'b0111_????_????_????: begin
			// ANDI Rd,K or CBR Rd, K
			alu_op = `OP_AND;
			alu_Rd = regs[op_Rdi];
			alu_Rr = op_K;
			alu_store = 1;
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
				if (is_store) begin
					// STS: write to that address
					// no extra cycle required
					next_wdata = regs[op_Rd];
					next_wen = 1;
				end else begin
					// LDS: request a read of the addr
					// wait at this PC
					next_cycle = 2;
					next_ren = 1;
				end
			end
			2'b10: begin
				// only LDS, store the data read
				alu_store = 1;
				alu_Rd = data_read;
			end
			endcase

		16'b1001_00??_????_1100,
		16'b1001_00??_????_1101,
		16'b1001_00??_????_1110:
			// ST / LD Rd, X / X- / X+ (no sreg update)
			case(cycle)
			2'b00: begin
				if (is_store) begin
					// STS (no extra cycle needed)
					next_wen = 1;
					next_wdata = regs[op_Rd];
				end else begin
					// LD
					next_ren = 1;
					next_cycle = 1;
				end

				dest = BASE_X;
				alu_store = 1;
				alu_word = 1;
				alu_Rd = reg_X;
				alu_Rr = 1;

				case(opcode[1:0])
				2'b00: next_addr = reg_X;
				2'b01: begin
					// post-increment
					alu_op = `OP_ADD;
					next_addr = reg_X;
				end
				2'b10: begin
					// pre-decrement
					alu_op = `OP_SUB;
					next_addr = reg_X - 1;
				end
				endcase
			end
			2'b01: begin
				// extra cycle only for LD to store into Rd
				alu_store = 1;
				alu_Rd = data_read;
			end
			endcase

		// ST / LD Rd, Z+Q (no status update)
		16'b10?0_????_????_0???:
			case(cycle)
			2'b00: begin
				if (is_store) begin
					// ST instruction
					next_wen = 1;
					next_wdata = regs[op_Rd];
				end else begin
					// LD instruction
					next_cycle = 1;
					next_ren = 1;
				end

				next_addr = reg_Z + op_Q;
			end
			2'b01: begin
				alu_store = 1;
				alu_Rd = data_read;
			end
			endcase

		// ST / LD Rd, -Z / Z+ (no sreg update)
		16'b1001_00??_????_0001,
		16'b1001_00??_????_0010:
			case(cycle)
			2'b00: begin
				if (is_store) begin
					// ST instruction
					next_wen = 1;
					next_wdata = regs[op_Rd];
				end else begin
					// LD instruction
					next_ren = 1;
					next_cycle = 1;
				end

				case(opcode[1:0])
				2'b01: begin
					// post increment
					alu_store = 1;
					alu_word = 1;
					dest = BASE_Z;
					alu_op = `OP_ADD;
					alu_Rd = reg_Z;
					alu_Rr = 1;
					next_addr = reg_Z;
				end
				2'b10: begin
					// predecrement
					alu_store = 1;
					alu_word = 1;
					dest = BASE_Z;
					alu_op = `OP_SUB;
					alu_Rd = reg_Z;
					alu_Rr = 1;
					next_addr = reg_Z - 1;
				end
				endcase
			end
			2'b01: begin
				alu_store = 1;
				alu_Rd = data_read;
			end
			endcase

		// ST / LD Rd, Y+Q (no sreg update)
		16'b10?0_????_????_1???:
			case(cycle)
			2'b00: begin
				if (is_store) begin
					// ST instruction
					next_wen = 1;
					next_wdata = regs[op_Rd];
				end else begin
					// LD instruction
					next_cycle = 1;
					next_ren = 1;
				end

				next_addr = reg_Y + op_Q;
			end
			2'b01: begin
				alu_store = 1;
				alu_Rd = data_read;
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
				alu_store = 1;
				alu_Rd = reg_Z[0] ? cdata[15:8] : cdata[7:0];
				next_cycle = 2;

				// and return to the program flow by
				// reading the temp reg.
				next_PC = temp;
				force_PC = 1;
			end
			2'b10: begin
				if(opcode[1:0] == 2'b01) begin
					// Z+ addressing, 3 cycles
					alu_store = 1;
					alu_word = 1;
					dest = BASE_Z;
					alu_op = `OP_ADD;
					alu_Rd = reg_Z;
					alu_Rr = 1;
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
*/
		// PUSH Rd
		16'b1001_001?_????_1111: begin
			// enqueue the data write
			next_wen = 1;
			next_addr = reg_SP;
			next_SP = reg_SP - 1;
			next_wdata = alu_Rd;
		end
		// POP Rd
		16'b1001_000?_????_1111:
			case(cycle)
			2'b00: begin
				// start the read
				next_ren = 1;
				next_addr = reg_SP + 1;
				next_SP = reg_SP + 1;
			end
			2'b01: begin
				alu_store = 1;
				alu_Rd = data_read;
			end
			endcase

		// One operand instructions

		// COM Rd
		16'b1001010_?????_0000: begin
			alu_store = 1;
			alu_op = `OP_SUB;
			alu_Rd = 8'hFF;
			alu_Rr = regs[op_Rd];
		end

		// NEG Rd
		16'b1001010_?????_0001: begin
			alu_store = 1;
			alu_op = `OP_SUB;
			alu_Rd = 8'h00;
			alu_Rr = regs[op_Rd];
		end

		// SWAP Rd, no sreg updates
		16'b1001010_?????_0010: begin
			alu_store = 1;
			alu_Rd = { regs[op_Rd][3:0], regs[op_Rd][7:4] };
		end

/*
		// RESERVED
		16'b1001010_?????_0100: begin
			invalid_op = 1;
		end
*/
		// ASR Rd
		16'b1001010_?????_0101: begin
			alu_store = 1;
			alu_op = `OP_ASR;
		end

		// LSR Rd
		16'b1001010_?????_0110: begin
			alu_store = 1;
			alu_op = `OP_LSR;
		end

		// ROR Rd
		16'b1001_010?_????_0111: begin
			alu_store = 1;
			alu_op = `OP_ROR;
		end

		// CLx Status register clear bit
		16'b1001_0100_1???_1000: begin
			(* full_case *)
			case(opcode[6:4])
			3'b000: SC = 0;
			3'b001: SZ = 0;
			3'b010: SN = 0;
			3'b011: SV = 0;
			3'b100: SS = 0;
			3'b101: SH = 0;
			3'b110: ST = 0;
			3'b111: SI = 0;
			endcase
		end

		// CLx Status register clear bit
		16'b1001_0100_0???_1000: begin
			(* full_case *)
			case(opcode[6:4])
			3'b000: SC = 1;
			3'b001: SZ = 1;
			3'b010: SN = 1;
			3'b011: SV = 1;
			3'b100: SS = 1;
			3'b101: SH = 1;
			3'b110: ST = 1;
			3'b111: SI = 1;
			endcase
		end

		// Zero-operand instructions

		// RET
		16'b1001010100001000:
			case(cycle)
			2'b00: begin
				next_cycle = cycle + 1;
				next_SP = reg_SP + 1;
				next_addr = next_SP;
				next_ren = 1;
			end
			2'b01: begin
				next_cycle = cycle + 1;
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

*/
		// misc instructions

		// DEC Rd
		16'b1001010_?????_1010: begin
			alu_store = 1;
			alu_op = `OP_SUB;
			alu_Rr = 1;
		end

		// INC Rd
		16'b1001010_?????_0011: begin
			alu_store = 1;
			alu_op = `OP_ADD;
			alu_Rr = 1;
		end
/*
		16'b10010100_????_1011: begin
			// DES round k
			invalid_op = 1;
		end
*/

		// CPSE Rd,Rr (no sreg updates)
		16'b0001_00??_????_????:
			if (alu_Rd == alu_Rr)
				next_skip = 1;

		// SBRC/SBRS skip if register bit b equals B
		16'b1111_110?_????_0???, // SBRC
		16'b1111_111?_????_0???: // SBRS
			if (alu_Rd[opcode[2:0]] == opcode[9])
				next_skip = 1;

		// BRBS/BRBC - Branch if bit in SREG is set/clear
		16'b1111_00??_????_????, // BRBS
		16'b1111_01??_????_????: // BRBC
			if (sreg[opcode[2:0]] != opcode[9])
				next_PC = reg_PC + simm7 + 1;

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
			alu_store = 1;
			alu_word = 1;
			dest = { opcode[5:4], 3'b000 };
			alu_Rd = { regs[dest|1], regs[dest] };
			alu_Rr = immw6;

			if (opcode[8])
				alu_op = `OP_SBW;
			else
				alu_op = `OP_ADW;
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
		// the ones for registers are handled here,
		// otherwise the external controller will handle it
		16'b1011_1???_????_????: begin
			next_wen = 1;
			next_wdata = alu_Rd;
			next_addr = io_addr + 8'h20;

			case(io_addr)
			6'h3D: next_SP[ 7:0] = alu_Rd;
			6'h3E: next_SP[15:8] = alu_Rd;
			6'h3F: { SI, ST, SH, SS, SV, SN, SZ, SC } = alu_Rd;
			default: begin
				// nothing to do here; SOC handles it
			end
			endcase
		end

		// IN from IO space (no sreg update, should be 1 cycle)
		// the registers ones are handled here, otherwise
		// the external SOC will handle it.
		16'b1011_0???_????_????: begin
			alu_store = 1;
			next_addr = io_addr + 8'h20;
			next_ren = 1;

			case(io_addr)
			6'h3D: alu_Rd = reg_SP[ 7:0];
			6'h3E: alu_Rd = reg_SP[15:8];
			6'h3F: alu_Rd = sreg;
			default: alu_Rd = data_read; // from the SOC
			endcase
		end

		// IJMP Z - Indirect jump/call to Z or EIND:Z
		16'b1001010_?_000_?_1001:
			// 2 cycles
			case(cycle)
			2'b00: next_cycle = 1;
			2'b01: next_PC = reg_Z;
			endcase

		// RJMP to PC + simm12
		16'b1100_????????????:
			// 2 cycles
			case(cycle)
			2'b00: next_cycle = 1;
			2'b01: next_PC = reg_PC + simm12 + 1;
			endcase

		// RCALL to PC + simm12
		16'b1101_????????????:
			// 3 cycles
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
			alu_store = 1;
			alu_Rd = op_K;
		end

/*
		16'b111110_?_?????_0_???: begin
			// BLD/BST register bit to STATUS.T
			invalid_op = 1;
		end
*/
		16'b11111_??_?????_1_???: begin
			// RESERVED
			invalid_op = 1;
		end
		default: begin
			invalid_op = 1;
		end
		endcase
	end
endmodule

`endif
