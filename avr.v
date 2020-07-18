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
	reg [7:0] regs[31:0];
	wire [15:0] reg_X = { regs[25], regs[24] };
	wire [15:0] reg_Y = { regs[29], regs[28] };
	wire [15:0] reg_Z = { regs[31], regs[30] };
	reg [15:0] reg_PC;
	reg [15:0] reg_SP;
	reg [15:0] next_SP;
	reg [7:0] sreg;
	reg [7:0] next_sreg;
	localparam SREG_I = 7;
	localparam SREG_T = 6;
	localparam SREG_H = 5;
	localparam SREG_S = 4;
	localparam SREG_V = 3;
	localparam SREG_N = 2;
	localparam SREG_Z = 1;
	localparam SREG_C = 0;

	// the PC output is almost always the actual PC,
	// although sometimes it is the address for a LPM
	assign pc = next_PC; // lpm_active ? addr : reg_PC;
	reg [15:0] next_PC;

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

	reg [7:0] R;
	reg [1:0] dest;
	localparam DEST_RD = 1;
	localparam DEST_RDI = 2;


	// ALU registers
	wire [5:0] alu_r = { opcode[9], opcode[3:0] };
	wire [5:0] alu_d = opcode[8:4];
	wire [5:0] alu_di = { 1'b1, opcode[7:4] };
	wire [7:0] alu_Rr = regs[alu_r];
	wire [7:0] alu_Rd = regs[alu_d];
	wire [7:0] alu_K = { opcode[11:8], opcode[3:0] };
	wire [7:0] alu_Rdi = regs[alu_di];

	// two operand
	wire [8:0] alu_SUB = alu_Rd - alu_Rr;
	wire [8:0] alu_SBC = alu_Rd - alu_Rr - sreg[SREG_C];
	wire [8:0] alu_ADD = alu_Rd + alu_Rr;
	wire [8:0] alu_ADC = alu_Rd + alu_Rr + sreg[SREG_C];
	wire [7:0] alu_EOR = alu_Rd ^ alu_Rr;
	wire [7:0] alu_OR  = alu_Rd | alu_Rr;
	wire [7:0] alu_AND = alu_Rd & alu_Rr;
	wire [7:0] alu_LSL = { alu_Rd[6:0], 1'b0 };
	wire [7:0] alu_ROL = { alu_Rd[6:0], sreg[SREG_C] };

	// helpers for computing sreg updates
	wire Rd3 = alu_Rd[3];
	wire Rr3 = alu_Rd[3];
	wire Rd7 = alu_Rd[7];
	wire Rr7 = alu_Rd[7];
	wire K3 = alu_K[3];
	wire K7 = alu_K[7];
	wire R3 = R[3];
	wire R7 = R[7];

	// register-immediate operand
	wire [7:0] alu_ANDI = alu_Rdi & alu_K;
	wire [7:0] alu_ORI  = alu_Rdi | alu_K;
	wire [8:0] alu_SUBI = alu_Rdi - alu_K;
	wire [8:0] alu_SBCI = alu_Rdi - alu_K - sreg[SREG_C];

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
		reg_PC <= next_PC;
		reg_SP <= next_SP;
		cycle <= next_cycle;
		skip <= next_skip;
		prev_opcode <= opcode;
		sreg <= next_sreg;

		addr <= next_addr;
		wen <= next_wen;
		ren <= next_ren;
		wdata <= next_wdata;

		case(dest)
		DEST_RD: begin
			$display("%04x: Rd %d = %02x", reg_PC*2, alu_d, R);
			regs[alu_d] <= R;
		end
		DEST_RDI: begin
			$display("%04x: Rdi %d = %02x", reg_PC*2, alu_di, R);
			regs[alu_di] <= R;
		end
		default: begin

		$display("%04x.%d %04x opcode %04x%s",
			reg_PC * 2,
			cycle,
			next_PC * 2,
			opcode,
			skip ? " SKIP" : ""
		);
		end
		endcase
	end

	always @(*)
	begin
		next_PC = reg_PC + 1;
		next_SP = reg_SP;
		next_sreg = sreg;
		next_cycle = 0;
		next_skip = 0;

		next_wdata = wdata;
		next_addr = addr;
		next_wen = 0;
		next_ren = 0;
	
		R = 0;
		dest = 0;

		casez(opcode)
		16'b0000000000000000: if (!skip) begin
			// NOP
		end
		16'b00000001_????_????: if (!skip) begin
			// MOVW Rd,Rr Move register pair
		end
		16'b0000_0011_0???_1???: if (!skip) begin
			// MUL and FMUL, unimplemented
		end

		// 2-operand instructions
		16'b000_?_01_?_?????_????: if (!skip) begin
			// CPC/CP Rd,Rr
		end
		16'b000010_?_?????_????: if (!skip) begin
			// SBC Rd,Rr
			dest = DEST_RD;
			R = alu_SBC;
		end
		16'b000110_?_?????_????: if (!skip) begin
			// SUB Rd,Rr
			dest = DEST_RD;
			R = alu_SUB;
		end
		16'b000_0_11_?_?????_????: if (!skip) begin
			// ADD Rd,Rr, LSL Rd when Rd=Rr
			dest = DEST_RD;
			if (alu_r == alu_d) begin
				R = alu_LSL;
				next_sreg[SREG_H] = Rd3;
				next_sreg[SREG_S] = R7 ^ (R7 ^ Rd7); // N^V
				next_sreg[SREG_V] = R7 ^ Rd7; // N^C
				next_sreg[SREG_N] = R7;
				next_sreg[SREG_Z] = R == 0;
				next_sreg[SREG_C] = Rd7;
			end else begin
				R = alu_ADD;
				next_sreg[SREG_H] = (Rd3 & Rr3) | (Rr3 & !R3) | (Rd3 & !R3);
				next_sreg[SREG_S] = R7 ^ (R7 ^ Rd7); // N^V
				next_sreg[SREG_V] = R7 ^ Rd7; // N^C
				next_sreg[SREG_N] = R7;
				next_sreg[SREG_Z] = R == 0;
				next_sreg[SREG_C] = Rd7 & Rr7 | Rr7 & !R7 | Rd7 & !R7;
			end
		end
		16'b000_1_11_?_?????_????: if (!skip) begin
			// ADC Rd,Rr, ROL Rd when Rd=Rr
			dest = DEST_RD;
			if (alu_r == alu_d) begin
				R = alu_ROL;
			end else begin
				R = alu_ADC;
			end
		end
		16'b000100_?_?????_????: if (!skip) begin
			// CPSE Rd,Rr
			if (alu_Rd == alu_Rr)
				next_skip = 1;
		end
		16'b001000_?_?????_????: if (!skip) begin
			// AND Rd,Rr
			dest = DEST_RD;
			R = alu_AND;
			next_sreg[SREG_S] = R7 ^ 0; // N^V
			next_sreg[SREG_V] = 0;
			next_sreg[SREG_N] = R7;
			next_sreg[SREG_Z] = R == 0;
		end
		16'b001000_?_?????_????: if (!skip) begin
			// EOR Rd,Rr
			dest = DEST_RD;
			R = alu_EOR;
		end
		16'b001010_?_?????_????: if (!skip) begin
			// OR Rd,Rr
			dest = DEST_RD;
			R = alu_OR;
		end
		16'b001011_?_?????_????: if (!skip) begin
			// MOV Rd,Rr
			dest = DEST_RD;
			R = alu_Rr;
		end

		// Register-immediate operations
		16'b0011_????_????_????: if (!skip) begin
			// CPI Rd,K
		end
		16'b0100_????_????_????: if (!skip) begin
			// SBCI Rd, K
			dest = DEST_RDI;
			R = alu_SBCI;
		end
		16'b0101_????_????_????: if (!skip) begin
			// SUBI Rd, K
			dest = DEST_RDI;
			R = alu_SUBI;
			next_sreg[SREG_H] = (K3 & !Rd3) | (K3 & R3) | (R3 & !Rd3);
			next_sreg[SREG_S] = next_sreg[SREG_N] ^ next_sreg[SREG_V];
			next_sreg[SREG_V] = (Rd7 & !K7 & !R7) | (K7 & R7 & !Rd7);
			next_sreg[SREG_N] = R7;
			next_sreg[SREG_Z] = R == 0;
			next_sreg[SREG_C] = (!Rd7 & K7) | (K7 & R7) | (R7 & !Rd7);
		end
		16'b0110_????_????_????: if (!skip) begin
			// ORI Rd,K or SBR Rd, K
			dest = DEST_RDI;
			R = alu_ORI;
		end
		16'b0111_????_????_????: if (!skip) begin
			// ANDI Rd,K or CBR Rd, K
			dest = DEST_RDI;
			R = alu_ANDI;
		end

		// Load-store instructions
		16'b100100_?_?????_0000: if (!skip) begin
			// LDS rd,i  / STS i,rd
			// followed by 16-bit immediate SRAM address
			if (cycle == 0) begin
				// wait for the next read to get the address
				next_cycle = 1;
			end else
			if (cycle == 1) begin
				next_addr = cdata;
				if (opcode[9] == 0) begin
					// LDS: request a read of that address and wait at this PC
					next_cycle = 2;
					next_PC = reg_PC;
					next_ren = 1;
				end else begin
					// STS: write to that address and go on
					next_wdata = regs[alu_d];
					next_wen = 1;
					next_cycle = 0;
				end
			end else begin
				// load the register from the data read from RAM
				dest = DEST_RD;
				R = data_read;
			end
		end else begin
			// need one extra cycle to read the extra word
			if (cycle == 0) begin
				next_cycle = 1;
				next_skip = 1;
			end else begin
				next_cycle = 0;
				next_skip = 0;
			end
		end
		16'b100100_?_?????_?_001: if (!skip) begin
			// LD/ST Rd through Z+/Y+
		end
		16'b100100_?_?????_?_010: if (!skip) begin
			// LD/ST Rd through -Z/-Y
		end
		16'b1001000_?????_01_?_0: if (!skip) begin
			// LPM/ELPM Rd,Z
		end
		16'b1001000_?????_01_?_1: if (!skip) begin
			// LPM/ELPM Rd,Z+
		end
		16'b1001001_?????_0100: if (!skip) begin
			// XCH Z,Rd
		end
		16'b1001001_?????_0101: if (!skip) begin
			// LAS Z,Rd
		end
		16'b1001001_?????_0110: if (!skip) begin
			// LAC Z,Rd
		end
		16'b1001001_?????_0111: if (!skip) begin
			// LAT Z,Rd
		end
		16'b1001001_?????_1100: if (!skip) begin
			// LD/ST Rd through X
		end
		16'b1001001_?????_1101: if (!skip) begin
			// LD/ST Rd through X+
		end
		16'b1001001_?????_1110: if (!skip) begin
			// LD/ST Rd through -X
		end
		16'b1001001_?????_1111: if (!skip) begin
			// POP/PUSH Rd
		end

		// One operand instructions
		16'b1001010_?????_0000: if (!skip) begin
			// COM Rd
		end
		16'b1001010_?????_0001: if (!skip) begin
			// NEG Rd
		end
		16'b1001010_?????_0010: if (!skip) begin
			// SWAP Rd
		end
		16'b1001010_?????_0011: if (!skip) begin
			// INC Rd
		end
		16'b1001010_?????_0100: if (!skip) begin
			// RESERVED
		end
		16'b1001010_?????_0101: if (!skip) begin
			// ASR Rd
		end
		16'b1001010_?????_0110: if (!skip) begin
			// LSR Rd
		end
		16'b1001010_?????_0111: if (!skip) begin
			// ROR Rd
		end
		16'b10010100_?_???_1000: if (!skip) begin
			// SEx/CLx Status register clear/set bit
		end

		// Zero-operand instructions
		16'b1001010100001000: if (!skip) begin
			// RET
			if (cycle == 0) begin
				next_cycle = 1;
				next_SP = reg_SP + 1;
				next_addr = next_SP;
				next_ren = 1;
				$display("%d read PC <= %04x", cycle, next_addr);
			end else
			if (cycle == 1) begin
				next_cycle = 2;
				next_PC[15:8] = data_read;
				next_SP = reg_SP + 1;
				next_addr = next_SP;
				next_ren = 1;
				$display("%d next PC high = %02x", cycle, data_read);
			end else begin
				next_cycle = 0;
				next_PC[7:0] = data_read;
				$display("%d next PC low = %02x", cycle, data_read);
			end
		end
		16'b1001010100011000: if (!skip) begin
			// RETI
		end
		16'b10010101001x1000: if (!skip) begin
			// RESERVED
		end
		16'b1001010101??1000: if (!skip) begin
			// RESERVED
		end
		16'b1001010110001000: if (!skip) begin
			// SLEEP
		end
		16'b1001010110011000: if (!skip) begin
			// BREAK
		end
		16'b1001010110101000: if (!skip) begin
			// WDR
		end
		16'b1001010110111000: if (!skip) begin
			// RESERVED
		end
		16'b10010101110_?_1000: if (!skip) begin
			// LPM/ELPM
		end
		16'b1001010111101000: if (!skip) begin
			// SPM
		end
		16'b1001010111111000: if (!skip) begin
			// SPM X+
		end

		// misc instructions
		16'b1001010_?_000_?_1001: if (!skip) begin
			// Indirect jump/call to Z or EIND:Z
		end
		16'b1001010_?????_1010: if (!skip) begin
			// DEC Rd
			dest = DEST_RD;
			R = alu_Rd - 1;
		end
		16'b10010100_????_1011: if (!skip) begin
			// DES round k
		end
		16'b1001010_?????_11_?_1: if (!skip) begin
			// JMP/CALL abs22
			// 16 bits in next word
			if (cycle == 0) begin
				next_cycle = 1;
				// write the first half of the return address
				next_addr = reg_SP;
				next_SP = reg_SP + 1;
				next_wdata = reg_PC + 2;
				next_wen = 1;
			end else begin
				next_PC = cdata;
				next_cycle = 0;
				// write the second half of the return address
				next_addr = reg_SP;
				next_SP = reg_SP + 1;
				next_wdata = reg_PC + 1;
				next_wen = 1;
			end
		end else begin
			// skip the read of the next word, do nothing
			if (cycle == 0) begin
				next_skip = 1;
				next_cycle = 1;
			end else begin
				next_cycle = 0;
			end
		end

		16'b10010110_??_??_????: if (!skip) begin
			// ADIW Rp, uimm6
		end
		16'b10010111_??_??_????: if (!skip) begin
			// SBIW Rp, uimm6
		end
		16'b100110_?_0_?????_???: if (!skip) begin
			// CBI/SBI a,b (clear/set IO bit)
		end
		16'b100110_?_1_?????_???: if (!skip) begin
			// SBIC/SBIS a,b (IO bit test)
		end
		16'b100111_?_?????_????: if (!skip) begin
			// MUL unsigned R1:R0 = Rr*Rd
		end
		16'b1011_?_??_?????_????: if (!skip) begin
			// IN/OUT to IO space
		end
		16'b1100_????????????: if (!skip) begin
			// RJMP to PC + simm12
			next_PC = reg_PC + simm12 + 1;
		end
		16'b1101_????????????: if (!skip) begin
			// RCALL to PC + simm12
			if (cycle == 0) begin
				// push the first half of the PC
				next_wen = 1;
				next_addr = reg_SP;
				next_SP = reg_SP - 1;
				next_wdata = next_PC[7:0]; // pc + 1

				// and cycle until the next half
				next_cycle = 1;
			end else begin
				// push the second half
				next_wen = 1;
				next_addr = reg_SP;
				next_SP = reg_SP - 1;
				next_wdata = reg_PC[15:8]; // pc is already plus 1

				// and do the jump
				next_PC = reg_PC + simm12; // pc is already plus 1
				next_cycle = 0;
			end
		end
		16'b1110_????_????_????: if (!skip) begin
			// LDI Rd, K
			dest = DEST_RDI;
			R = alu_K;
		end
		16'b111100_???????_???: if (!skip) begin
			// BRBS - Branch if bit in SREG is set
			if (sreg[opcode[2:0]])
				next_PC = reg_PC + simm7 + 1;
		end
		16'b111101_???????_???: if (!skip) begin
			// BRBC - Branch if bit in SREG is clear
			if (!sreg[opcode[2:0]])
				next_PC = reg_PC + simm7 + 1;
		end
		16'b111110_?_?????_0_???: if (!skip) begin
			// BLD/BST register bit to STATUS.T
		end
		16'b111111_?_?????_0_???: if (!skip) begin
			// SBRC/SBRS skip if register bit b equals B
		end
		16'b11111_??_?????_1_???: if (!skip) begin
			// RESERVED
		end
		default: begin
		end
		endcase
	end
endmodule

`endif
