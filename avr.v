`ifndef _avr_cpu_v_
`define _avr_cpu_v_

module avr_cpu(
	input clk,
	input reset,

	// the program memory should provide a new opcode
	// every clock cycle
	output [15:0] pc,
	input [15:0] cdata,

	// the data memory is used for LD/ST
	output [15:0] addr,
	output wen,
	input [7:0] rdata,
	output [7:0] wdata
);
	// register file (as flops, not as BRAM)
	reg [7:0] regs[31:0];
	wire [15:0] reg_X = { regs[25], regs[24] };
	wire [15:0] reg_Y = { regs[29], regs[28] };
	wire [15:0] reg_Z = { regs[31], regs[30] };
	reg [15:0] reg_PC;
	reg [15:0] reg_SP;
	wire [7:0] sreg = { sreg_I, sreg_T, sreg_H, sreg_S, sreg_V, sreg_N, sreg_Z, sreg_C };
	reg sreg_I = 0;
	reg sreg_T = 0;
	reg sreg_H = 0;
	reg sreg_S = 0;
	reg sreg_V = 0;
	reg sreg_N = 0;
	reg sreg_Z = 0;
	reg sreg_C = 0;

	// the PC output is almost always the actual PC,
	// although sometimes it is the address for a LPM
	assign pc = reg_PC; // lpm_active ? addr : reg_PC;
	wire [15:0] next_PC = reg_PC + 1;

	// Some instructions require an extra cycle;
	// they will set cycle and re-use the previous opcode
	reg [1:0] cycle = 0;
	// Some instruction can cause the next instruction to be skipped,
	// which might be multiple words; this still executes the instruction,
	// but doesn't write any results
	reg skip = 0;
	reg [15:0] prev_opcode;
	wire [15:0] opcode = cycle == 0 ? cdata : prev_opcode;
	reg [15:0] addr;
	reg [7:0] wdata;
	reg wen;


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
	wire [8:0] alu_SBC = alu_Rd - alu_Rr - sreg_C;
	wire [8:0] alu_ADD = alu_Rd + alu_Rr;
	wire [8:0] alu_ADC = alu_Rd + alu_Rr + sreg_C;
	wire [7:0] alu_EOR = alu_Rd ^ alu_Rr;
	wire [7:0] alu_OR  = alu_Rd | alu_Rr;
	wire [7:0] alu_AND = alu_Rd & alu_Rr;
	wire [7:0] alu_LSL = { alu_Rd[6:0], 1'b0 };
	wire [7:0] alu_ROL = { alu_Rd[6:0], sreg_C };

	// helpers for computing sreg updates
	wire Rd3 = alu_Rd[3];
	wire Rr3 = alu_Rd[3];
	wire Rd7 = alu_Rd[7];
	wire Rr7 = alu_Rd[7];
	wire K3 = alu_K[3];
	wire K7 = alu_K[7];

	// register-immediate operand
	wire [7:0] alu_ANDI = alu_Rdi & alu_K;
	wire [7:0] alu_ORI  = alu_Rdi | alu_K;
	wire [8:0] alu_SUBI = alu_Rdi - alu_K;
	wire [8:0] alu_SBCI = alu_Rdi - alu_K - sreg_C;

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
		reg_PC <= 0;
		reg_SP <= 0;

		sreg_I <= 0;
		sreg_T <= 0;
		sreg_H <= 0;
		sreg_S <= 0;
		sreg_V <= 0;
		sreg_N <= 0;
		sreg_Z <= 0;
		sreg_C <= 0;

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
		// default is PC advances by one word
		reg_PC <= next_PC;
		cycle <= 0;
		skip <= 0;
		wen <= 0;
		prev_opcode <= opcode;
		$display("%04x.%d opcode %04x%s",
			reg_PC,
			cycle,
			opcode,
			skip ? " SKIP" : ""
		);

		casez(cycle == 0 ? opcode : prev_opcode)
		16'b0000000000000000: if (!skip) begin
			// NOP
			$display("NOP");
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
			regs[alu_d] <= alu_SBC;
		end
		16'b000110_?_?????_????: if (!skip) begin
			// SUB Rd,Rr
			regs[alu_d] <= alu_SUB;
		end
		16'b000_0_11_?_?????_????: if (!skip) begin
			// ADD Rd,Rr, LSL Rd when Rd=Rr
			if (alu_r == alu_d) begin
				regs[alu_d] <= alu_LSL;
				sreg_H <= Rd3;
				sreg_S <= alu_LSL[7] ^ (alu_LSL[7] ^ Rd7); // N^V
				sreg_V <= alu_LSL[7] ^ Rd7; // N^C
				sreg_N <= alu_LSL[7];
				sreg_Z <= alu_LSL == 0;
				sreg_C <= Rd7;
			end else begin
				$display("ADD %02x = %02x + %02x", alu_ADD, alu_Rd, alu_Rr);
				regs[alu_d] <= alu_ADD;
				sreg_H <= (Rd3 & Rr3) | (Rr3 & !alu_ADD[3]) | (Rd3 & !alu_ADD[3]);
				sreg_S <= alu_LSL[7] ^ (alu_LSL[7] ^ Rd7); // N^V
				sreg_V <= alu_LSL[7] ^ Rd7; // N^C
				sreg_N <= alu_ADD[7];
				sreg_Z <= alu_ADD == 0;
				sreg_C <= Rd7 & Rr7 | Rr7 & !alu_ADD[7] | Rd7 & !alu_ADD[7];
			end
		end
		16'b000_1_11_?_?????_????: if (!skip) begin
			// ADC Rd,Rr, ROL Rd when Rd=Rr
			if (alu_r == alu_d) begin
				regs[alu_d] <= alu_ROL;
			end else begin
				regs[alu_d] <= alu_ADC;
			end
		end
		16'b000100_?_?????_????: if (!skip) begin
			// CPSE Rd,Rr
			$display("CPSE %02x == %02x", alu_Rd, alu_Rr);
			if (alu_Rd == alu_Rr)
				skip <= 1;
		end
		16'b001000_?_?????_????: if (!skip) begin
			// AND Rd,Rr
			regs[alu_d] <= alu_AND;
			sreg_S <= alu_AND[7] ^ 0; // N^V
			sreg_V <= 0;
			sreg_N <= alu_AND[7];
			sreg_Z <= alu_AND == 0;
		end
		16'b001000_?_?????_????: if (!skip) begin
			// EOR Rd,Rr
			regs[alu_d] <= alu_EOR;
		end
		16'b001010_?_?????_????: if (!skip) begin
			// OR Rd,Rr
			regs[alu_d] <= alu_OR;
		end
		16'b001011_?_?????_????: if (!skip) begin
			// MOV Rd,Rr
			regs[alu_d] <= alu_Rr;
		end

		// Register-immediate operations
		16'b0011_????_????_????: if (!skip) begin
			// CPI Rd,K
		end
		16'b0100_????_????_????: if (!skip) begin
			// SBCI Rd, K
			$display("SBCI");
			regs[alu_di] <= alu_SBCI;
		end
		16'b0101_????_????_????: if (!skip) begin
			// SUBI Rd, K
			$display("SUBI");
			regs[alu_di] <= alu_SUBI;
			sreg_H <= (K3 & !Rd3) | (K3 & alu_SUBI[3]) | (alu_SUBI[3] & !Rd3);
			sreg_S <= ((Rd7 & !K7 & !alu_SUBI[7]) | (K7 & alu_SUBI[7] & !Rd7)) ^ alu_SUBI[7]; // N^V
			sreg_V <= (Rd7 & !K7 & !alu_SUBI[7]) | (K7 & alu_SUBI[7] & !Rd7);
			sreg_N <= alu_SUBI[7];
			sreg_Z <= alu_SUBI == 0;
			sreg_C <= (!Rd7 & K7) | (K7 & alu_SUBI[7]) | (alu_SUBI[7] & !Rd7);
		end
		16'b0110_????_????_????: if (!skip) begin
			// ORI Rd,K or SBR Rd, K
			regs[alu_di] <= alu_ORI;
		end
		16'b0111_????_????_????: if (!skip) begin
			// ANDI Rd,K or CBR Rd, K
			regs[alu_di] <= alu_ANDI;
		end

		// Load-store instructions
		16'b100100_?_?????_0000: if (!skip) begin
			// LDS rd,i  / STS i,rd
			// followed by 16-bit immediate SRAM address
			if (cycle == 0) begin
				// wait for the next read to get the address
				cycle <= 1;
			end else
			if (cycle == 1) begin
				addr <= cdata;
				if (opcode[9] == 0) begin
					// LDS: request a read of that address and wait at this PC
					cycle <= 2;
					reg_PC <= reg_PC;
				end else begin
					// STS: write to that address and go on
					wdata <= regs[alu_d];
					wen <= 1;
					cycle <= 0;
				end
			end else begin
				// load the register from the data read from RAM
				regs[alu_d] <= rdata;
			end
		end else begin
			// need one extra cycle to read the extra word
			if (cycle == 0) begin
				cycle <= 1;
				skip <= 1;
			end else begin
				cycle <= 0;
				skip <= 0;
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
				$display("RET pop %04x...", reg_SP + 1);
				cycle <= 1;
				addr <= reg_SP + 1;
				reg_SP <= reg_SP + 1;
			end else
			if (cycle == 1) begin
				$display("RET %02x pop %04x... ", rdata, reg_SP + 1);
				reg_PC[15:8] <= rdata;
				addr <= reg_SP + 1;
				reg_SP <= reg_SP + 1;
				cycle <= 2;
			end else begin
				$display("RET %02x => %04x", rdata, reg_PC);
				reg_PC[7:0] <= rdata;
				cycle <= 0;
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
			regs[alu_di] <= alu_Rd - 1;
		end
		16'b10010100_????_1011: if (!skip) begin
			// DES round k
		end
		16'b1001010_?????_11_?_1: if (!skip) begin
			// JMP/CALL abs22
			// 16 bits in next word
			if (cycle == 0) begin
				cycle <= 1;
				// write the first half of the return address
				addr <= reg_SP;
				reg_SP <= reg_SP + 1;
				wdata <= reg_PC + 2;
				wen <= 1;
			end else begin
				reg_PC <= opcode;
				cycle <= 0;
				// write the second half of the return address
				addr <= reg_SP;
				reg_SP <= reg_SP + 1;
				wdata <= reg_PC + 1;
				wen <= 1;
			end
		end else begin
			// skip the read of the next word, do nothing
			if (cycle == 0) begin
				skip <= 1;
				cycle <= 1;
			end else begin
				cycle <= 0;
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
			$display("RJMP %04x + %04x", reg_PC, simm12);
			reg_PC <= reg_PC + simm12 + 1;
		end
		16'b1101_????????????: if (!skip) begin
			// RCALL to PC + simm12
			if (cycle == 0) begin
				// push the first half of the PC
				$display("RCALL push %02x %04x", next_PC[7:0], reg_SP);
				wen <= 1;
				addr <= reg_SP;
				reg_SP <= reg_SP - 1;
				wdata <= next_PC[7:0]; // pc + 1

				// and cycle until the next half
				cycle <= 1;
			end else begin
				// push the second half
				wen <= 1;
				addr <= reg_SP;
				reg_SP <= reg_SP - 1;
				wdata <= reg_PC[15:8]; // pc is already plus 1

				// and do the jump
				$display("RCALL %04x + %04x (%04x)", reg_PC, simm12, opcode);
				reg_PC <= reg_PC + simm12; // pc is already plus 1
				cycle <= 0;
			end
		end
		16'b1110_????_????_????: if (!skip) begin
			// LDI Rd, K
			$display("LDI");
			regs[alu_di] <= alu_K;
		end
		16'b111100_???????_???: if (!skip) begin
			// BRBS - Branch if bit in SREG is set
			if (sreg[opcode[2:0]])
				reg_PC <= reg_PC + simm7 + 1;
		end
		16'b111101_???????_???: if (!skip) begin
			// BRBC - Branch if bit in SREG is clear
			if (!sreg[opcode[2:0]])
				reg_PC <= reg_PC + simm7 + 1;
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
			$display("UNKNOWN: %04x", opcode);
		end
		endcase
	end
endmodule

`endif
