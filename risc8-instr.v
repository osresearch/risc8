`ifndef _risc8_instruction_v_
`define _risc8_instruction_v_
/*
 * Decode the RISC-8 instruction opcodes into macro instructions
 * and src/dest registers.
 *
 * The single cycle instructions are grouped first.
 */

// Single cycle arithmetic operators mostly go straight to the alu
// op Rd, Rr
`define is_alu		5'h00

// although some require special handling
`define is_inc		5'h01
`define is_dec		5'h02
`define is_com		5'h03
`define is_adiw_or_sbiw	5'h04
`define is_movw		5'h05
`define is_clx_or_sex	5'h06
`define is_mulu		5'h07

// Memory instructions
`define is_out		5'h08
`define is_in		5'h09
`define is_lds		5'h0a
`define is_ld_xyz	5'h0b
`define is_ld_yz_plus_q	5'h0c
`define is_lpm		5'h0d
`define is_pop		5'h0e
`define is_push		5'h0f

// Control flow instructions
`define is_ret		5'h10
`define is_cpse		5'h11
`define is_sbrc_or_sbrs	5'h12
`define is_brbc_or_brbs	5'h13
`define is_jmp		5'h14
`define is_call		5'h15
`define is_ijmp		5'h16
`define is_rjmp		5'h17
`define is_rcall	5'h18
`define is_sbis_or_sbic	5'h19

module risc8_instruction(
	input [15:0] opcode,
	output [4:0] instr,
	output [3:0] alu_op,
	output alu_rdi,
	output alu_store,
	output alu_carry
);
	// Register in the opcode
	wire [5:0] op_Rd = opcode[8:4]; // 0-31

	/* Instruction decoding */
	reg [4:0] instr;
	reg [3:0] alu_op;
	reg alu_store;
	reg alu_carry;
	reg alu_rdi;

`define ALU_OP(op, store, carry) \
	begin \
		alu_op = op; \
		alu_store = store; \
		alu_carry = carry; \
	end

`define ALU_OP_RDI(op, store, carry) \
	begin \
		alu_op = op; \
		alu_store = store; \
		alu_carry = carry; \
		alu_rdi = 1; \
	end

	/*
	 * Match instructions on every bit except for the
	 * five Rd bits (opcode[8:4]), which are wildcard
	 * for almost every instruction.
	 */
	always @(*) begin
		instr = `is_alu;
		alu_op = 0;
		alu_store = 0;
		alu_carry = 0;
		alu_rdi = 0;

		casez({opcode[15:9],opcode[3:0]})
		11'b0000_000_????: if (opcode[8] == 1'b1) instr = `is_movw; // else NOP
		11'b0000_01?_????: `ALU_OP(`OP_SUB, 0, 1) // CPC Rd,Rr
		11'b0000_10?_????: `ALU_OP(`OP_SUB, 1, 1) // SBC Rd, Rr
		11'b0000_11?_????: `ALU_OP(`OP_ADD, 1, 0) // ADD Rd, Rd
		11'b0001_00?_????: instr = `is_cpse;
		11'b0001_01?_????: `ALU_OP(`OP_SUB, 0, 0) // CP Rd,Rr
		11'b0001_10?_????: `ALU_OP(`OP_SUB, 1, 0) // SUB Rd, Rr
		11'b0001_11?_????: `ALU_OP(`OP_ADD, 1, 1) // ADC Rd, Rr
		11'b0010_00?_????: `ALU_OP(`OP_AND, 1, 0) // AND Rd, Rr
		11'b0010_01?_????: `ALU_OP(`OP_EOR, 1, 0) // EOR Rd, Rr
		11'b0010_10?_????: `ALU_OP(`OP_OR,  1, 0) // OR Rd, Rr
		11'b0010_11?_????: `ALU_OP(`OP_MOVR, 1, 0) // MOV Rd, Rr
		11'b0011_???_????: `ALU_OP_RDI(`OP_SUB, 0, 0) // CPI Rdi, K
		11'b0100_???_????: `ALU_OP_RDI(`OP_SUB, 1, 1) // SBCI Rdi, K
		11'b0101_???_????: `ALU_OP_RDI(`OP_SUB, 1, 0) // SUBI Rdi, K
		11'b0110_???_????: `ALU_OP_RDI(`OP_OR, 1, 0) // ORI Rdi, K
		11'b0111_???_????: `ALU_OP_RDI(`OP_AND, 1, 0) // ANDI Rdi, K
		11'b1001_00?_0000: instr = `is_lds;
		11'b1001_000_010?: instr = `is_lpm; // Z
		11'b1000_00?_0000: instr = `is_ld_xyz; // z
		11'b1000_00?_1000: instr = `is_ld_xyz; // Y
		11'b1001_00?_1100: instr = `is_ld_xyz; // X
		11'b1001_00?_0001: instr = `is_ld_xyz; // Z+
		11'b1001_00?_0010: instr = `is_ld_xyz; // -Z
		11'b1001_00?_1001: instr = `is_ld_xyz; // Y+
		11'b1001_00?_1010: instr = `is_ld_xyz; // -Y
		11'b1001_00?_1101: instr = `is_ld_xyz; // X+
		11'b1001_00?_1110: instr = `is_ld_xyz; // -X
		11'b10?0_???_????: instr = `is_ld_yz_plus_q;
		11'b1001_000_1111: instr = `is_pop;
		11'b1001_001_1111: instr = `is_push;
		11'b1001_010_0000: instr = `is_com;
		11'b1001_010_0001: `ALU_OP(`OP_NEG, 1, 0) // NEG Rd
		11'b1001_010_0010: `ALU_OP(`OP_SWAP, 1, 0) // SWAP Rd
		11'b1001_010_0011: instr = `is_inc; // INC Rd
		//11'b1001_010?_0100: instr = `is_nop; // reserved
		11'b1001_010_0101: `ALU_OP(`OP_ASR, 1, 0) // ASR Rd
		11'b1001_010_0110: `ALU_OP(`OP_LSR, 1, 0) // LSR Rd
		11'b1001_010_0111: `ALU_OP(`OP_ROR, 1, 0) // ROR Rd
		11'b1001_010_1000: begin
			casez(opcode[8:4])
			5'b0????: instr = `is_clx_or_sex;
			5'b10000: instr = `is_ret;
			5'b11100: instr = `is_lpm;
			endcase
		end
		11'b1001_010_1001: instr = `is_ijmp;
		11'b1001_010_1010: instr = `is_dec; // DEC Rd
		11'b1001_010_110?: instr = `is_jmp;
		11'b1001_010_1111: instr = `is_call;
		11'b1001_011_????: instr = `is_adiw_or_sbiw;
		11'b1001_11?_????: instr = `is_mulu;
		11'b1001_10?_????: instr = `is_sbis_or_sbic;
		11'b1011_0??_????: instr = `is_in;
		11'b1011_1??_????: instr = `is_out;
		11'b1100_???_????: instr = `is_rjmp;
		11'b1101_???_????: instr = `is_rcall;
		11'b1110_???_????: `ALU_OP_RDI(`OP_MOVR, 1, 0) // LDI Rdi, K also SER, with all 1
		11'b1111_0??_????: instr = `is_brbc_or_brbs;
		11'b1111_11?_0???: instr = `is_sbrc_or_sbrs;
		endcase
	end
endmodule

`endif
