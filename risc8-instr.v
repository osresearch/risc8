`ifndef _risc8_instruction_v_
`define _risc8_instruction_v_
/*
 * Decode the RISC-8 instruction opcodes into macro instructions
 * and src/dest registers.
 *
 * The single cycle instructions are grouped first.
 */

`define is_invalid	6'h00

// Single cycle arithmetic operators
`define is_cpc		6'h01
`define is_cp		6'h02
`define is_sbc		6'h03
`define is_sub		6'h04
`define is_add		6'h05
`define is_adc		6'h06
`define is_and		6'h07
`define is_eor		6'h08
`define is_or		6'h09
`define is_mov		6'h0a
`define is_subi		6'h0b
`define is_sbci		6'h0c
`define is_cpi		6'h0d
`define is_ori		6'h0e
`define is_andi		6'h0f
`define is_com		6'h10
`define is_neg		6'h11
`define is_swap		6'h12
`define is_inc		6'h13
`define is_asr		6'h14
`define is_lsr		6'h15
`define is_ror		6'h16
`define is_dec		6'h17
`define is_adiw_or_sbiw	6'h18
`define is_movw		6'h19
`define is_ldi		6'h1a
`define is_clx_or_sex	6'h1b
`define is_mulu		6'h1c
`define is_nop		6'h1f

// Memory instructions
`define is_out		6'h20
`define is_in		6'h21
`define is_lds		6'h22
`define is_ld_xyz	6'h23
`define is_ld_yz_plus_q	6'h24
`define is_lpm		6'h25
`define is_push		6'h26
`define is_pop		6'h27

// Control flow instructions
`define is_ret		6'h30
`define is_cpse		6'h31
`define is_sbrc_or_sbrs	6'h32
`define is_brbc_or_brbs	6'h33
`define is_jmp		6'h34
`define is_call		6'h35
`define is_ijmp		6'h36
`define is_rjmp		6'h37
`define is_rcall	6'h38
`define is_sbis_or_sbic	6'h39

module risc8_instruction(
	input [15:0] opcode,
	output [5:0] instr,
	output [5:0] Rd,
	output [5:0] Rr
);

	/* Instruction decoding */
	reg [5:0] instr;
	wire [5:0] op_Rd = opcode[8:4]; // 0-31

	/*
	 * Match instructions on every bit except for the
	 * five Rd bits (opcode[8:4]), which are wildcard
	 * for almost every instruction.
	 */
	always @(*) begin
		instr = `is_invalid; 

		casez({opcode[15:9],opcode[3:0]})
		11'b0000_000_0000: if (op_Rd == 5'b0000) instr = `is_nop;
		11'b0000_000_????: if (opcode[8] == 1'b1) instr = `is_movw;
		11'b0000_01?_????: instr = `is_cpc;
		11'b0000_10?_????: instr = `is_sbc;
		11'b0000_11?_????: instr = `is_add; // also LSL
		11'b0001_00?_????: instr = `is_cpse;
		11'b0001_01?_????: instr = `is_cp;
		11'b0001_10?_????: instr = `is_sub;
		11'b0001_11?_????: instr = `is_adc; // also ROL
		11'b0010_00?_????: instr = `is_and;
		11'b0010_01?_????: instr = `is_eor;
		11'b0010_10?_????: instr = `is_or;
		11'b0010_11?_????: instr = `is_mov;
		11'b0011_???_????: instr = `is_cpi;
		11'b0100_???_????: instr = `is_sbci;
		11'b0101_???_????: instr = `is_subi;
		11'b0110_???_????: instr = `is_ori;
		11'b0111_???_????: instr = `is_andi;
		11'b1001_00?_0000: instr = `is_lds;
		11'b1001_000_010?: instr = `is_lpm; // Z
		11'b1000_00?_0000: instr = `is_ld_xyz; // z
		11'b1000_00?_1000: instr = `is_ld_xyz; // Y
		11'b1000_00?_1100: instr = `is_ld_xyz; // X
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
		11'b1001_010_0001: instr = `is_neg;
		11'b1001_010_0010: instr = `is_swap;
		11'b1001_010_0011: instr = `is_inc;
		//11'b1001_010?_0100: instr = `is_nop; // reserved
		11'b1001_010_0101: instr = `is_asr;
		11'b1001_010_0110: instr = `is_lsr;
		11'b1001_010_0111: instr = `is_ror;
		11'b1001_010_1000: begin
			casez(opcode[8:4])
			5'b0????: instr = `is_clx_or_sex;
			5'b10000: instr = `is_ret;
			5'b11100: instr = `is_lpm;
			endcase
		end
		11'b1001_010_1001: instr = `is_ijmp;
		11'b1001_010_1010: instr = `is_dec;
		11'b1001_010_110?: instr = `is_jmp;
		11'b1001_010_1111: instr = `is_call;
		11'b1001_011_????: instr = `is_adiw_or_sbiw;
		//12'b1001_11??_????: instr = `is_mulu; // need to infer multiply
		11'b1001_10?_????: instr = `is_sbis_or_sbic;
		11'b1011_0??_????: instr = `is_in;
		11'b1011_1??_????: instr = `is_out;
		11'b1100_???_????: instr = `is_rjmp;
		11'b1101_???_????: instr = `is_rcall;
		11'b1110_???_????: instr = `is_ldi; // also SER, with all 1
		11'b1111_0??_????: instr = `is_brbc_or_brbs;
		11'b1111_11?_0???: instr = `is_sbrc_or_sbrs;
		endcase
	end
endmodule

`endif

