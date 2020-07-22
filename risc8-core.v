/*
 * RISC-8, a mostly AVR comptaible softcore.
 *
 * Two stage pipeline design, similar to the actual ATtiny85 architecture.
 * Each clock can retire one instruction.
 *
 * First stage:
 * - Receive opcode on input
 * - Decode into registers required
 * - Setup reads of those registers
 * - Latch constant values and flags for next stage.
 * - Latch destination register
 *
 * Second stage:
 * - Route registers values or constant values to ALU
 * - Setup write of ALU output
 * 
 */
`ifndef _risc8_core_v_
`define _risc8_core_v_
`include "risc8-alu.v"
`include "regfile.v"


module risc8_core(
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

	// The register file has a block ram underneath with a one-cycle
	// delay to reads. This means that there is a two stage pipeline
	// of decoding the existing instruction on one clock and loading
	// the register, and then one clock to evaluate and retire it.
	reg [5:0] sel_Ra;
	reg [5:0] sel_Rb;
	reg [5:0] sel_Rd;
	wire [15:0] reg_Ra;
	wire [7:0] reg_Rb;


	regfile regs(
		.clk(clk),
		.reset(reset),
		// Read ports. Rd is 8 or 16 bits, Rr is always 8
		.a(sel_Ra),
		.b(sel_Rb),
		.Ra(reg_Ra),
		.Rb(reg_Rb),
		// write port, 8 or 16 bits, delayed by a clock for the ALU
		.d(prev_sel_Rd),
		.Rd(alu_out),
		.write(prev_alu_store),
		.write_word(prev_alu_word)
	);

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
	reg alu_store;
	reg alu_word;
	reg alu_carry;

	// delayed by one cycle for the register file to finish loading
	reg prev_alu_store;
	reg prev_alu_word;
	reg prev_alu_carry;
	reg [5:0] prev_sel_Rd;

	// opcode registers
	wire [5:0] op_Rr = { opcode[9], opcode[3:0] }; // 0-31
	wire [5:0] op_Rd = opcode[8:4]; // 0-31
	wire [5:0] op_Rdi = { 1'b1, opcode[7:4] }; // 16-31
 	wire [5:0] op_Rp = { opcode[5:4], 3'b000 };
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
	reg [3:0] alu_op;
	reg [3:0] prev_alu_op;
	wire [15:0] alu_out;
	reg [7:0] alu_const_value;
	reg [7:0] prev_alu_const_value;
	reg alu_const;
	reg prev_alu_const;
	reg [7:0] next_sreg;
	wire [7:0] sreg_out;

	wire [15:0] alu_Rd = reg_Ra;
	wire [ 7:0] alu_Rr = prev_alu_const ? prev_alu_const_value : reg_Rb; // sometimes a constant value

	risc8_alu core_alu(
		.clk(clk),
		.reset(reset),
		.op(prev_alu_op),
		.use_carry(prev_alu_carry),
		.Rd_in(alu_Rd),
		.Rr_in(alu_Rr),
		.R_out(alu_out),
		.sreg_in(sreg),
		.sreg_out(sreg_out)
	);

	always @(posedge clk) if (reset) begin
		cycle <= 0;
		skip <= 0;
		reg_PC <= 0;
		reg_SP <= 16'h1000;
		sreg <= 0;
		addr <= 0;
		wen <= 0;
		ren <= 0;
		wdata <= 0;
		prev_alu_store <= 0;

	end else begin
		if (cycle == 0)
		$display("%04x: %04x %02x A[%d]=%04x B[%d]=%02x, %04x %x %02x %b = %04x => %d%s%s",
			reg_PC * 16'h2,
			opcode,
			sreg,
			sel_Ra, reg_Ra,
			sel_Rb, reg_Rb,
			alu_Rd,
			prev_alu_op,
			alu_Rr,
			prev_alu_carry,
			alu_out,
			prev_sel_Rd,
			prev_alu_store ? " WRITE" : "",
			skip ? " SKIP" : ""
		);

		// only advance the PC if we are not in
		// a multi-cycle instruction and not a LPM
		if (force_PC || next_cycle == 0)
			reg_PC <= next_PC;

		reg_SP <= next_SP;
		sreg <= next_sreg;
		temp <= next_temp;
		cycle <= next_cycle;
		skip <=  next_skip;
		prev_opcode <= opcode;

		addr <= next_addr;
		wen <= next_wen;
		ren <= next_ren;
		wdata <= next_wdata;

		// Since the register file takes a cycle to
		// read, update the actual destination
		// to write into the register file on the
		// following cycle, after the ALU has
		// finished the operation.
		prev_sel_Rd <= sel_Rd;
		prev_alu_op <= alu_op;
		prev_alu_store <= alu_store;
		prev_alu_carry <= alu_carry;
		prev_alu_const <= alu_const;
		prev_alu_const_value <= alu_const_value;
		prev_alu_word <= alu_word;
	end

	/* Instruction decoding */
	reg is_nop, is_movw, is_cpc, is_sbc, is_add, is_cpse, is_cp;
	reg is_sub, is_adc, is_and, is_eor, is_or, is_mov, is_ld_xyz;
	reg is_lds, is_brbc_or_brbs, is_sbrc_or_sbrs, is_in, is_out;
	reg is_cpi, is_sbci, is_subi, is_ori, is_andi, is_rjmp, is_rcall;
	reg is_ldi, is_lpm, is_push, is_pop, is_com, is_neg, is_swap;
	reg is_inc, is_asr, is_lsr, is_ror, is_ijmp, is_dec, is_jmp, is_call;
	reg is_adiw_or_sbiw, is_ld_yz_plus_q, is_ret, is_clx_or_sex;

	always @(*) begin
		is_nop = 0; is_movw = 0; is_cpc = 0; is_sbc = 0; is_add = 0;
		is_cpse = 0; is_cp = 0; is_sub = 0; is_adc = 0; is_and = 0;
		is_eor = 0; is_or = 0; is_mov = 0; is_ld_xyz = 0; is_lds = 0;
		is_brbc_or_brbs = 0; is_sbrc_or_sbrs = 0; is_in = 0;
		is_out = 0; is_cpi = 0; is_sbci = 0; is_subi = 0; is_ori = 0;
		is_andi = 0; is_rjmp = 0; is_rcall = 0; is_ldi = 0; is_lpm = 0;
		is_push = 0; is_pop = 0; is_com = 0; is_neg = 0; is_swap = 0;
		is_inc = 0; is_asr = 0; is_lsr = 0; is_ror = 0; is_ijmp = 0;
		is_dec = 0; is_jmp = 0; is_jmp = 0; is_call = 0; is_ret = 0;
		is_adiw_or_sbiw = 0; is_ld_yz_plus_q = 0; is_clx_or_sex = 0;

		case(opcode[15:0])
		16'b0000000000000000: is_nop = 1;
		16'b1001010100001000: is_ret = 1;
		16'b1001010111001000: is_lpm = 1;
		endcase

		case(opcode[15:10])
		6'b0000_01: is_cpc = 1;
		6'b0000_10: is_sbc = 1;
		6'b0000_11: is_add = 1; // also LSL
		6'b0001_00: is_cpse = 1;
		6'b0001_01: is_cp = 1;
		6'b0001_10: is_sub = 1;
		6'b0001_11: is_adc = 1; // also ROL
		6'b0010_00: is_and = 1;
		6'b0010_01: is_eor = 1;
		6'b0010_10: is_or = 1;
		6'b0010_11: is_mov = 1;
		6'b1000_00: begin
			case(opcode[3:0])
			4'b0000: is_ld_xyz = 1; // z
			4'b1000: is_ld_xyz = 1; // Y
			4'b1100: is_ld_xyz = 1; // X
			endcase
		end
		6'b1001_00: begin
			case(opcode[3:0])
			4'b0000: is_lds = 1;
			4'b0001: is_ld_xyz = 1; // Z+
			4'b0010: is_ld_xyz = 1; // -Z
			4'b1001: is_ld_xyz = 1; // Y+
			4'b1010: is_ld_xyz = 1; // -Y
			4'b1101: is_ld_xyz = 1; // X+
			4'b1110: is_ld_xyz = 1; // -X
			endcase
		end
		6'b1111_00: is_brbc_or_brbs = 1;
		6'b1111_01: is_brbc_or_brbs = 1;
		6'b1111_11: if (opcode[3] == 0) is_sbrc_or_sbrs = 1;
		endcase

		case(opcode[15:11])
		5'b1011_0: is_in = 1;
		5'b1011_1: is_out = 1;
		endcase

		case(opcode[15:12])
		4'b0011: is_cpi = 1;
		4'b0100: is_sbci = 1;
		4'b0101: is_subi = 1;
		4'b0110: is_ori = 1;
		4'b0111: is_andi = 1;
		4'b1100: is_rjmp = 1;
		4'b1101: is_rcall = 1;
		4'b1110: is_ldi = 1; // also SER, with all 1
		endcase

		case(opcode[15:9])
		7'b0000_000: if (opcode[8]) is_movw = 1;
		7'b1001_000: begin
			case(opcode[3:0])
			4'b0100: is_lpm = 1; // Z
			4'b0101: is_lpm = 1; // Z+
			endcase
		end
		7'b1001_001: begin
			case(opcode[3:0])
			4'b1111: is_push = 1;
			4'b1111: is_pop = 1;
			endcase
		end
		7'b1001_010: begin
			case(opcode[3:0])
			4'b0000: is_com = 1;
			4'b0001: is_neg = 1;
			4'b0010: is_swap = 1;
			4'b0011: is_inc = 1;
			// 4'b0100: is_nop = 1; // reserved
			4'b0101: is_asr = 1;
			4'b0110: is_lsr = 1;
			4'b0111: is_ror = 1;
			4'b1001: is_ijmp = 1;
			4'b1010: is_dec = 1;
			4'b1100: is_jmp = 1;
			4'b1101: is_jmp = 1;
			4'b1110: is_call = 1;
			4'b1111: is_call = 1;
			endcase
		end
		7'b1001_011: is_adiw_or_sbiw = 1;
		endcase

		if (opcode[15:14] == 2'b10
		&&  opcode[   12] == 1'b0)
			is_ld_yz_plus_q = 1;

		if (opcode[15:8] == 8'b1001_0100
		&&  opcode[ 3:0] == 4'b1000)
			is_clx_or_sex = 1;
	end

	/*******************************/

	always @(*) begin

		// start pre-fetching the next PC
		if (reset)
			next_PC = 0;
		else
			next_PC = reg_PC + 1;

		// most instructions are single cycle, no writes, no reads
		next_sreg = sreg_out;
		next_cycle = 0;
		next_skip = 0;
		next_ren = 0;
		next_wen = 0;
		next_addr = 0;
		next_wdata = 0;
		next_temp = temp;
		force_PC = 0;
		next_SP = reg_SP;


		// Default is to not store, but if commiting to the register
		// file is selected, then to store to the Rd value
		alu_store = 0;
		alu_word = 0;
		alu_const = 0;
		alu_const_value = 0;
		alu_carry = 0;

		// default is to select the Rd and Rr from the opcode, storing into Rd
		alu_op = `OP_MOVE;
		sel_Ra = op_Rd;
		sel_Rb = op_Rr;
		sel_Rd = op_Rd;

		if (skip) begin
			// only a few instructions are multiple
			// bytes.  Otherwise we only skip one PC.
			if (is_call
			|| is_jmp
			|| is_lds) begin
				force_PC = 1;
				next_cycle = 1;
				next_skip = 1;
			end
		end else begin

		if (is_cpc) begin
			// CPC Rd,Rr (no dest, only sreg)
			// 16'b0000_01_?_?????_????: begin
			alu_op = `OP_SUB;
			alu_carry = 1;
		end
		if (is_cp) begin
			// CP Rd,Rr (no dest, only sreg)
			alu_op = `OP_SUB;
		end
		if (is_sbc) begin
			// SBC Rd,Rr
			alu_op = `OP_SUB;
			alu_carry = 1;
			alu_store = 1;
		end
		if (is_sub) begin
			// SUB Rd,Rr
			alu_op = `OP_SUB;
			alu_store = 1;
		end
		if (is_add) begin
			// ADD Rd,Rr / LSL Rd when Rd=Rr
			alu_store = 1;
			alu_op = `OP_ADD;
		end
		if (is_adc) begin
			// ADC Rd,Rr / ROL Rd when Rd=Rr
			alu_store = 1;
			alu_carry = 1;
			alu_op = `OP_ADD;
		end
		if (is_and) begin
			// AND Rd,Rr
			alu_store = 1;
			alu_op = `OP_AND;
		end
		if (is_eor) begin
			// EOR Rd,Rr
			alu_store = 1;
			alu_op = `OP_EOR;
		end
		if (is_or) begin
			// OR Rd,Rr
			alu_store = 1;
			alu_op = `OP_OR;
		end
		if (is_mov) begin
			// MOV Rd,Rr (no sreg updates)
			alu_store = 1;
			alu_op = `OP_MOVR;
		end
		if (is_cpi) begin
			// CPI Rd,K (only updates status register, so no dest)
			alu_op = `OP_SUB;
			sel_Ra = op_Rdi;
			alu_const = 1;
			alu_const_value = op_K;
		end
		if (is_sbci) begin
			// SBCI Rd, K
			alu_op = `OP_SUB;
			alu_carry = 1;
			sel_Ra = op_Rdi;
			sel_Rd = op_Rdi;
			alu_const_value = op_K;
			alu_const = 1;
			alu_store = 1;
		end
		if (is_subi) begin
			// SUBI Rd, K
			alu_op = `OP_SUB;
			sel_Ra = op_Rdi;
			sel_Rd = op_Rdi;
			alu_store = 1;
			alu_const = 1;
			alu_const_value = op_K;
		end
		if (is_ori) begin
			// ORI Rd,K or SBR Rd, K
			alu_op = `OP_OR;
			sel_Ra = op_Rdi;
			sel_Rd = op_Rdi;
			alu_const_value = op_K;
			alu_const = 1;
			alu_store = 1;
		end
		if (is_andi) begin
			// ANDI Rd,K or CBR Rd, K
			alu_op = `OP_AND;
			sel_Ra = op_Rdi;
			sel_Rd = op_Rdi;
			alu_const_value = op_K;
			alu_const = 1;
			alu_store = 1;
		end
		if (is_com) begin
			// COM Rd
			alu_store = 1;
			alu_op = `OP_EOR;
			alu_const = 1;
			alu_const_value = 8'hFF;
		end
		if (is_neg) begin
			// NEG Rd
			// 16'b1001_010?_????_0001: begin
			alu_store = 1;
			alu_op = `OP_NEG;
		end
		if (is_swap) begin
			// SWAP Rd, no sreg updates
			// 16'b1001_010?__????_0010: begin
			alu_store = 1;
			alu_op = `OP_SWAP;
		end
		if (is_inc) begin
			// INC Rd
			//16'b1001_010_?????_0011: begin
			alu_store = 1;
			alu_op = `OP_ADD;
			alu_const = 1;
			alu_const_value = 1;
		end
		if (is_asr) begin
			// ASR Rd
			alu_store = 1;
			alu_op = `OP_ASR;
		end
		if (is_lsr) begin
			// LSR Rd
			alu_store = 1;
			alu_op = `OP_LSR;
		end
		if (is_ror) begin
			// ROR Rd
			// 16'b1001_010?_????_0111: begin
			alu_store = 1;
			alu_op = `OP_ROR;
		end
		if (is_dec) begin
			// DEC Rd
			// 16'b1001_010?_????_1010: begin
			alu_store = 1;
			alu_op = `OP_SUB;
			alu_const = 1;
			alu_const_value = 1;
		end
		if (is_adiw_or_sbiw) begin
			// ADIW/SBIW Rp, uimm6
			sel_Ra = op_Rp;
			sel_Rd = op_Rp;
			alu_store = 1;
			alu_word = 1;
			alu_const = 1;
			alu_const_value = immw6;

			if (opcode[8])
				alu_op = `OP_SBW;
			else
				alu_op = `OP_ADW;
		end
		if (is_movw) begin
			// MOVW Rd,Rr Move register pair
			sel_Ra = { opcode[3:0], 1'b0 }; // will read both bytes
			sel_Rd = { opcode[7:4], 1'b0 }; // will write both bytes
			alu_word = 1;
			alu_store = 1;
		end
		if (is_ldi) begin
			// LDI Rdi, K (no sreg updates)
			sel_Rd = op_Rdi;
			alu_op = `OP_MOVR;
			alu_store = 1;
			alu_const = 1;
			alu_const_value = op_K;
		end
		if (is_nop) begin
			// NOP. relax!
		end
		if (is_lds) begin
			// LDS rd,i  / STS i,rd
			// No sreg update
			// 2 cycles
			// Load or store instructions
			// followed by 16-bit immediate SRAM address
			case(cycle)
			2'b00: begin
				// wait for the next read to get the address
				// for a STS the op_Rd will load the correct
				// register into reg_Ra by the next cycle
				force_PC = 1;
				next_cycle = 1;
			end
			2'b01: begin
				next_addr = cdata;
				if (is_store) begin
					// STS: write to that address
					// no extra cycle required
					next_wdata = reg_Ra[7:0];
					next_wen = 1;
				end else begin
					// LDS: request a read of the addr
					// wait at this PC
					next_cycle = 2;
					next_ren = 1;
				end
			end
			2'b10: begin
				// only LDS, store the data read from const
				alu_op = `OP_MOVR;
				alu_store = 1;
				alu_const = 1;
				alu_const_value = data_read;
			end
			endcase
		end
		if (is_ld_xyz) begin
			case(opcode[3:2])
			2'b00: sel_Ra = BASE_Z;
			2'b10: sel_Ra = BASE_Y;
			2'b11: sel_Ra = BASE_X;
			endcase

			sel_Rb = op_Rd;
			sel_Rd = sel_Ra;

			case(cycle)
			2'b00: begin
				// wait for the full X/Z register to fetch
				// as well as the contents of Rd
				next_cycle = 1;

				// setup an ALU operation to store a
				// whole word back into X/Z
				alu_word = 1;
				alu_const = 1;
				alu_const_value = 1;

				case(opcode[1:0])
				2'b01: begin
					// post-increment the register word
					alu_op = `OP_ADW;
					alu_store = 1;
				end
				2'b10: begin
					// pre-decrement the register word
					alu_op = `OP_SBW;
					alu_store = 1;
				end
				endcase
			end
			2'b01: begin
				// pointer word is in Ra, d is in Rb,
				// for a pre-decrement, pointer-1 is in alu_out
				if (opcode[1:0] == 2'b10)
					next_addr = alu_out;
				else
					next_addr = reg_Ra;

				if (is_store) begin
					// STS (no extra cycle needed)
					next_wen = 1;
					next_wdata = reg_Rb;
				end else begin
					// LD (one more cycle required)
					next_ren = 1;
					next_cycle = 2;
				end
			end
			2'b10: begin
				// extra cycle only for LD
				// the memory has loaded the value,
				// so use the ALU to store into Rd
				sel_Rd = op_Rd;
				alu_op = `OP_MOVR;
				alu_store = 1;
				alu_const = 1;
				alu_const_value = data_read;
			end
			endcase
		end
		if (is_ld_yz_plus_q) begin
			// ST / LD Rd, Y/Z+Q (no status update)
			// Z+Q: 16'b10?0_????_????_0???:
			// Y+Q: 16'b10?0_????_????_1???:
			case(cycle)
			2'b00: begin
				// wait for the full Y or Z register,
				// with the immediate value added
				// to fetch as well as the contents of Rd
				sel_Ra = opcode[3] ? BASE_Y : BASE_Z;
				sel_Rb = op_Rd;

				alu_op = `OP_ADW;
				alu_const = 1;
				alu_const_value = op_Q;
				
				next_cycle = 1;
			end
			2'b01: begin
				// reg + Q is in the alu, d is in Rb,
				next_addr = alu_out;

				if (is_store) begin
					// STS (no extra cycle needed)
					next_wen = 1;
					next_wdata = reg_Rb;
				end else begin
					// LD (one more cycle required)
					next_ren = 1;
					next_cycle = 2;
				end
			end
			2'b10: begin
				// extra cycle only for LD
				// the memory has loaded the value,
				// so use the ALU to store into Rd
				sel_Rd = op_Rd;
				alu_op = `OP_MOVR;
				alu_store = 1;
				alu_const = 1;
				alu_const_value = data_read;
			end
			endcase
		end
		if (is_lpm) begin
			// LPM/ELPM Rd, Z / Z+
			case(cycle)
			2'b00: begin
				// fetch the Z register
				sel_Ra = BASE_Z;
				next_cycle = 1;
			end
			2'b01: begin
				// if this is Z+ mode, add one to Z
				alu_op = `OP_ADW;
				sel_Ra = BASE_Z;
				sel_Rd = sel_Ra;
				alu_store = 1;
				alu_word = 1;
				alu_const = 1;
				alu_const_value = opcode[0];

				// start a read of the program memory space
				// storing the real next PC into the temp reg
				// PC is in words, not bytes
				force_PC = 1;
				next_PC = reg_Ra >> 1;
				next_temp = reg_PC;
				next_cycle = 2;
			end
			2'b10: begin
				// store the correct byte of read data,
				// based on the bottom bit of the original Z
				alu_store = 1;
				alu_op = `OP_MOVR;
				alu_const = 1;
				alu_const_value = reg_Ra[0] ? cdata[15:8] : cdata[7:0];
				// and return to the program flow by
				// reading the temp reg.
				next_PC = temp;
				force_PC = 1;
			end
			endcase
		end

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
		if (is_push) begin
			// PUSH Rd
			case(cycle)
			2'b00: begin
				// just delay until we have the Rd available
				// in register A
				next_cycle = 1;
			end
			2'b01: begin
				// enqueue the write
				next_wen = 1;
				next_addr = reg_SP;
				next_SP = reg_SP - 1;
				next_wdata = reg_Ra[7:0];
			end
			endcase
		end

		if (is_pop) begin
			// POP Rd
			case(cycle)
			2'b00: begin
				// start the read
				next_ren = 1;
				next_addr = reg_SP + 1;
				next_SP = reg_SP + 1;
				next_cycle = 1;
			end
			2'b01: begin
				alu_op = `OP_MOVR;
				alu_store = 1;
				alu_const = 1;
				alu_const_value = data_read;
			end
			endcase
		end

		if (is_clx_or_sex) begin
			// Status register update bit
			// 16'b1001_0100_1???_1000: CLx
			// 16'b1001_0100_0???_1000: SEx
			alu_op = `OP_SREG;
			alu_carry = opcode[7];
			alu_const = 1;
			alu_const_value = opcode[6:4];
		end

		if (is_ret) begin
			// RET
			case(cycle)
			2'b00: begin
				next_cycle = 1;
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
		end

		// CPSE Rd,Rr (no sreg updates)
		if(is_cpse) begin
			// wait for Rd and Rr to be available
			if (cycle == 0) begin
				next_cycle = 1;
			end else
			if (reg_Ra[7:0] == reg_Rb)
				next_skip = 1;
		end

		// SBRC/SBRS skip if register bit b equals B
		if (is_sbrc_or_sbrs) begin
			// 16'b1111_110?_????_0???, // SBRC
			// 16'b1111_111?_????_0???: // SBRS
			if(cycle == 0)
				next_cycle = 1;
			else
			if (reg_Ra[opcode[2:0]] == opcode[9])
				next_skip = 1;
		end

		// BRBS/BRBC - Branch if bit in SREG is set/clear
		if (is_brbc_or_brbs) begin
			// 16'b1111_00??_????_????, // BRBS
			// 16'b1111_01??_????_????: // BRBC
			if (sreg[opcode[2:0]] != opcode[9])
				next_PC = reg_PC + simm7 + 1;
		end

		if (is_jmp) begin
			// JMP abs22, 3 cycles
			// 16'b1001_010?_????_110?:
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
		end

		if (is_call) begin
			// CALL abs22
			// 16'b1001_010?_????_111?:
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
		end

		if (is_ijmp) begin
			// IJMP Z - Indirect jump/call to Z or EIND:Z
			// 16'b1001_010?_000?_1001:
			// 2 cycles
			case(cycle)
			2'b00: begin
				next_cycle = 1;
				sel_Ra = BASE_Z;
			end
			2'b01: begin
				next_PC = reg_Ra;
			end
			endcase
		end

		if (is_rjmp) begin
			// RJMP to PC + simm12
			// 16'b1100_????????????:
			// 2 cycles
			next_PC = reg_PC + simm12 + 1;
		end

		if (is_rcall) begin
			// RCALL to PC + simm12
			// 16'b1101_????????????:
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
				force_PC = 1;
			end
			endcase
		end

		// OUT to IO space (no sreg update)
		// the ones for registers are handled here,
		// otherwise the external controller will handle it
		// should be single cycle, except that reading
		// the register now takes a cycle
		if (is_out) begin
			// 16'b1011_1???_????_????: begin
			case(cycle)
			2'b00: begin
				// wait for Rd to show up in Ra
				next_cycle = 1;
			end
			2'b01: begin
				next_wen = 1;
				next_wdata = reg_Ra;
				next_addr = io_addr + 8'h20;

				case(io_addr)
				6'h3D: next_SP[ 7:0] = reg_Ra;
				6'h3E: next_SP[15:8] = reg_Ra;
				6'h3F: next_sreg = reg_Ra;
				default: begin
					// nothing to do here;
					// the SOC handles it
				end
				endcase
			end
			endcase
		end

		// IN from IO space (no sreg update, should be 1 cycle)
		// the registers ones are handled here, otherwise
		// the external SOC will handle it.
		if (is_in) begin
			// 16'b1011_0???_????_????: begin
			case(cycle)
			2'b00: begin
				next_addr = io_addr + 8'h20;
				next_ren = 1;
				next_cycle = 1;
			end
			2'b01: begin
				alu_op = `OP_MOVR;
				alu_store = 1;
				alu_const = 1;
				case(io_addr)
				6'h3D: alu_const_value = reg_SP[ 7:0];
				6'h3E: alu_const_value = reg_SP[15:8];
				6'h3F: alu_const_value = sreg;
				default: alu_const_value = data_read; // from the SOC
				endcase
			end
			endcase
		end
	end // skip
	end
endmodule

`endif