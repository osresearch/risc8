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

//`define CONFIG_MULU

`include "risc8-alu.v"
`include "risc8-regs.v"
`include "risc8-instr.v"

//`define config_is_inc
//`define config_is_dec
`define config_is_com
`define config_is_adiw_or_sbiw
`define config_is_movw
`define config_is_clx_or_sex
//`define config_is_mulu
`define config_is_out
`define config_is_in
//`define config_is_lds
`define config_is_ld_xyz
`define config_is_ld_yz_plus_q
`define config_is_lpm
`define config_is_push
`define config_is_pop
`define config_is_ret
`define config_is_cpse
`define config_is_sbrc_or_sbrs
`define config_is_brbc_or_brbs
//`define config_is_jmp
//`define config_is_call
//`define config_is_ijmp
`define config_is_rjmp
`define config_is_rcall
`define config_is_sbis_or_sbic



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


	risc8_regs regs(
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

	// the PC output is almost always the actual PC,
	// although sometimes it is the address for a LPM
	// or a LDS instruction that uses the next cdata
	assign pc = next_PC;
	reg [15:0] next_PC;
	reg force_PC;

	// Some instructions require an extra cycle;
	// they will set cycle and re-use the previous opcode
	reg [1:0] cycle;
	reg [1:0] next_cycle;

	// Some instruction can cause the next instruction to be skipped,
	// which might be multiple words; this still executes the instruction,
	// but doesn't write any results
	reg skip;
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

	reg is_invalid;
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
	wire [5:0] op_Rp = { 2'b11, opcode[5:4], 1'b0 }; // 24-30
	wire [7:0] op_K = { opcode[11:8], opcode[3:0] };
	wire [5:0] op_Q = { opcode[13], opcode[11:10], opcode[2:0] };

	// IN and OUT instructions
	wire [5:0] io_addr = { opcode[10:9], opcode[3:0] };
	wire [2:0] op_bit_select = opcode[2:0];
	wire op_bit_set = opcode[9];

	// LD vs ST is in the 9th bit
	wire op_is_store = opcode[9];

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

		if (is_invalid)
			$display("INVALID %04x", opcode);
	end

	wire [4:0] instr;
	wire [3:0] is_alu_op;
	wire is_alu_rdi;
	wire is_alu_store;
	wire is_alu_carry;
	risc8_instruction decoder(
		.opcode(opcode),
		.instr(instr),
		.alu_op(is_alu_op),
		.alu_store(is_alu_store),
		.alu_carry(is_alu_carry),
		.alu_rdi(is_alu_rdi)
	);

	/*******************************/
	reg do_sp_push;
	reg do_sp_pop;
	reg do_ldst;
	reg do_alu_ldst;
	reg do_reg_ldst;
	reg do_data_load;

	always @(*) begin
		// start pre-fetching the next PC if we are not in reset
		if (reset)
			next_PC = 0;
		else
			next_PC = reg_PC + 1;

		// most instructions are single cycle, no writes, no reads
		is_invalid = 0;
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

		// micro-ops
		do_sp_push = 0;
		do_sp_pop = 0;
		do_ldst = 0;
		do_alu_ldst = 0;
		do_reg_ldst = 0;
		do_data_load = 0;

		// Default is to not store, but if commiting to the register
		// file is selected, then to store to the Rd value
		alu_store = 0;
		alu_word = 0;
		alu_const = 0;
		alu_const_value = 0;
		alu_carry = 0;

		// default is to select the Rd and Rr from the opcode,
		// storing into Rd.  Most instructions modify these
		alu_op = `OP_MOVE;
		sel_Ra = op_Rd;
		sel_Rb = op_Rr;
		sel_Rd = op_Rd;

		if (skip) begin
			// only a few instructions are multiple
			// bytes.  Otherwise we only skip one PC.
			if (instr == `is_call
			|| instr == `is_jmp
			|| instr == `is_lds) begin
				force_PC = 1;
				next_cycle = 1;
				next_skip = 1;
			end
		end else

		(* full_case *)
		case(instr)
		`is_alu: begin
			alu_op = is_alu_op;
			alu_carry = is_alu_carry;
			alu_store = is_alu_store;

			if (is_alu_rdi) begin
				sel_Ra = op_Rdi;
				sel_Rd = op_Rdi;
				alu_const = 1;
				alu_const_value = op_K;
			end
		end

`ifdef config_is_inc
		`is_inc: begin
			// INC Rd
			alu_op = `OP_ADD;
			alu_store = 1;
			alu_const = 1;
			alu_const_value = 1;
		end
`endif
`ifdef config_is_dec
		`is_dec: begin
			// DEC Rd
			alu_op = `OP_SUB;
			alu_store = 1;
			alu_const = 1;
			alu_const_value = 1;
		end
`endif
`ifdef config_is_com
		`is_com: begin
			// COM Rd
			alu_op = `OP_EOR;
			alu_store = 1;
			alu_const = 1;
			alu_const_value = 8'hFF;
		end
`endif
`ifdef config_is_adiw_or_sbiw
		`is_adiw_or_sbiw: begin
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
`endif
`ifdef config_is_movw
		`is_movw: begin
			// MOVW Rd,Rr Move register pair
			sel_Ra = { opcode[3:0], 1'b0 }; // will read both bytes
			sel_Rd = { opcode[7:4], 1'b0 }; // will write both bytes
			alu_word = 1;
			alu_store = 1;
		end
`endif
`ifdef config_is_clx_or_sex
		`is_clx_or_sex: begin
			// Status register update bit
			// 16'b1001_0100_1???_1000: CLx
			// 16'b1001_0100_0???_1000: SEx
			alu_op = `OP_SREG;
			alu_carry = opcode[7];
			alu_const = 1;
			alu_const_value = opcode[6:4];
		end
`endif
`ifdef config_is_mulu
		`is_mulu: begin
			// MULU Rd, Rr => R1/R0
			alu_op = `OP_MUL;
			alu_store = 1;
			alu_word = 1;
			sel_Rd = 0;
		end
`endif

`ifdef config_is_out
		// OUT to IO space (no sreg update)
		// the ones for registers are handled here,
		// otherwise the external controller will handle it
		// should be single cycle, except that reading
		// the register now takes a cycle
		`is_out: begin
			if(cycle[0] == 0) begin
				// wait for Rd to show up in Ra
				next_cycle = 1;
			end else begin
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
		end
`endif

`ifdef config_is_in
		// IN from IO space (no sreg update, should be 1 cycle)
		// the registers ones are handled here, otherwise
		// the external SOC will handle it.
		`is_in: begin
			if(cycle[0] == 0) begin
				next_addr = io_addr + 8'h20;
				next_ren = 1;
				next_cycle = 1;
			end else begin
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
		end
`endif

`ifdef config_is_lds
		`is_lds: begin
			// LDS rdi,i  / STS i,rdi
			// No sreg update
			// 2 cycles
			// Load or store instructions
			// followed by 16-bit immediate SRAM address
			sel_Rb = op_Rdi;
			sel_Rd = op_Rdi;

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
				do_ldst = 1;
			end
			2'b10: do_data_load = 1;
			endcase
		end
`endif

`ifdef config_is_ld_xyz
		`is_ld_xyz: begin
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
					do_alu_ldst = 1;
				else
					do_reg_ldst = 1;
			end
			2'b10: begin
				sel_Rd = op_Rd;
				do_data_load = 1;
			end
			endcase
		end
`endif

`ifdef config_ld_yz_plus_q
		`is_ld_yz_plus_q: begin
			// ST / LD Rd, Y/Z+Q (no status update)
			// Z+Q: 16'b10?0_????_????_0???:
			// Y+Q: 16'b10?0_????_????_1???:
			sel_Ra = opcode[3] ? BASE_Y : BASE_Z;
			sel_Rb = op_Rd;
			sel_Rd = op_Rd;

			case(cycle)
			2'b00: begin
				// wait for the full Y or Z register,
				// with the immediate value added
				// to fetch as well as the contents of Rd
				alu_op = `OP_ADW;
				alu_const = 1;
				alu_const_value = op_Q;
				
				next_cycle = 1;
			end
			2'b01: do_alu_ldst = 1;
			2'b10: do_data_load = 1;
			endcase
		end
`endif

`ifdef config_is_lpm
		`is_lpm: begin
			// LPM/ELPM Rd, Z / Z+
			sel_Ra = BASE_Z;
			sel_Rd = sel_Ra;

			case(cycle)
			2'b00: begin
				// fetch the Z register
				next_cycle = 1;
			end
			2'b01: begin
				// if this is Z+ mode, add one to Z
				alu_op = `OP_ADW;
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
				// store the correct byte of read data into Rd
				// based on the bottom bit of the original Z
				alu_op = `OP_MOVR;
				alu_store = 1;
				alu_const = 1;
				alu_const_value = reg_Ra[0] ? cdata[15:8] : cdata[7:0];
				sel_Rd = op_Rd;

				// Exception for simple LPM
				if(opcode == 16'b1001_0101_1100_1000) sel_Rd = 0;

				// restore the PC, and do one more cycle
				// so that the next_PC will prefetch the
				// correct next instruction
				force_PC = 1;
				next_PC = temp;
				next_cycle = 3;
			end
			2'b11: begin
				// nothing to do, just allow prefetch to work
			end
			endcase
		end
`endif

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
		if (do_ldst) begin
			if (op_is_store) begin
				// STS (no extra cycle needed)
				next_wen = 1;
				next_wdata = reg_Rb;
			end else begin
				// LD (one more cycle required)
				next_ren = 1;
				next_cycle = 2;
			end
		end
*/

`ifdef config_is_push
		`is_push: begin
			next_wdata = reg_Ra[7:0];

			// PUSH Rd
			// delay one cycle until we have the Rd
			// available in register A
			if(cycle[0] == 0)
				next_cycle = 1;
			else
				do_sp_push = 1;
		end
`endif

`ifdef config_is_pop
		`is_pop: begin
			// POP Rd
			// start the read and load the data into Rd
			// once it is ready on the next cycle
			if(cycle[0] == 0)
				do_sp_pop = 1;
			else
				do_data_load = 1;
		end
`endif

`ifdef config_is_ret
		`is_ret: begin
			// RET
			case(cycle)
			2'b00: begin
				do_sp_pop = 1;
				next_cycle = 1;
			end
			2'b01: begin
				do_sp_pop = 1;
				next_temp[7:0] = data_read;
				next_cycle = 2;
			end
			2'b10: begin
				next_PC = { temp[7:0], data_read };
			end
			endcase
		end
`endif

`ifdef config_is_cpse
		// CPSE Rd,Rr
		`is_cpse: begin
			// wait for Rd and Rr to be available
			if (cycle[0] == 0)
				next_cycle = 1;
			else
			if (reg_Ra[7:0] == reg_Rb)
				next_skip = 1;
		end
`endif

`ifdef config_is_sbrc_or_sbrs
		// SBRC/SBRS skip if register bit b equals B
		`is_sbrc_or_sbrs: begin
			// 16'b1111_110?_????_0???, // SBRC
			// 16'b1111_111?_????_0???: // SBRS
			if(cycle[0] == 0)
				next_cycle = 1;
			else
			if (reg_Ra[op_bit_select] == op_bit_set)
				next_skip = 1;
		end
`endif

`ifdef config_is_brbc_or_brbs
		// BRBS/BRBC - Branch if bit in SREG is set/clear
		// this happens while the ALU is still computing the
		// previous instruction, so use the next SREG value,
		// not the current register.
		`is_brbc_or_brbs: begin
			// 16'b1111_00??_????_????, // BRBS
			// 16'b1111_01??_????_????: // BRBC
			if (next_sreg[op_bit_select] != op_bit_set)
				next_PC = reg_PC + simm7 + 1;
		end
`endif

`ifdef config_is_jmp
		`is_jmp: begin
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
`endif

`ifdef config_is_call
		`is_call: begin
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
				do_sp_push = 1;
				next_temp = cdata;
				next_wdata = next_PC[7:0];
				next_cycle = 2;
			end
			2'b10: begin
				// write the second half of the return address
				next_wdata = next_PC[15:8];
				do_sp_push = 1;
				next_cycle = 3;
			end
			2'b11: begin
				// 22-bit PC has extra bits in opcode
				// but we are a 16-bit PC CPU, so ignored
				next_PC = temp;
			end
			endcase
		end
`endif

`ifdef config_is_ijmp
		`is_ijmp: begin
			// IJMP Z - Indirect jump/call to Z or EIND:Z
			// 16'b1001_010?_000?_1001:
			// 2 cycles
			sel_Ra = BASE_Z;
			if(cycle[0] == 0)
				next_cycle = 1;
			else
				next_PC = reg_Ra;
		end
`endif

`ifdef config_is_rjmp
		`is_rjmp: begin
			// RJMP to PC + simm12
			// 16'b1100_????????????:
			// 2 cycles
			next_PC = reg_PC + simm12 + 1;
		end
`endif

`ifdef config_is_rcall
		`is_rcall: begin
			// RCALL to PC + simm12
			// 16'b1101_????????????:
			// 3 cycles
			case(cycle)
			2'b00: begin
				// push the first half of the PC
				do_sp_push = 1;
				next_wdata = next_PC[7:0]; // pc + 1
				next_cycle = 1;
			end
			2'b01: begin
				// push the second half
				do_sp_push = 1;
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
`endif

`ifdef config_is_sbis_or_sbic
		// Skip if bit in IO space is set or clear.
		`is_sbis_or_sbic: begin
			if (cycle[0] == 0) begin
				next_addr = opcode[7:3] + 8'h20;
				next_ren = 1;
				next_cycle = 1;
			end else
			if (data_read[op_bit_select] == op_bit_set)
				next_skip = 1;
		end
`endif

		default: begin
			is_invalid = 1;
		end
		endcase


		/*
		 * Micro-ops
		 */

		// post-decrement the stack pointer
		// and start a write of next_wdata to the stack
		if (do_sp_push) begin
			next_wen = 1;
			next_addr = reg_SP;
			next_SP = reg_SP - 1;
		end

		// pre-increment the stack pointer
		// and start a read of the stack, will be in read_data
		if (do_sp_pop) begin
			next_ren = 1;
			next_addr = reg_SP + 1;
			next_SP = reg_SP + 1;
		end

		// complete a load/store using either the ALU
		// or Ra output
		if (do_alu_ldst) begin
			next_addr = alu_out;
			do_ldst = 1;
		end
		if (do_reg_ldst) begin
			next_addr = reg_Ra;
			do_ldst = 1;
		end

		// continue a load from the address in next_addr,
		// using the data into Rb. This must be called
		// on cycle 1 (or else the next_cycle will be wrong).
		if (do_ldst) begin
			if (op_is_store) begin
				// STS (no extra cycle needed)
				next_wen = 1;
				next_wdata = reg_Rb;
			end else begin
				// LD (one more cycle required)
				next_ren = 1;
				next_cycle = 2;
			end
		end

		// finish a load by copying the data into Rd
		if (do_data_load) begin
			// extra cycle only for LD
			// the memory has loaded the value,
			// so use the ALU to store into Rd
			alu_op = `OP_MOVR;
			alu_store = 1;
			alu_const = 1;
			alu_const_value = data_read;
		end
	end
endmodule

`endif
