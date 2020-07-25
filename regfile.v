`ifndef _regfile_v_
`define _regfile_v_

/*
 * The block RAM regfile has a limitation that writes are not available
 * until the next clock cycle.
 *
 * The A register can be a pair of registers, while the B is always one byte.
 */
module regfile(
	input clk,
	input reset,

	// read ports
	input [5:0] a,
	input [5:0] b,
	output [15:0] Ra,
	output [7:0] Rb,

	// write port
	input write,
	input write_word,
	input [5:0] d,
	input [15:0] Rd
);
	// duplicate the register file so that we can emulate a
	// dual-read port, single-write port block RAM
	// 32 8-bit registers == 16 16-bit words
	reg [15:0] ram_a[0:15];
	reg [15:0] ram_b[0:15];

	// there has to be a better way to do this
	initial $readmemh("zero.hex", ram_a);
	initial $readmemh("zero.hex", ram_b);

	wire [4:0] a_word = a[5:1];
	wire [4:0] b_word = b[5:1];
	wire [4:0] d_word = d[5:1];

	reg [1:0] al_src;
	reg [1:0] ah_src;
	reg [1:0] bl_src;

	reg [15:0] Ra_ram;
	reg [15:0] Rb_ram;

	reg [15:0] Ra;
	reg [ 7:0] Rb;
	reg [15:0] cache_Rd;

	always @(posedge clk) if (!reset) begin
		Ra_ram <= ram_a[a_word];
		Rb_ram <= ram_b[a_word];

		// default source is from the ram read
		// with the correct byte selected from the a and b address
		al_src <= a[0] == 0 ? 0 : 1;
		ah_src <= 1;
		bl_src <= b[0];

		if (write) begin
			cache_Rd <= Rd;

			if (write_word) begin
				// assume aligned write for d
				ram_a[d_word][15:0] <= Rd;
				ram_b[d_word][15:0] <= Rd;

				// check for cache hit
				if (a_word == d_word) begin
					if (a[0] == 0) begin
						// replace both bytes from the cache
						al_src <= 2;
						ah_src <= 3;
					end else
					if (a[1] == 1) begin
						// replace bottom byte from top byte
						al_src <= 3;
					end 
				end

				if (b_word == d_word) begin
					if (b[0] == 0) begin
						// low byte of cache
						bl_src <= 2;
					end else begin
						// high byte of cache
						bl_src <= 3;
					end
				end
			end else
			if (d[0] == 0) begin
				// low byte write
				ram_a[d_word][7:0] <= Rd[7:0];
				ram_b[d_word][7:0] <= Rd[7:0];

				if (a_word == d_word && a[0] == 0) begin
					// cache hit on low byte of A
					al_src <= 2;
				end

				if (b == d) begin
					// cache hit on low byte of B
					bl_src <= 2;
				end
			end else begin
				// high byte write
				ram_a[d_word][15:8] <= Rd[7:0];
				ram_b[d_word][15:8] <= Rd[7:0];

				if (a_word == d_word) begin
					if (a[0] == 0) begin
						// cache hit on high byte of A
						ah_src <= 2;
					end else begin
						// cache hit on low byte of A
						al_src <= 2;
					end
				end

				if (b == d) begin
					// cache hit on low byte of D
					bl_src <= 2;
				end
			end
		end
	end

	// satisfy reads from the cache
	always @(*) begin
		case(al_src)
		2'b00: Ra[7:0] = Ra_ram[7:0];
		2'b01: Ra[7:0] = Ra_ram[15:8];
		2'b10: Ra[7:0] = cache_Rd[7:0];
		2'b11: Ra[7:0] = cache_Rd[15:8];
		endcase

		case(ah_src)
		2'b00: Ra[15:8] = Ra_ram[7:0];
		2'b01: Ra[15:8] = Ra_ram[15:8];
		2'b10: Ra[15:8] = cache_Rd[7:0];
		2'b11: Ra[15:8] = cache_Rd[15:8];
		endcase

		case(bl_src)
		2'b00: Rb[7:0] = Rb_ram[7:0];
		2'b01: Rb[7:0] = Rb_ram[15:8];
		2'b10: Rb[7:0] = cache_Rd[7:0];
		2'b11: Rb[7:0] = cache_Rd[15:8];
		endcase
	end
endmodule


/*
 * The regfile caches the last written value so that if it is
 * the value being read it can be replayed from the cache.
 *
 * TODO: Fix the word writes.
 */
module regfile_register(
	input clk,
	input reset,

	// read ports
	input [5:0] a,
	input [5:0] b,
	output [15:0] Ra,
	output [7:0] Rb,

	// write port
	input write,
	input write_word,
	input [5:0] d,
	input [15:0] Rd
);
	/* flip flop based register file */
	reg [7:0] regs[0:31];
	initial begin
		regs[ 0] = 0;
		regs[ 1] = 0;
		regs[ 2] = 0;
		regs[ 3] = 0;
		regs[ 4] = 0;
		regs[ 5] = 0;
		regs[ 6] = 0;
		regs[ 7] = 0;
		regs[ 8] = 0;
		regs[ 9] = 0;
		regs[10] = 0;
		regs[11] = 0;
		regs[12] = 0;
		regs[13] = 0;
		regs[14] = 0;
		regs[15] = 0;
		regs[16] = 0;
		regs[17] = 0;
		regs[18] = 0;
		regs[19] = 0;
		regs[20] = 0;
		regs[21] = 0;
		regs[22] = 0;
		regs[23] = 0;
		regs[24] = 0;
		regs[25] = 0;
		regs[26] = 0;
		regs[27] = 0;
		regs[28] = 0;
		regs[29] = 0;
		regs[30] = 0;
		regs[31] = 0;
	end
	reg [15:0] Ra;
	reg [ 7:0] Rb;
	always @(posedge clk) begin
		Ra <= { regs[a|1], regs[a] };
		Rb <= regs[b];

		if (write) begin
			regs[d] <= Rd[7:0];

			if (write_word) begin
				// An entire word update
				regs[d|1] <= Rd[15:8];

				if (a[4:1] != d[4:1]) begin
					// no match, use the default
				end else
				if (a[0] == 0) begin
					// reading the word, so use the entire Rd
					Ra[15:0] <= Rd[15:0];
				end else begin
					// reading a high byte, so use high of Rd
					Ra[7:0] <= Rd[15:8];
				end

				if (b[4:1] != d[4:1]) begin
					// no match, use the default
				end else
				if (b[0] == 0) begin
					// reading the low byte
					Rb[7:0] <= Rd[7:0];
				end else begin
					// reading the high byte
					Rb[7:0] <= Rd[15:8];
				end
			end else begin
				// only a single byte update
				if (a[4:1] != d[4:1]) begin
					// no chance of a cache hit
				end else
				if (a[0] == d[0]) begin
					// A and D are either both high or low
					Ra[7:0] <= Rd[7:0];
				end else
 				if (d[0] == 1) begin
					// A is low, and D was high
					Ra[15:8] <= Rd[7:0];
				end

				if (b == d) begin
					// B and D are either both high or low
					Rb[7:0] <= Rd[7:0];
				end
			end
		end
	end
endmodule

`endif
