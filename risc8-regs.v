`ifndef _risc8_regfile_v_
`define _risc8_regfile_v_

/*
 * The block RAM regfile has a limitation that writes are not available
 * until the next clock cycle.
 *
 * The A register can be a pair of registers, while the B is always one byte.
 */
module risc8_regs(
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
		Rb_ram <= ram_b[b_word];

		// default source is from the ram read
		// with the correct byte selected from the a and b address
		al_src <= { 1'b0, a[0] };
		ah_src <= { 1'b0, 1'b1 };
		bl_src <= { 1'b0, b[0] };

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
						al_src <= 2'b10;
						ah_src <= 2'b11;
					end else
					if (a[0] == 1) begin
						// replace bottom byte from top byte
						al_src <= 2'b11;
					end 
				end

				if (b_word == d_word) begin
					if (b[0] == 0) begin
						// low byte of cache
						bl_src <= 2'b10;
					end else begin
						// high byte of cache
						bl_src <= 2'b11;
					end
				end
			end else
			if (d[0] == 0) begin
				// low byte write
				//$display("R[%dL] <= %02x", d_word, Rd[7:0]);
				ram_a[d_word][7:0] <= Rd[7:0];
				ram_b[d_word][7:0] <= Rd[7:0];

				if (a_word == d_word && a[0] == 0) begin
					// cache hit on low byte of A
					al_src <= 2'b10;
				end

				if (b == d) begin
					// cache hit on low byte of B
					bl_src <= 2'b10;
				end
			end else begin
				// high byte write
				//$display("R[%dH] <= %02x", d_word, Rd[7:0]);
				ram_a[d_word][15:8] <= Rd[7:0];
				ram_b[d_word][15:8] <= Rd[7:0];

				if (a_word == d_word) begin
					if (a[0] == 0) begin
						// cache hit on high byte of A
						ah_src <= 2'b10;
					end else begin
						// cache hit on low byte of A
						al_src <= 2'b10;
					end
				end

				if (b == d) begin
					// cache hit on low byte of D
					bl_src <= 2'b10;
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

`endif
