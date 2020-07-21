`ifndef _regfile_v_
`define _regfile_v_

/*
 * The block RAM regfile has a limitation that writes are not available
 * until the next clock cycle.
 *
 * The A register can be a pair of registers, while the B is always one byte.
 */
module regfile_ram(
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

	initial $readmemh("zero.hex", ram_a);
	initial $readmemh("zero.hex", ram_b);

	reg [15:0] Ra_ram;
	reg [15:0] Rb_ram;
	reg a_high;
	reg b_high;

	wire [4:0] a_word = a[5:1];
	wire [4:0] b_word = b[5:1];
	wire [4:0] d_word = d[5:1];

	always @(negedge clk)
		$display("----");

	assign Ra = a_high ? { 8'h00, Ra_ram[15:8] } : Ra_ram[15:0];
	assign Rb = b_high ? Rb_ram[15:8] : Rb_ram[7:0];
	

	always @(posedge clk) if (!reset) begin
		a_high <= a[0];
		b_high <= b[0];
		Ra_ram <= ram_a[a_word];
		Rb_ram <= ram_b[b_word];

		if (write)
			$display("R[%d]=%04x R[%d]=%02x R[%d]<=%04x", a, Ra, b, Rb, d, Rd);
		else
			$display("READ %d=%04x %d=%04x", a, ram_a[a_word], b, ram_b[b_word]);
		//$display("r20=%04x %04x", Rb_ram, ram_b[13]);

		if (write) begin
			if (write_word) begin
				// assume aligned write
				$display("WRITE %d word <= %04x", d_word, Rd);
				ram_a[d_word][15:0] <= Rd;
				ram_b[d_word][15:0] <= Rd;
			end else
			if (d[0]) begin
				// high byte write
				$display("WRITE %d high <= %02x", d_word, Rd);
				ram_a[d_word][15:8] <= Rd[7:0];
				ram_b[d_word][15:8] <= Rd[7:0];
			end else begin
				// low byte write
				$display("WRITE %d low <= %02x", d_word, Rd);
				ram_a[d_word][7:0] <= Rd[7:0];
				ram_b[d_word][7:0] <= Rd[7:0];
			end
		end
	end
endmodule


/*
 * The regfile caches the last written value so that if it is
 * the value being read it can be replayed from the cache.
 *
 * TODO: Fix the word writes.
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
	reg  [15:0] Ra;
	reg  [ 7:0] Rb;
	wire [15:0] Ra_ram;
	wire [ 7:0] Rb_ram;

	regfile_ram ram_regs(
		.clk(clk),
		.reset(reset),
		// Read ports
		.a(a),
		.b(b),
		.Ra(Ra_ram),
		.Rb(Rb_ram),
		// Write port
		.d(d),
		.Rd(Rd),
		.write(write),
		.write_word(write_word)
	);

	// If the register being read matches the last one written,
	// replay it from the input buffer instead of from the BRAM.
	reg [15:0] cache_Rd;
	reg [5:0] cache_d;
	reg cache_valid;
	reg cache_word;

	always @(posedge clk) begin
		cache_valid <= 0;

		if (!reset && write) begin
			cache_valid <= 1;
			cache_Rd <= Rd;
			cache_d <= d;
			cache_word <= write_word;
		end
	end

	// Handle un-aligned reads of the word-oriented block rams
	// with matching the last written word
	always @(*) begin
		// default is to use the value from RAM
		Ra = Ra_ram;
		Rb = Rb_ram;

/*
		if (cache_valid) begin
			// assume no word writes
			if (a == cache_d) begin
				$display("CACHE HIT A %d = %04x", a, cache_Rd);
				Ra = cache_Rd;
			end
			if (b == cache_d) begin
				$display("CACHE HIT B %d = %02x", b, cache_Rd);
				Rb = cache_Rd;
			end
		end
*/
		if (a == d)
			Ra = Rd;
		if (b == d)
			Rb = Rd;
	end
endmodule

`endif
