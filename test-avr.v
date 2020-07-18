`default_nettype none
`include "avr.v"

module top(
);
	reg clk;
	reg reset;

	initial begin
		$dumpfile("test-avr.vcd");
		$dumpvars(0,top);
		clk = 0;
		reset = 1;
		repeat(4) #4 clk = ~clk;
		reset = 0;
		$display("!RESET");
		forever #5 clk = ~clk;
	end

	always begin
		#1000
		$finish;
	end

	// code memory
	reg [15:0] code[0:127];
	reg [15:0] cdata;
	wire [15:0] pc;
	initial $readmemh("test1.hex", code);
/*
	initial begin
		code[0] <= 16'h0000;
		code[1] <= 16'he011;
		code[2] <= 16'he019;
		code[3] <= 16'he022;
		code[4] <= 16'he033;
		code[5] <= 16'he044;
		code[6] <= 16'h5042;
		code[7] <= 16'h1b43;
	end
*/

	// data memory
	wire [15:0] addr;
	reg [7:0] rdata;
	wire [7:0] wdata;
	wire wen;
	wire ren;
	reg [7:0] ram[0:255];

	always @(posedge clk)
		cdata <= code[pc];

	always @(posedge clk)
	begin
		rdata <= ram[addr[7:0]];
		if (ren) begin
			$display("RD %04x => %02x", addr, ram[addr[7:0]]);
		end

		if (wen) begin
			$display("WR %04x <= %02x", addr, wdata);
			ram[addr[7:0]] <= wdata;
		end
	end

	avr_cpu cpu(
		.clk(clk),
		.reset(reset),

		.pc(pc),
		.cdata(cdata),

		.data_addr(addr),
		.data_wen(wen),
		.data_ren(ren),
		.data_read(rdata),
		.data_write(wdata)
	);

endmodule
