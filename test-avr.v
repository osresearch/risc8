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
		#10000
		$finish;
	end

	localparam RAMBITS = 12;
	localparam CODEBITS = 10;

	// code memory
	reg [15:0] code[0:(1 << CODEBITS) - 1];
	reg [15:0] cdata;
	wire [15:0] pc;
	initial $readmemh("test4.hex", code);
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
	wire ram_sel = addr[15] == 0;
	wire io_sel = addr[15] == 1;
	reg [15:0] ram_data;
	reg [15:0] io_data;
	wire [7:0] rdata = ram_sel ? ram_data : io_sel ? io_data : 8'h00;
	wire [7:0] wdata;
	wire wen;
	wire ren;
	reg [7:0] ram[0:(1 << RAMBITS)-1];

	initial $readmemh("zero.hex", ram);

	always @(posedge clk)
		cdata <= code[pc[CODEBITS-1:0]];

	always @(posedge clk)
	begin
		ram_data <= ram[addr[RAMBITS-1:0]];
		if (ren && ram_sel) begin
			$display("RD %04x => %02x", addr, ram[addr[RAMBITS-1:0]]);
		end

		if (wen && ram_sel) begin
			$display("WR %04x <= %02x", addr, wdata);
			ram[addr[RAMBITS-1:0]] <= wdata;
		end
	end

	reg [7:0] counter = 0;
	always @(posedge clk)
	begin
		counter <= counter + 1;
		if (ren && io_sel)
			io_data <= counter;
		if (wen && io_sel) begin
			$display("IO %04x <= %02x", addr, wdata);
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
