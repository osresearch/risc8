`default_nettype none
`include "avr.v"

module top(
	output led_r,
	output led_g,
	output led_b,
	output spi_cs
);
	assign spi_cs = 1;

	wire clk_48;
	wire reset = 0;
	SB_HFOSC u_hfosc (
		.CLKHFPU(1'b1),
		.CLKHFEN(1'b1),
		.CLKHF(clk_48)
	);
	wire clk = clk_48;

	// code memory
	reg [15:0] code[0:511];
	reg [15:0] cdata;
	wire [15:0] pc;
	initial $readmemh("test3.hex", code);

	// data memory
	wire [15:0] addr;
	reg [7:0] rdata;
	wire [7:0] wdata;
	wire wen;
	wire ren;
	reg [7:0] ram[0:8191];

	assign led_r = 1;
	assign led_g = 1;
	assign led_b = pc[0];

	always @(posedge clk)
		cdata <= code[pc];

	always @(posedge clk)
	begin
		rdata <= ram[addr[12:0]];
		if (ren) begin
			$display("RD %04x => %02x", addr, ram[addr[12:0]]);
		end

		if (wen) begin
			$display("WR %04x <= %02x", addr, wdata);
			ram[addr[12:0]] <= wdata;
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
