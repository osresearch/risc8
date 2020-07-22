`default_nettype none
`define RISC8_PROGRAM "blink.hex"
`include "risc8-soc.v"

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
	reg [2:0] clk_div;
	always @(posedge clk_48)
		clk_div <= clk_div + 1;
	wire clk = clk_div[2];

	assign led_r = ~port_b[0];
	assign led_g = ~port_b[1];
	assign led_b = ~port_b[2];

	wire [7:0] port_b;
	wire [7:0] ddr_b;
	reg [7:0] pin_b = 0;

	risc8_soc cpu(
		.clk(clk),
		.reset(reset),

		.port_b(port_b),
		.pin_b(pin_b),
		.ddr_b(ddr_b),
	);

endmodule
