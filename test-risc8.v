`default_nettype none

`ifndef RISC8_PROGRAM
`define RISC8_PROGRAM "tests/test0.hex"
`endif

`include "risc8-soc.v"

module top(
);
	reg clk;
	reg reset;

	initial begin
		$dumpfile("test-risc8.vcd");
		$dumpvars(0,top);
		clk = 0;
		reset = 1;
		repeat(4) #4 clk = ~clk;
		reset = 0;
		$display("!RESET");
		forever #5 clk = ~clk;
	end

	always begin
		#24000
		$finish;
	end

	wire [7:0] port_b;
	wire [7:0] ddr_b;
	reg [7:0] pin_b;

	//always @(posedge clk)
		//$display("PORTB %02x", port_b);

	risc8_soc cpu(
		.clk(clk),
		.reset(reset),

		.port_b(port_b),
		.pin_b(pin_b),
		.ddr_b(ddr_b)
	);

endmodule
