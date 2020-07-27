`ifndef _risc8_dev_
`define _risc8_dev_
/*
 * IO bus peripherals.
 */
`include "uart.v"

module risc8_uart(
	input clk,
	input reset,
	// IO bus
	input ren,
	input wen,
	input [6:0] addr,
	input [7:0] wdata,
	output reg [7:0] rdata,
	output reg valid,
	// physical
	input rx_in,
	output tx_out
);
	parameter BASE = 7'h2D;
	reg [7:0] uart_baud_div = 12 - 1; // 12 Mhz / 12 == 1 megabaud
	reg [7:0] uart_tx_data;
	reg uart_tx_strobe;
	wire uart_tx_ready;

	wire [7:0] uart_status_reg = { 7'b0000000, uart_tx_ready };

	uart uart(
		.clk(clk),
		.reset(reset),
		.baud_div(uart_baud_div),
		.tx_strobe(uart_tx_strobe),
		.tx_data(uart_tx_data),
		.tx_ready(uart_tx_ready),
		.tx_out(tx_out)
		//.rx_strobe(uart_rx_
		//.rx_data(usidr
	);

	always @(posedge clk) begin
		uart_tx_strobe <= 0;
		valid <= 0;

		if (wen) case(addr)
		BASE + 0: uart_baud_div <= wdata;
		BASE + 2: { uart_tx_data, uart_tx_strobe } <= { wdata, 1'b1 };
		endcase

		if (ren) case(addr)
		BASE + 0: { rdata, valid } <= { uart_baud_div, 1'b1 };
		BASE + 1: { rdata, valid } <= { uart_status_reg, 1'b1 };
		endcase
	end
endmodule

module risc8_timer(
	input clk,
	input reset,
	// IO bus
	input ren,
	input wen,
	input [6:0] addr,
	input [7:0] wdata,
	output reg [7:0] rdata,
	output reg valid
);
	parameter BASE = 7'h4F;

	// timer running at the clock speed
	reg [7:0] tcnt1 = 8'h55;

	always @(posedge clk) begin
		valid <= 0;
		tcnt1 <= tcnt1 + 1;

		if (wen) case(addr)
		BASE + 0: tcnt1 <= wdata;
		endcase

		if (ren) case(addr)
		BASE + 0: { rdata, valid } <= { tcnt1, 1'b1 };
		endcase
	end
endmodule

module risc8_gpio(
	input clk,
	input reset,
	// IO bus
	input ren,
	input wen,
	input [6:0] addr,
	input [7:0] wdata,
	output reg [7:0] rdata,
	output reg valid,
	// physical
	output [7:0] port,
	input [7:0] pin,
	output [7:0] ddr
);
	parameter BASE = 7'h36;
	reg [7:0] port;
	reg [7:0] ddr;

	always @(posedge clk) begin
		if (reset) begin
			port <= 0;
			ddr <= 0;
		end

		valid <= 0;

		if (wen) case(addr)
		//BASE + 0: pin <= wdata;
		BASE + 1: ddr <= wdata;
		BASE + 2: begin
			$display("PORT %02x", wdata);
			port <= wdata;
		end
		endcase

		if (ren) case(addr)
		BASE + 0: { rdata, valid } <= { pin, 1'b1 };
		BASE + 1: { rdata, valid } <= { ddr, 1'b1 };
		BASE + 2: { rdata, valid } <= { port, 1'b1 };
		endcase
	end
endmodule

`endif
