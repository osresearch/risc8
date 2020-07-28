`ifndef _risc8_soc_v_
`define _risc8_soc_v_

`default_nettype none
`include "risc8-core.v"
`include "risc8-ram.v"
`include "risc8-dev.v"

`ifndef RISC8_PROGRAM
`define RISC8_PROGRAM "program.syn.hex"
`endif



module risc8_soc(
	input clk,
	input reset,

	output [7:0] port_b,
	input [7:0] pin_b,
	output [7:0] ddr_b,
	output serial_tx,
	input serial_rx
);
	localparam CODEBITS = 12;

	// code memory (stored in normal block RAM)
	reg [15:0] code[0:(1 << CODEBITS) - 1];
	reg [15:0] cdata;
	wire [15:0] pc;
`ifdef RISC8_PROGRAM
	initial $readmemh(`RISC8_PROGRAM, code);
`endif

	always @(posedge clk)
		cdata <= code[pc[CODEBITS-1:0]];


	// data memory (stored in up5k SPRAM or in normal block RAM)
	// byte addressable, single port, either read or write but not both
	wire [15:0] addr;
	wire [7:0] ram_data;
	wire [7:0] wdata;
	wire wen;
	wire ren;

	risc8_ram ram(
		.clk(clk),
		.wen(wen),
		.addr(addr),
		.wdata(wdata),
		.rdata(ram_data)
	);

	// IO mapped peripherals are at the bottom 64-bytes of RAM
	wire [6:0] io_addr = addr[6:0];
	wire io_sel = addr[15:7] == 0;
	wire io_ren = io_sel & ren;
	wire io_wen = io_sel & wen;

	// uart on the serial pins
	wire [7:0] uart_data;
	wire uart_valid;
	risc8_uart uart(
		.clk(clk),
		.reset(reset),
		// logical
		.addr(io_addr),
		.ren(io_ren),
		.wen(io_wen),
		.wdata(wdata),
		.rdata(uart_data),
		.valid(uart_valid),
		// physical
		.tx_out(serial_tx)
	);

	// timer
	wire [7:0] tcnt1_data;
	wire tcnt1_valid;
	risc8_timer #(.BASE(7'h3F)) tcnt1(
		.clk(clk),
		.reset(reset),
		// logical
		.addr(io_addr),
		.ren(io_ren),
		.wen(io_wen),
		.wdata(wdata),
		.rdata(tcnt1_data),
		.valid(tcnt1_valid)
	);

	// gpio for port b
	wire [7:0] portb_data;
	wire portb_valid;
	risc8_gpio #(.BASE(7'h36)) portb_dev(
		.clk(clk),
		.reset(reset),
		// logical
		.addr(io_addr),
		.ren(io_ren),
		.wen(io_wen),
		.wdata(wdata),
		.rdata(portb_data),
		.valid(portb_valid),
		// physical
		.port(port_b),
		.pin(pin_b),
		.ddr(ddr_b)
	);

	// Memory or peripheral read response
	reg [7:0] rdata;
	always @(*) begin
		rdata = ram_data;
		if (uart_valid) rdata = uart_data;
		if (tcnt1_valid) rdata = tcnt1_data;
		if (portb_valid) rdata = portb_data;
	end

	always @(posedge clk)
		if (wen) $display("WR %04x <= %02x", addr, wdata);

	risc8_core core(
		.clk(clk),
		.reset(reset),

		// Program memory
		.pc(pc),
		.cdata(cdata),

		// Data memory and IO bus
		.data_addr(addr),
		.data_wen(wen),
		.data_ren(ren),
		.data_read(rdata),
		.data_write(wdata)
	);

endmodule

`endif
