`ifndef _risc8_soc_v_
`define _risc8_soc_v_

`default_nettype none
`include "risc8-core.v"
`include "uart.v"

module risc8_soc(
	input clk,
	input reset,

	output [7:0] port_b,
	input [7:0] pin_b,
	output [7:0] ddr_b,
	output serial_tx,
	input serial_rx
);
	localparam RAMBITS = 12;
	localparam CODEBITS = 10;

	reg [7:0] port_b;
	reg [7:0] ddr_b;

	// code memory
	reg [15:0] code[0:(1 << CODEBITS) - 1];
	reg [15:0] cdata;
	wire [15:0] pc;
`ifdef RISC8_PROGRAM
	initial $readmemh(`RISC8_PROGRAM, code);
`endif

	always @(posedge clk)
		cdata <= code[pc[CODEBITS-1:0]];


	// data memory

	wire [15:0] addr;
	reg [7:0] ram_data;
	wire [7:0] wdata;
	wire wen;
	wire ren;
	reg [7:0] ram[0:(1 << RAMBITS)-1];

	//initial $readmemh("zero.hex", ram);

	// the RAM is clocked logic for reads and writes
	// it shadows the IO memory space below 64 bytes
	always @(posedge clk)
	begin
		ram_data <= ram[addr[RAMBITS-1:0]];
		if (ren) begin
			//$display("RD %04x => %02x", addr, ram[addr[RAMBITS-1:0]]);
		end

		if (wen) begin
			$display("WR %04x <= %02x", addr, wdata);
			ram[addr[RAMBITS-1:0]] <= wdata;
		end
	end

	// IO mapped peripherals are at the bottom 64-bytes of RAM
	reg io_sel;
	wire [6:0] io_addr = addr[6:0];
	reg  [7:0] io_data;

	// timer running at the clock speed
	reg [7:0] tcnt1 = 8'h55;

	// uart on the serial pins
	reg [7:0] uart_baud_div = 8'h05; // 6 Mhz / 52 == 115200
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
		.tx_out(serial_tx)
		//.rx_strobe(uart_rx_
		//.rx_data(usidr
	);

	// io writes are on the rising edge of the clock
	always @(posedge clk) begin
		tcnt1 <= tcnt1 + 1;
		uart_tx_strobe <= 0;
		io_sel <= 0;

 		if ((wen | ren) && (addr < 16'h0060)) begin
			io_sel <= 1;
			if (wen) begin
				$display("IO %02x <= %02x", io_addr, wdata);
				case(io_addr)
				7'h2F: { uart_tx_data, uart_tx_strobe } <= { wdata, 1'b1 };
				7'h2D: uart_baud_div <= wdata;
				7'h37: ddr_b <= wdata;
				7'h38: port_b <= wdata;

				7'h4F: tcnt1 <= wdata;
				endcase
			end

			if (ren) begin
				$display("IO %02x", io_addr);
				case(io_addr)
				7'h2E: io_data <= uart_status_reg;
				7'h2D: io_data <= uart_baud_div;
				7'h36: io_data <= pin_b;
				7'h37: io_data <= ddr_b;
				7'h38: io_data <= port_b;

				7'h4F: io_data <= tcnt1;
				default: io_data <= io_addr;
				endcase
			end
		end
	end

	risc8_core core(
		.clk(clk),
		.reset(reset),

		.pc(pc),
		.cdata(cdata),

		.data_addr(addr),
		.data_wen(wen),
		.data_ren(ren),
		.data_read(io_sel ? io_data : ram_data),
		.data_write(wdata)
	);

endmodule

`endif
