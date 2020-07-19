`ifndef _avr_soc_v_
`define _avr_soc_v_

`default_nettype none
`include "avr.v"

module avr_soc(
	input clk,
	input reset,

	output [7:0] port_b,
	input [7:0] pin_b,
	output [7:0] ddr_b
);
	localparam RAMBITS = 12;
	localparam CODEBITS = 10;

	reg [7:0] port_b;
	reg [7:0] ddr_b;

	// code memory
	reg [15:0] code[0:(1 << CODEBITS) - 1];
	reg [15:0] cdata;
	wire [15:0] pc;
`ifdef AVR_PROGRAM
	initial $readmemh(`AVR_PROGRAM, code);
`endif

	always @(posedge clk)
		cdata <= code[pc[CODEBITS-1:0]];


	// data memory

	wire [15:0] addr;
	reg [7:0] ram_rdata;
	wire [7:0] wdata;
	wire wen;
	wire ren;
	reg [7:0] ram[0:(1 << RAMBITS)-1];

	//initial $readmemh("zero.hex", ram);

	// the RAM is clocked logic for reads and writes
	// it shadows the IO memory space below 64 bytes
	always @(posedge clk)
	begin
		ram_rdata <= ram[addr[RAMBITS-1:0]];
		if (ren) begin
			$display("RD %04x => %02x", addr, ram[addr[RAMBITS-1:0]]);
		end

		if (wen) begin
			$display("WR %04x <= %02x", addr, wdata);
			ram[addr[RAMBITS-1:0]] <= wdata;
		end
	end

	// IO mapped peripherals are at the bottom 64-bytes of RAM
	// and require continuous assignment
	wire io_sel = addr < 16'h0060;
	wire [6:0] io_addr = addr[6:0];
	reg  [7:0] io_rdata;

	reg [7:0] tcnt1 = 0;

	// io writes are on the rising edge of the clock
	always @(posedge clk) begin
		tcnt1 <= tcnt1 + 1;

		if (wen && io_sel) begin
			$display("IO %02x <= %02x", io_addr, wdata);
			case(io_addr)
			7'h37: ddr_b <= wdata;
			7'h38: port_b <= wdata;

			7'h4F: tcnt1 <= wdata;
			endcase
		end
	end

	// io reads are continuous assignment since they must be 
	// available in a single cycle.  note that these are memory
	// addresses, not IO port addresses
	always @(*) begin
		case(io_addr)
		7'h36: io_rdata = pin_b;
		7'h37: io_rdata = ddr_b;
		7'h38: io_rdata = port_b;

		7'h4F: io_rdata = tcnt1;

		default: io_rdata = io_addr;
		endcase
		//if (io_sel && ren)
			//$display("IN %02x = %02x", io_addr, io_rdata);
	end

	avr_cpu cpu(
		.clk(clk),
		.reset(reset),

		.pc(pc),
		.cdata(cdata),

		.data_addr(addr),
		.data_wen(wen),
		.data_ren(ren),
		.data_read(io_sel ? io_rdata : ram_rdata),
		.data_write(wdata)
	);

endmodule

`endif
