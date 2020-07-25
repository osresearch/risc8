`ifndef _risc8_ram_
`define _risc8_ram_

`ifdef FPGA_ICE40UP5K
/*
 * ice40up5k has 256 Kbit single port block RAMs that are perfect
 * for the risc8 memory since there are either read or write cycles.
 * The SPRAMs can't be initialized in the bitstream, so the startup code
 * in the CPU is responsible for copying data to the RAM.
 *
 * They are fixed in 16-bit widths, so a wrapper is needed to make them
 * byte addressable.
 */

module ice40up5k_spram(
	input clk,
	input cs,
	input wen,
	input [14:0] addr,
	input [7:0] wdata,
	output [7:0] rdata
);
	wire align = addr[0];
	wire [15:0] rdata16;
	reg byte;
	assign rdata = byte ? rdata16[15:8] : rdata16[7:0];

	always @(posedge clk)
		byte <= align;

	SB_SPRAM256KA spram (
		.CLOCK(clk),
		.CHIPSELECT(cs),
		.WREN(wen),
		.ADDRESS(addr[14:1]),
		.DATAOUT(rdata16),
		.DATAIN({wdata, wdata}),
		.MASKWREN({align, align, !align, !align}),
		.STANDBY(1'b0),
		.SLEEP(1'b0),
		.POWEROFF(1'b1)
	);
endmodule


/*
 * Bond together two SPRAM's to make one 64 KB data RAM for
 * the risc8 CPU.  Since `addr` can change after a read, it is
 * necessary to buffer the `bank` that the address needs.
 */
module risc8_ram(
	input clk,
	input wen,
	input [15:0] addr,
	input [7:0] wdata,
	output [7:0] rdata
);

	wire [7:0] rdata_00, rdata_01;
	wire [7:0] rdata = bank ? rdata_01 : rdata_00;
	reg bank;

	always @(posedge clk)
		bank <= addr[15];

	ice40up5k_spram spram00(
		.clk(clk),
		.cs(addr[15] == 1'b0),
		.wen(wen),
		.addr(addr[14:0]),
		.rdata(rdata_00),
		.wdata(wdata)
	);

	ice40up5k_spram spram01(
		.clk(clk),
		.cs(addr[15] == 1'b1),
		.wen(wen),
		.addr(addr[14:0]),
		.rdata(rdata_01),
		.wdata(wdata)
	);
endmodule

`else

/*
 * Fall back for simulation or other FPGAs
 */
module risc8_ram(
	input clk,
	input wen,
	input [15:0] addr,
	input [7:0] wdata,
	output [7:0] rdata
);
	reg [7:0] ram[0:65535];
	reg [7:0] rdata;

	always @(posedge clk)
	begin
		rdata <= ram[addr];
		if (wen)
			ram[addr] <= wdata;
	end

endmodule

`endif

`endif
