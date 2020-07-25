`ifndef _risc8_ram_
`define _risc8_ram_

`ifdef FPGA_ICE40UP5K
/*
 * ice40up5k has 256 Kbit single port block RAMs that are perfect
 * for the risc8 memory since there are either read or write cycles.
 * The SPRAMs can't be initialized in the bitstream, so the startup code
 * in the CPU is responsible for copying data to the RAM.
 */
module risc8_ram(
	input clk,
	input wen,
	input [15:0] addr,
	input [7:0] wdata,
	output [7:0] rdata
);
	wire high = addr[0];
	wire cs_0 = addr[15] == 0;
	wire cs_1 = addr[15] == 1;
	wire [15:0] rdata_0, rdata_1;
	wire [ 7:0] rdata_0_byte = high ? rdata_0[15:8] : rdata_0[7:0];
	wire [ 7:0] rdata_1_byte = high ? rdata_1[15:8] : rdata_1[7:0];

	assign rdata = cs_1 ? rdata_1_byte : rdata_0_byte;

	wire [3:0] wen_mask = !wen ? 4'b0000 : { high, high, !high, !high };

	SB_SPRAM256KA ram00 (
		.ADDRESS(addr[14:1]),
		.DATAIN({wdata, wdata}),
		.MASKWREN(wen_mask),
		.WREN(wen),
		.CHIPSELECT(cs_0),
		.CLOCK(clk),
		.STANDBY(1'b0),
		.SLEEP(1'b0),
		.POWEROFF(1'b1),
		.DATAOUT(rdata_0[15:0])
	);

	SB_SPRAM256KA ram01 (
		.ADDRESS(addr[14:1]),
		.DATAIN({wdata, wdata}),
		.MASKWREN(wen_mask),
		.WREN(wen),
		.CHIPSELECT(cs_1),
		.CLOCK(clk),
		.STANDBY(1'b0),
		.SLEEP(1'b0),
		.POWEROFF(1'b1),
		.DATAOUT(rdata_1[15:0])
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
