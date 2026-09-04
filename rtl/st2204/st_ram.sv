// Copyright (c) 2026 Jamie Blanks
//
// ST2204 internal SRAM, $0080-$27FF as observed on the GameKing.
//
// Dual-ported in BRAM: port A serves the CPU and DMA, port B the LCDC fetch.
// The real chip arbitrates a single array with the LCDC winning and the CPU
// stalling; the dual port removes the need for wait states on the fetch side
// while st_lcdc still asserts its stall so the CPU timing stays right.

module st_ram
(
	input  wire        clk,
	input  wire        reset,

	// CPU/DMA port
	input  wire        sel,
	input  wire [13:0] addr,
	input  wire        we,             // ce-qualified by st_bus
	input  wire [7:0]  wdata,
	output wire [7:0]  rdata,

	// LCDC fetch port (read-only)
	input  wire [13:0] lcdc_addr,
	output wire [7:0]  lcdc_rdata,

	// Savestate memory port (multiplexed onto port A while the core sleeps)
	input  wire        ss_active,
	input  wire [13:0] ss_mem_addr,
	input  wire        ss_mem_wren,
	input  wire [7:0]  ss_mem_wdata,
	output wire [7:0]  ss_mem_rdata
);

	// Cold-boot clear. Reset sweeps the array to zero through port A, which
	// is free because the CPU is held. Without it an OSD reset is a warm
	// boot: the BIOS finds its cold/warm flag and the rest of RAM intact
	// and skips cold initialization. The sweep starts at each reset
	// assertion and runs only while reset is held - 16K clocks, well inside
	// any reset window - and aborts on release so it can never contend with
	// the CPU port. It stays out of the way while the savestate controller
	// owns the port, since a restore pulses reset before rewriting RAM.
	reg [13:0] clr_addr = 14'd0;
	reg        clr_run  = 1'b0;
	reg        reset_d  = 1'b0;
	always @(posedge clk) begin
		reset_d <= reset;
		if (reset && !reset_d && !ss_active) begin
			clr_run  <= 1'b1;
			clr_addr <= 14'd0;
		end else if (!reset || ss_active) begin
			clr_run <= 1'b0;
		end else if (clr_run) begin
			clr_addr <= clr_addr + 14'd1;
			if (clr_addr == 14'h3FFF) clr_run <= 1'b0;
		end
	end

	wire [13:0] a_addr  = clr_run ? clr_addr : ss_active ? ss_mem_addr : addr;
	wire        a_wren  = clr_run ? 1'b1 : ss_active ? ss_mem_wren : (sel && we);
	wire [7:0]  a_wdata = clr_run ? 8'h00 : ss_active ? ss_mem_wdata : wdata;
	wire [7:0]  a_q;

	assign rdata        = a_q;
	assign ss_mem_rdata = a_q;

	cache_ram_dp #(
		.ADDR_WIDTH(14),
		.DATA_WIDTH(8)
	) ram (
		.clk_i(clk),
		.addr_a_i(a_addr),
		.wren_a_i(a_wren),
		.wdata_a_i(a_wdata),
		.q_a_o(a_q),
		.addr_b_i(lcdc_addr),
		.wren_b_i(1'b0),
		.wdata_b_i(8'h00),
		.q_b_o(lcdc_rdata)
	);

endmodule
