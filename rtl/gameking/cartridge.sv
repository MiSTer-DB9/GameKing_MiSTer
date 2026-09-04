// Copyright (c) 2026 Jamie Blanks
//
// GameKing cartridge slot.
//
// The populated cartridge is a byte-wide mask ROM enabled by /CE0 alone. The
// /RD, /WR, /CE2/3/4 and "/CE8" (A22) lines and the 16 GPIO fingers are all
// present on the connector, but no known cart uses them. The ROM image is
// backed by its own SDRAM instance; the read launches at the bus-cycle start
// and returns within the cycle, matching the era's asynchronous mask ROM.

module cartridge
(
	input  wire        clk,
	input  wire        ce,             // 6 MHz bus cycle start
	input  wire        reset,

	// Connector (from the mainboard/SoC side)
	input  wire [21:0] a,              // A21-A0
	input  wire        a22,            // "/CE8" finger, cart pin 48
	input  wire [7:0]  d_din,          // data driven by the console
	output wire [7:0]  d_dout,         // data driven by the cart ROM
	output wire        d_oe,
	input  wire        ce0_n,          // cart pin 18 -> ROM /CE
	input  wire        ce2_n,
	input  wire        ce3_n,
	input  wire        ce4_n,
	input  wire        rd_n,
	input  wire        wr_n,
	output wire        rst_drive_n,    // cart pin 15 can pull /RESET low (unused)

	input  wire        cart_present,   // a cart image is loaded

	// SDRAM backend (cart instance port)
	output wire        mem_req,
	output wire [21:0] mem_addr,
	input  wire [7:0]  mem_rdata,
	input  wire        mem_ready
);

	// Mask ROM behavior: drive data while /CE0 is low and a cart is loaded.
	assign d_dout      = mem_rdata;
	assign d_oe        = cart_present && !ce0_n;
	assign rst_drive_n = 1'b1;

	// Live request, same contract as st_rom: the connector address is
	// stable for the whole bus cycle and the top-level staging samples it
	// just after the cycle-start enable.
	assign mem_req  = cart_present && !ce0_n;
	assign mem_addr = a;

	wire unused_ok = &{1'b0, clk, ce, reset, a22, d_din, ce2_n, ce3_n,
		ce4_n, rd_n, wr_n, mem_ready};

endmodule
