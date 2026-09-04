// Copyright (c) 2026 Jamie Blanks
//
// ST2204 internal 512 KB mask ROM interface.
//
// The mask ROM contents (the GameKing BIOS) live in SDRAM, loaded by the
// framework at core start. A read is launched at the start of the 6 MHz bus
// cycle and the SDRAM round-trip completes deterministically within that
// cycle, so from the SoC's side this behaves as the asynchronous mask ROM.
// Keeping the request pacing here leaves st_bus storage-agnostic.

module st_rom
(
	input  wire        clk,
	input  wire        ce,             // 6 MHz bus cycle start
	input  wire        reset,

	input  wire        sel,
	input  wire [18:0] addr,
	output wire [7:0]  rdata,

	// SDRAM backend (BIOS instance port)
	output wire        mem_req,
	output wire [18:0] mem_addr,
	input  wire [7:0]  mem_rdata,
	input  wire        mem_ready
);

	assign rdata = mem_rdata;

	// The request is the live bus decode: address and select are stable
	// for the whole 6 MHz bus cycle, and the top-level staging samples
	// them one controller clock after the cycle-start enable. Registering
	// them here would push the SDRAM read a full bus cycle late, past the
	// enable at which the CPU consumes the data.
	assign mem_req  = sel;
	assign mem_addr = addr;

	wire unused_ok = &{1'b0, clk, ce, reset, mem_ready};

endmodule
