// Copyright (c) 2026 Jamie Blanks
//
// ST2204 system control: SYS ($30), MISC ($38) and the inert latches at
// $39-$3B.
//
// SYS bits: 0 = LVDEN (reads back the low-voltage detect result), 1 = IRREN
// (the program-window IRR override during interrupt service), 2 = WAIT mode
// select. MISC: bit 4 = LVDS, bit 3 = WDTEN, bit 2 = WDTPS, and any write
// kicks the watchdog.

module st_sys
#(
	parameter [9:0] SS_BASE = 10'd0
)
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	input  wire        sel,
	input  wire [6:0]  reg_addr,
	input  wire        reg_we,
	input  wire [7:0]  reg_wdata,
	output reg  [7:0]  reg_rdata,

	output wire        irren,          // SYS.1 to st_bus
	output wire        wait_mode,      // SYS.2
	output wire        wdt_en,         // MISC.3
	output wire        wdt_ps,         // MISC.2
	output reg         wdt_kick,       // pulses on any MISC write

	input  wire [9:0]  ss_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata
);

	reg [7:0] sys;          // $30
	reg [4:0] misc;         // $38 (mask $1F)
	reg [3:0] r39;          // $39 low nibble writable
	reg [7:0] r3a;          // $3A fully writable inert latch

	assign irren     = sys[1];
	assign wait_mode = sys[2];
	assign wdt_en    = misc[3];
	assign wdt_ps    = misc[2];

	always @(posedge clk) begin
		if (reset) begin
			sys      <= 8'h00;
			misc     <= 5'h00;
			r39      <= 4'h0;
			r3a      <= 8'hFF;
			wdt_kick <= 1'b0;
		end else if (ce) begin
			wdt_kick <= 1'b0;
			if (sel && reg_we) begin
				case (reg_addr[3:0])
					4'h0: sys <= reg_wdata;                 // $30
					4'h8: begin                             // $38
						misc     <= reg_wdata[4:0];
						wdt_kick <= 1'b1;
					end
					4'h9: r39 <= reg_wdata[3:0];            // $39
					4'hA: r3a <= reg_wdata;                 // $3A
					default: ;
				endcase
			end
		end
		// Savestate restore (overrides normal logic while asleep)
		if (ss_wren && (ss_addr == SS_BASE)) begin
			sys  <= ss_wdata[24:17];
			misc <= ss_wdata[16:12];
			r39  <= ss_wdata[11:8];
			r3a  <= ss_wdata[7:0];
		end
	end

	always @* begin
		case (reg_addr[3:0])
			// LVD result in bit 0. The battery is always good here, and
			// the bit reads 1 out of reset before LVDEN is ever written,
			// so it reads 1 regardless of LVDEN.
			4'h0: reg_rdata = {sys[7:1], 1'b1};
			4'h8: reg_rdata = {3'b111, misc};
			4'h9: reg_rdata = {4'hF, r39};
			4'hA: reg_rdata = r3a;
			default: reg_rdata = 8'hFF;                     // $3B reads 1s
		endcase
	end

	// Savestate word: [24:17]=SYS [16:12]=MISC [11:8]=$39 [7:0]=$3A
	assign ss_rdata = (ss_addr == SS_BASE)
		? {39'd0, sys, misc, r39, r3a} : 64'd0;

	wire unused_ok = &{1'b0, reg_addr[6:4], ss_wdata[63:25]};

endmodule
