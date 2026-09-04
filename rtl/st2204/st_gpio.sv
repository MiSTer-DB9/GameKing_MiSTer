// Copyright (c) 2026 Jamie Blanks
//
// ST2204 GPIO ports A-E ($00-$0F). Port L is muxed against the LCDC at the
// SoC top level.
//
// Port model: the data register reads the instantaneous pin level and the
// control (direction) register enables the output driver. An input with
// data = 1 carries the ~200 uA weak pull-up, so the pin reads 1 unless
// something drives it low; an input with data = 0 floats and reads the
// external level, which is 1 on an undriven net.

module st_gpio
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
	input  wire        reg_re,
	input  wire [7:0]  reg_wdata,
	output reg  [7:0]  reg_rdata,

	// Pads (DIN = level at pin, DOUT/OE = driver)
	input  wire [7:0]  pa_din,
	output wire [7:0]  pa_dout,
	output wire [7:0]  pa_oe,
	input  wire [7:0]  pb_din,
	output wire [7:0]  pb_dout,
	output wire [7:0]  pb_oe,
	input  wire [7:0]  pc_din,
	output wire [7:0]  pc_dout,
	output wire [7:0]  pc_oe,
	input  wire [7:0]  pd_din,
	output wire [7:0]  pd_dout,
	output wire [7:0]  pd_oe,
	input  wire [7:0]  pe_din,
	output wire [7:0]  pe_dout,
	output wire [7:0]  pe_oe,

	// Port D alternate function: chip-select outputs from st_bus when PFD set
	input  wire [6:1]  cs_n,           // /CS1../CS6 onto PD0..PD5

	output reg         pa_transition,  // st_irq level 4

	input  wire [9:0]  ss_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata
);

	// Data, direction, function registers
	reg [7:0] pdata [0:4];   // PA..PE output latches
	reg [7:0] pctrl [0:4];   // PCA..PCE direction
	reg [7:0] pa_last;       // transition detector history
	reg       pa_armed;
	reg [7:0] psc;           // $05 port C special function select
	reg [7:0] pfc;           // $0D
	reg [7:0] pfd;           // $0E
	reg [7:0] pmcr;          // $0F

	// Pin drivers: output enable follows direction; pull-up modeling is done
	// by the reader (din returns 1 when nothing drives the net).
	assign pa_dout = pdata[0];
	assign pa_oe   = pctrl[0];
	assign pb_dout = pdata[1];
	assign pb_oe   = pctrl[1];
	assign pc_dout = pdata[2];
	assign pc_oe   = pctrl[2];
	// PD carries /CS1-6 on bits 5:0, per bit, where PFD enables them; the
	// remaining bits keep their GPIO latch. PFD gates the data mux and the
	// output enable identically.
	wire [7:0] pd_alt = {2'b00, pfd[5:0]};
	assign pd_dout = (pdata[3] & ~pd_alt) | ({2'b00, cs_n[6:1]} & pd_alt);
	assign pd_oe   = pctrl[3] | pd_alt;
	assign pe_dout = pdata[4];
	assign pe_oe   = pctrl[4];

	wire [7:0] pin_din [0:4];
	assign pin_din[0] = pa_din;
	assign pin_din[1] = pb_din;
	assign pin_din[2] = pc_din;
	assign pin_din[3] = pd_din;
	assign pin_din[4] = pe_din;

	always @(posedge clk) begin
		if (reset) begin
			pdata[0] <= 8'hFF;
			pdata[1] <= 8'hFF;
			pdata[2] <= 8'hFF;
			pdata[3] <= 8'hFF;
			pdata[4] <= 8'hFF;
			pctrl[0] <= 8'h00;
			pctrl[1] <= 8'h00;
			pctrl[2] <= 8'h00;
			pctrl[3] <= 8'h00;
			pctrl[4] <= 8'h00;
			psc      <= 8'hFF;
			pfc      <= 8'h00;
			pfd      <= 8'h00;
			pmcr     <= 8'h80;
			pa_last       <= 8'hFF;   // pins idle high through the pull-ups
			pa_armed      <= 1'b0;    // armed by the first read of PA
			pa_transition <= 1'b0;
		end else if (ce) begin
			// Port A transition detector: any input edge fires the level-4
			// interrupt once, and reading PA ($00) re-arms it.
			pa_transition <= 1'b0;
			pa_last <= pa_din;
			if (pa_armed && (pa_din != pa_last)) begin
				pa_transition <= 1'b1;
				pa_armed      <= 1'b0;
			end
			if (sel && reg_re && (reg_addr[3:0] == 4'h0)) begin
				pa_armed <= 1'b1;
			end
			if (sel && reg_we) begin
				case (reg_addr[3:0])
					4'h0: pdata[0] <= reg_wdata;
					4'h1: pdata[1] <= reg_wdata;
					4'h2: pdata[2] <= reg_wdata;
					4'h3: pdata[3] <= reg_wdata;
					4'h4: pdata[4] <= reg_wdata;
					4'h5: psc      <= reg_wdata;
					4'h8: pctrl[0] <= reg_wdata;
					4'h9: pctrl[1] <= reg_wdata;
					4'hA: pctrl[2] <= reg_wdata;
					4'hB: pctrl[3] <= reg_wdata;
					4'hC: pctrl[4] <= reg_wdata;
					4'hD: pfc      <= reg_wdata;
					4'hE: pfd      <= reg_wdata;
					4'hF: pmcr     <= reg_wdata;
					default: ;
				endcase
			end
		end
		// Savestate restore
		if (ss_wren && (ss_addr == SS_BASE)) begin
			pdata[0] <= ss_wdata[63:56];
			pdata[1] <= ss_wdata[55:48];
			pdata[2] <= ss_wdata[47:40];
			pdata[3] <= ss_wdata[39:32];
			pdata[4] <= ss_wdata[31:24];
			pctrl[0] <= ss_wdata[23:16];
			pctrl[1] <= ss_wdata[15:8];
			pctrl[2] <= ss_wdata[7:0];
		end
		if (ss_wren && (ss_addr == SS_BASE + 10'd1)) begin
			pctrl[3]      <= ss_wdata[57:50];
			pctrl[4]      <= ss_wdata[49:42];
			psc           <= ss_wdata[41:34];
			pfc           <= ss_wdata[33:26];
			pfd           <= ss_wdata[25:18];
			pmcr          <= ss_wdata[17:10];
			pa_last       <= ss_wdata[9:2];
			pa_armed      <= ss_wdata[1];
			pa_transition <= ss_wdata[0];
		end
	end

	// Data registers read the pin level; others read their latches.
	always @* begin
		case (reg_addr[3:0])
			4'h0: reg_rdata = pin_din[0];
			4'h1: reg_rdata = pin_din[1];
			4'h2: reg_rdata = pin_din[2];
			4'h3: reg_rdata = pin_din[3];
			4'h4: reg_rdata = pin_din[4];
			4'h5: reg_rdata = psc;
			4'h8: reg_rdata = pctrl[0];
			4'h9: reg_rdata = pctrl[1];
			4'hA: reg_rdata = pctrl[2];
			4'hB: reg_rdata = pctrl[3];
			4'hC: reg_rdata = pctrl[4];
			4'hD: reg_rdata = pfc;
			4'hE: reg_rdata = pfd;
			4'hF: reg_rdata = pmcr;
			default: reg_rdata = 8'hFF;   // $06/$07 unmapped
		endcase
	end

	// Savestate words: 0 = {pdata[0..4], pctrl[0..2]};
	// 1 = {pctrl[3..4], psc, pfc, pfd, pmcr, pa_last, pa_armed, pa_transition}
	assign ss_rdata =
		(ss_addr == SS_BASE) ?
			{pdata[0], pdata[1], pdata[2], pdata[3], pdata[4],
			 pctrl[0], pctrl[1], pctrl[2]} :
		(ss_addr == SS_BASE + 10'd1) ?
			{6'd0, pctrl[3], pctrl[4], psc, pfc, pfd, pmcr, pa_last,
			 pa_armed, pa_transition} : 64'd0;

	wire unused_ok = &{1'b0, reg_addr[6:4]};

endmodule
