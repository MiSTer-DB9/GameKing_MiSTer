// Copyright (c) 2026 Jamie Blanks
//
// GameKing I/II mainboard.
//
// Holds the ST2204 SoC, the cartridge slot, the LCD panel and the speaker,
// wired by their real nets: buttons on PA0/PA1 and PB2-7 (active low,
// switching to ground against the port pull-ups), the power LED on PC6, the
// JP4 mode jumper on PC7, the link jack on PD6/PD7, the speaker on
// PSGO/PSGOB, and the LCD on the PL pads plus /POFF. The 6 MHz and
// 32.768 kHz crystals arrive as clock enables.

module mainboard
(
	input  wire        clk,
	input  wire        ce_osc,         // 6 MHz crystal (phase 2)
	input  wire        ce_osc_n,       // phase 1
	input  wire        ce_32k,         // 32.768 kHz crystal
	input  wire        stall,          // savestate sleep

	input  wire        reset_n,        // RC power-on + reset button + OSD

	// Controls (pressed = 1 from the framework; board inverts to
	// switch-to-ground on the port nets)
	input  wire        btn_start,
	input  wire        btn_select,
	input  wire        btn_a,
	input  wire        btn_b,
	input  wire        btn_up,
	input  wire        btn_down,
	input  wire        btn_left,
	input  wire        btn_right,
	input  wire        mode_jumper,    // 1 = JP4 soldered (pulls PC7 low)
	input  wire        stp_nop,        // execute STP as NOP (OSD option)

	output wire        power_led,      // PC6

	// Cartridge backend
	input  wire        cart_present,
	output wire        cart_mem_req,
	output wire [21:0] cart_mem_addr,
	input  wire [7:0]  cart_mem_rdata,
	input  wire        cart_mem_ready,

	// BIOS (internal mask ROM) backend
	output wire        bios_mem_req,
	output wire [18:0] bios_mem_addr,
	input  wire [7:0]  bios_mem_rdata,
	input  wire        bios_mem_ready,

	// Audio
	output wire signed [15:0] audio_pcm,

	// Video read port (from video_gen)
	input  wire [10:0] vid_addr,
	output wire [3:0]  vid_level,

	// Savestate spine
	input  wire        ss_active,
	input  wire [9:0]  ss_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata,
	input  wire        ss_mem_type,    // 0 = internal SRAM, 1 = panel store
	input  wire [13:0] ss_mem_addr,
	input  wire        ss_mem_wren,
	input  wire [7:0]  ss_mem_wdata,
	output wire [7:0]  ss_mem_rdata
);

	// -------------------------------------------------------------------
	// Port nets: reading a pin returns the driven level, or the SoC's weak
	// pull-up (1) when nothing drives it. Buttons switch the net to ground.
	// -------------------------------------------------------------------
	wire [7:0] pa_dout, pa_oe, pb_dout, pb_oe, pc_dout, pc_oe;
	wire [7:0] pd_dout, pd_oe, pe_dout, pe_oe, pl_dout, pl_oe;

	// Port A: bit 0 = Select, bit 1 = Start; bits 4-7 are cart fingers.
	// The 4-in-1 multicart menus confirm on SELECT and their key handler
	// only tests bits 5:4 and 0, so Select on the wrong pin leaves every
	// multicart menu unable to launch a game.
	wire [7:0] pa_ext_drive_n = {4'hF, 2'b11, ~btn_start, ~btn_select};
	wire [7:0] pa_din;
	// Port B: bit 2 = A, 3 = B, 4-7 = Up/Down/Left/Right; bits 1:0 are
	// unconnected and always read high.
	wire [7:0] pb_ext_drive_n = {~btn_right, ~btn_left, ~btn_down, ~btn_up,
	                             ~btn_b, ~btn_a, 2'b11};
	wire [7:0] pb_din;
	// Port C: bits 0-3 cart GPIO (open), bit 6 LED (SoC output), bit 7 JP4.
	wire [7:0] pc_ext_drive_n = {~mode_jumper, 7'h7F};
	wire [7:0] pc_din;
	// Port D: /CS and UART1 alternates; nothing external drives it.
	wire [7:0] pd_din;
	// Port E: cart GPIO fingers (open).
	wire [7:0] pe_din;
	wire [7:0] pl_din;

	// Net resolution: SoC driver wins where enabled; else external
	// switch-to-ground; else pull-up 1.
	assign pa_din = (pa_oe & pa_dout) | (~pa_oe & pa_ext_drive_n);
	assign pb_din = (pb_oe & pb_dout) | (~pb_oe & pb_ext_drive_n);
	assign pc_din = (pc_oe & pc_dout) | (~pc_oe & pc_ext_drive_n);
	assign pd_din = (pd_oe & pd_dout) | ~pd_oe;
	assign pe_din = (pe_oe & pe_dout) | ~pe_oe;
	assign pl_din = (pl_oe & pl_dout) | ~pl_oe;

	assign power_led = pc_oe[6] & pc_dout[6];

	// -------------------------------------------------------------------
	// External bus and cartridge
	// -------------------------------------------------------------------
	wire [22:0] ext_a;
	wire [7:0]  ext_d_dout;
	wire        ext_d_oe;
	wire        rd_n, wr_n, cs0_n;
	wire [7:0]  cart_d_dout;
	wire        cart_d_oe;
	// Bus resolution: the cart drives when enabled, else the bus idles high.
	wire [7:0]  ext_d_din = cart_d_oe ? cart_d_dout : 8'hFF;

	// -------------------------------------------------------------------
	// LCD and audio nets
	// -------------------------------------------------------------------
	wire poff_n, blank_n, load2;
	wire psgo, psgob;
	wire [7:0] sram_ss_rdata, panel_ss_rdata;
	assign ss_mem_rdata = ss_mem_type ? panel_ss_rdata : sram_ss_rdata;

	st2204 soc (
		.stp_nop(stp_nop),
		.clk(clk),
		.ce_osc(ce_osc),
		.ce_osc_n(ce_osc_n),
		.ce_32k(ce_32k),
		.stall(stall),
		.reset_n(reset_n),
		.pa_din(pa_din),
		.pa_dout(pa_dout),
		.pa_oe(pa_oe),
		.pb_din(pb_din),
		.pb_dout(pb_dout),
		.pb_oe(pb_oe),
		.pc_din(pc_din),
		.pc_dout(pc_dout),
		.pc_oe(pc_oe),
		.pd_din(pd_din),
		.pd_dout(pd_dout),
		.pd_oe(pd_oe),
		.pe_din(pe_din),
		.pe_dout(pe_dout),
		.pe_oe(pe_oe),
		.pl_din(pl_din),
		.pl_dout(pl_dout),
		.pl_oe(pl_oe),
		.ext_a(ext_a),
		.ext_d_din(ext_d_din),
		.ext_d_dout(ext_d_dout),
		.ext_d_oe(ext_d_oe),
		.rd_n(rd_n),
		.wr_n(wr_n),
		.cs0_n(cs0_n),
		.psgo(psgo),
		.psgob(psgob),
		.poff_n(poff_n),
		.blank_n(blank_n),
		.load2(load2),
		.irom_req(bios_mem_req),
		.irom_addr_o(bios_mem_addr),
		.irom_data(bios_mem_rdata),
		.irom_ready(bios_mem_ready),
		.ss_active(ss_active),
		.ss_addr(ss_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wren),
		.ss_rdata(ss_rdata),
		.ss_mem_addr(ss_mem_addr),
		.ss_mem_wren(ss_mem_wren && !ss_mem_type),
		.ss_mem_wdata(ss_mem_wdata),
		.ss_mem_rdata(sram_ss_rdata)
	);

	cartridge cart (
		.clk(clk),
		.ce(ce_osc),
		.reset(~reset_n),
		.a(ext_a[21:0]),
		.a22(ext_a[22]),
		.d_din(ext_d_oe ? ext_d_dout : 8'hFF),
		.d_dout(cart_d_dout),
		.d_oe(cart_d_oe),
		.ce0_n(cs0_n),
		.ce2_n(1'b1),
		.ce3_n(1'b1),
		.ce4_n(1'b1),
		.rd_n(rd_n),
		.wr_n(wr_n),
		.rst_drive_n(),
		.cart_present(cart_present),
		.mem_req(cart_mem_req),
		.mem_addr(cart_mem_addr),
		.mem_rdata(cart_mem_rdata),
		.mem_ready(cart_mem_ready)
	);

	// LCD: PL7=FLM, PL6=LOAD1, PL5=/AC, PL4=CP, PL3-0=LD (board doc).
	lcd_panel panel (
		.clk(clk),
		.ce(ce_osc),
		.reset(~reset_n),
		.pin_flm(pl_dout[7]),
		.pin_load1(pl_dout[6]),
		.pin_ac_n(pl_dout[5]),
		.pin_cp(pl_dout[4]),
		.pin_ld(pl_dout[3:0]),
		.pin_poff_n(poff_n),
		.vid_addr(vid_addr),
		.vid_level(vid_level),
		.ss_active(ss_active && ss_mem_type),
		.ss_mem_addr(ss_mem_addr[10:0]),
		.ss_mem_wren(ss_mem_wren && ss_mem_type),
		.ss_mem_wdata(ss_mem_wdata),
		.ss_mem_rdata(panel_ss_rdata)
	);

	speaker spk (
		.clk(clk),
		.ce(ce_osc),
		.reset(~reset_n),
		.psgo(psgo),
		.psgob(psgob),
		.pcm(audio_pcm)
	);

	wire unused_ok = &{1'b0, blank_n, load2};

endmodule
