//============================================================================
//
//  GameKing I/II for MiSTer.
//  Copyright (c) 2026 Jamie Blanks
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
//  more details.
//
//============================================================================
//
// Structure (PLAN_gameking_core_20260731.md): this file is MiSTer framework
// glue only - PLL/clock enables, HPS/ioctl ROM loading into SDRAM, video and
// audio output, OSD options. The console lives in rtl/gameking/mainboard.sv.
//
// Savestates: the controller stack lives at the end of this file.

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [45:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler
	output        VGA_DISABLE, // analog out is off

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: USER_OSD + per-pin push-pull mask, USER_IO widened to 8 bits
	output        USER_OSD,
	output  [7:0] USER_PP,
	input   [7:0] USER_IN,
	output  [7:0] USER_OUT,
	// [MiSTer-DB9 END]

	input         OSD_STATUS
);

assign ADC_BUS  = 'Z;
// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: USER_PP driven by wrapper; USER_OUT driven by joydb (USER_OUT_DRIVE) below
assign USER_PP = USER_PP_DRIVE;
// [MiSTer-DB9 END]
assign {UART_RTS, UART_TXD, UART_DTR} = '0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign VGA_F1 = 1'b0;
assign VGA_SCALER = 1'b0;
assign VGA_DISABLE = 1'b0;
assign HDMI_FREEZE = 1'b0;
assign HDMI_BLACKOUT = 1'b0;
assign HDMI_BOB_DEINT = 1'b0;

assign AUDIO_S = 1'b1;
assign AUDIO_MIX = 2'd0;

assign LED_DISK = 1'b0;
assign LED_POWER = 2'd0;
assign BUTTONS = 2'd0;


`include "build_id.v"
localparam CONF_STR = {
	"GameKing;;",
	"F1,BIN,Load Cartridge;",
	"-;",
	"d7O[46:45],Savestate Slot,1,2,3,4;",
	"d7R[42],Save state (Alt-F1);",
	"d7R[43],Restore state (F1);",
	"P1,Video;",
	"P1-;",
	"P1O[3:2],Palette,LCD,Greyscale,Custom;",
	"H0P1F2,GBP,Load Palette;",
	"P1-;",
	"P1O[9:8],Aspect Ratio,Original,Full Screen,16:9;",
	"P1O[19:18],Display Mode,Glass,Scaled,Raw;",
	"P1O[13:11],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"P1O[15:14],Scale,Normal,V-Integer,Integer(Narrow),Integer(Wide);",
	"P2,Advanced;",
	"P2-;",
	"P2O[7],Region Jumper,Other,Chinese;",
	"P2O[17],Disable Auto-Sleep,On,Off;",
	"-;",
	// [MiSTer-DB9-Pro BEGIN] - Saturn-first joy_type (canonical bit notation)
	"O[127:126],UserIO Joystick,Off,Saturn,DB9MD,DB15;",
	// [MiSTer-DB9-Pro END]
	"R[0],Reset;",
	"J1,A,B,Select,Start,Savestates;",
	"jn,A,B,Select,Start,L;",
	// OSD info banner lines, indexed 1-based by savestate_ui's ss_info.
	// The module emits 5 (chord help), 6+slot (slot selected) and
	// 10+{slot,load} (saved / restored), so entries 1-4 are never
	// requested by this core: they are where a rewind build's toasts
	// would live, and rewind is not built. They are present because the
	// index is a position in this list, not a name.
	"I,",
	"-,",
	"-,",
	"-,",
	"-,",
	"Slot=L/R Load=U Save=D,",
	"Active Slot 1,",
	"Active Slot 2,",
	"Active Slot 3,",
	"Active Slot 4,",
	"Save to state 1,",
	"Restore state 1,",
	"Save to state 2,",
	"Restore state 2,",
	"Save to state 3,",
	"Restore state 3,",
	"Save to state 4,",
	"Restore state 4;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire [21:0] gamma_bus;
wire [1:0] buttons;
wire [127:0] status;
wire [31:0] joystick_0;
wire [10:0] ps2_key;

// Declared here because hps_io consumes them while the savestate stack that
// drives them sits at the end of this file.
wire [127:0] status_in;
wire         status_set;
wire         ss_info_req;
wire [7:0]   ss_info;

wire        ioctl_download;
wire [15:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire [7:0]  ioctl_dout;
wire        ioctl_wait;

wire clk_sys;   // 24 MHz console/framework-side fabric clock
wire clk_ram;   // 72 MHz SDRAM controller clock (3x fabric, same PLL)
wire clk_video; // 72 MHz video/render clock, shared PLL output
wire ce_pix;
wire pll_locked;

assign clk_video = clk_ram;

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joydb wrapper
wire         CLK_JOY = CLK_50M;                 // Assign clock between 40-50Mhz
wire   [1:0] joy_type_raw    = status[127:126]; // 0=Off, 1=Saturn, 2=DB9MD, 3=DB15
wire         joy_2p          = 1'b0;          // 1P-only: joy_2p unused
// SNAC cores: replace 1'b0 with the core's SNAC enable expression so SNAC
// preempts the joydb wrapper on shared USER_IO pins. Default 1'b0 is no-op.
wire         snac_active     = 1'b0;
// MT32-pi cores on primary USER_IO: replace 1'b0 with the core's MT32-active
// expression (e.g. `mt32_use` under `ifndef SECOND_MT32`, `~mt32_disable` for
// TRS-80's inverted polarity). Suppresses the OSD-open autodetect probe so it
// doesn't read the RPi's I2C master traffic as a ghost Saturn signature.
wire         mt32_primary_active = 1'b0;
wire   [1:0] joy_type        = snac_active ? 2'd0 : joy_type_raw;
wire         joy_db9md_en    = (joy_type == 2'd2);
wire         joy_db15_en     = (joy_type == 2'd3);
wire         joy_any_en      = |joy_type;
// [MiSTer-DB9 END]

// [MiSTer-DB9-Pro BEGIN] - Saturn key gate
wire         saturn_unlocked;                   // driven by hps_io UIO_DB9_KEY (0xFE)
// [MiSTer-DB9-Pro END]

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joydb wrapper wires + instance
wire   [7:0] USER_OUT_DRIVE;
wire   [7:0] USER_PP_DRIVE;
wire  [15:0] joydb_1, joydb_2;
wire         joydb_1ena, joydb_2ena;
wire  [15:0] joy_raw_payload;
// Programmable-remap matrix (joydb_remap inside joydb): clk_sys carries the
// 0xFD selector load (HPS-bus domain). joydb_*_mapped are the MiSTer-standard
// joystick words the core consumes at its joy0_USB merge point. db9_remap_*
// are driven by the hps_io instance (see the .db9_remap_* bindings there).
// Until a core consumes joydb_*_mapped (Layer B), the matrix stays at identity
// and synthesis prunes it -- binding these is dormant + fleet-safe.
wire  [15:0] joydb_1_mapped, joydb_2_mapped;
wire         db9_remap_cmd;
wire   [5:0] db9_remap_byte_cnt;
wire  [15:0] db9_remap_din;

// [MiSTer-DB9 BEGIN] - DB9 remap factory default (used until Main_MiSTer streams UIO 0xFD)
// Derived from CONF_STR J1, same rule as db9_map.cpp; lets the core work on a stock MiSTer binary.
// DB15:  A=A, B=B, Select=SELECT, Start=START, Savestates=C
// DB9MD: A=A, B=B, Select=MODE, Start=START, Savestates=C
wire  [35:0] db9_remap_default_db15  = 36'hFFFF6AB54;
wire  [35:0] db9_remap_default_db9md = 36'hFFFF6AB54;
// [MiSTer-DB9 END]
joydb joydb (
  .clk             ( CLK_JOY         ),
  .clk_sys         ( clk_sys            ),
  .USER_IN         ( USER_IN         ),
  .OSD_STATUS          ( OSD_STATUS          ),
  .snac_active         ( snac_active         ),
  .mt32_primary_active ( mt32_primary_active ),
  .joy_type        ( joy_type        ),
  .joy_2p          ( joy_2p          ),
  .saturn_unlocked ( saturn_unlocked ),
  .USER_OUT_DRIVE  ( USER_OUT_DRIVE  ),
  .USER_PP_DRIVE   ( USER_PP_DRIVE   ),
  .USER_OSD        ( USER_OSD        ),
  .joydb_1         ( joydb_1         ),
  .joydb_2         ( joydb_2         ),
  .joydb_1ena      ( joydb_1ena      ),
  .joydb_2ena      ( joydb_2ena      ),
  .remap_cmd       ( db9_remap_cmd      ),
  .remap_byte_cnt  ( db9_remap_byte_cnt ),
  .remap_din       ( db9_remap_din      ),
  .remap_default_db15  ( db9_remap_default_db15  ),
  .remap_default_db9md ( db9_remap_default_db9md ),
  .joydb_1_mapped  ( joydb_1_mapped     ),
  .joydb_2_mapped  ( joydb_2_mapped     ),
  .joy_raw         ( joy_raw_payload )
);

assign USER_OUT = USER_OUT_DRIVE;
// [MiSTer-DB9 END]

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: rename USB joystick wires
wire [31:0] joystick_0_USB;
// [MiSTer-DB9 END]

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),
	.forced_scandoubler(forced_scandoubler),
	.buttons(buttons),
	.status(status),
	// Bit 0 hides the custom-palette loader unless "Custom" is picked;
	// bit 7 is the d7 savestate gate on the slot/save/load entries.
	.status_menumask({8'd0, allow_ss, 6'd0, ~pal_custom}),
	.status_in(status_in),
	.status_set(status_set),
	.info_req(ss_info_req),
	.info(ss_info),
	.ps2_key(ps2_key),
	.joystick_0(joystick_0_USB),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),
	// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joy_raw
	.joy_raw(joy_raw_payload),
	// programmable remap matrix selector load (UIO_DB9_MAP 0xFD)
	.db9_remap_cmd(db9_remap_cmd),
	.db9_remap_byte_cnt(db9_remap_byte_cnt),
	.db9_remap_din(db9_remap_din),
	// [MiSTer-DB9 END]
	// [MiSTer-DB9-Pro BEGIN] - Saturn key gate
	.saturn_unlocked(saturn_unlocked)
	// [MiSTer-DB9-Pro END]
);

// [MiSTer-DB9-Pro BEGIN] - DB controllers muted while OSD is open
// joydb_*_mapped is already in MiSTer-standard J1 order, identical to the USB
// joystick word, so it replaces the USB input as-is.
assign joystick_0 = joydb_1ena ? (OSD_STATUS ? 16'b0 : joydb_1_mapped) : joystick_0_USB;
// [MiSTer-DB9-Pro END]

pll pll
(
	.refclk(CLK_50M),
	.rst(1'b0),
	.outclk_0(clk_sys),   // 24 MHz (user clocking direction 2026-07-31)
	.outclk_1(clk_ram),   // 72 MHz for the SDRAM controller
	.locked(pll_locked)
);

// The console must NOT sit in reset during downloads: a hung transfer
// would freeze video and every diagnostic with it. Instead the console
// gets a stretched reset pulse after each ROM download completes.
reg  [19:0] post_dl_reset = 20'hFFFFF;
reg         ioctl_download_d;
// The console must additionally be held from power-up until the FIRST
// BIOS download completes: without this it executes garbage from empty
// SDRAM and scribbles internal RAM, and RAM survives the post-download
// reset - the BIOS then reads a fake warm-boot flag and skips cold init
// (proven in soc_bios_tb +dirtyram: a garbage-filled RAM derails the
// same BIOS that boots cleanly from zeroed RAM).
reg         dl_is_bios;
// The index has to be latched at the START of the transfer: on the falling
// edge of ioctl_download it is no longer valid, so the reset arm below
// cannot ask what kind of file just finished.
reg         dl_is_rom;
reg         bios_loaded_ever = 1'b0;
always @(posedge clk_sys) begin
	ioctl_download_d <= ioctl_download;
	if (ioctl_download && !ioctl_download_d) begin
		dl_is_bios <= (ioctl_index[5:0] == 6'd0);
		dl_is_rom  <= (ioctl_index[5:0] <= 6'd1);
	end
	// Only a ROM download arms the pulse. A palette (.gbp) writes nothing
	// the console can see, so resetting on it would throw away the running
	// game just to change the screen colours.
	if (ioctl_download_d && !ioctl_download && dl_is_rom) begin
		post_dl_reset <= 20'hFFFFF;   // ~44 ms at 24 MHz
		if (dl_is_bios) bios_loaded_ever <= 1'b1;
	end else if (post_dl_reset != 20'd0) begin
		post_dl_reset <= post_dl_reset - 20'd1;
	end
end
// ROM downloads (BIOS index 0, cart index 1) hold reset like VirtualBoy's
// rom_download; palette (index 2) reloads live. The post-download stretch
// also gives the st_ram cold-boot sweep its window.
wire rom_download = ioctl_download && (ioctl_index[5:0] <= 6'd1);
wire reset = RESET | status[0] | buttons[1] | rom_download |
	(post_dl_reset != 20'd0) | ~bios_loaded_ever | ~pll_locked;
wire ss_reset_ss;
wire core_reset = reset | ss_reset_ss;

wire sleep_savestate;

////////////////////////////////////////////////////////////////////////////
// Pad mapping. J1 order is A, B, Select, Start, Savestates, so:
//
//   0..3  Right / Left / Down / Up
//   4     A          5  B          6  Select     7  Start
//   8     Savestates chord (savestate_ui joySS / joySaveState)
//
//   chord + Right / Left   next / previous slot, clamped to 0..3
//   chord + Down / Up      save / load
//   holding the chord      the slot help toast, after the module's timeout
//
// The d-pad is suppressed at the console while the chord is held, otherwise
// the game walks around while the user picks a slot. The chord logic itself
// reads the raw joystick, precisely because it must see the directions the
// console must not.
////////////////////////////////////////////////////////////////////////////
wire joy_ss_chord = joystick_0[8];
wire dpad_en = ~joy_ss_chord;
wire btn_right = joystick_0[0] & dpad_en;
wire btn_left  = joystick_0[1] & dpad_en;
wire btn_down  = joystick_0[2] & dpad_en;
wire btn_up    = joystick_0[3] & dpad_en;

// A savestate is only allowed once the console is actually running (a BIOS
// is loaded, no download in flight, not in reset) and no transfer is already
// under way. Also drives the d7 gate on the OSD savestate entries.
wire ss_busy;
wire allow_ss = ~reset & ~ss_busy;

////////////////////////////////////////////////////////////////////////////
// Clock enables: 6 MHz CPU (phase 2 + mid-cycle phase 1) and 32.768 kHz.
// The independent 72 MHz video domain generates its own 6 MHz pixel enable.
////////////////////////////////////////////////////////////////////////////
reg [1:0]  ce_cnt = 2'd0;
reg        ce_cpu, ce_cpu_n;
reg [26:0] acc_32k = 27'd0;
reg        ce_32k;
reg        mem_wait_s;
always @(posedge clk_sys) mem_wait_s <= fetch_late;

wire sleep_gate = sleep_savestate;
always @(posedge clk_sys) begin
	// The savestate controller freezes the console (not the video raster)
	// by holding the console clock enables while it walks the state.
	// The console enables pause when an SDRAM fetch is running late
	// (refresh collision): the phase counter freezes just before the
	// consuming enable so ce/ce_n alternation is preserved, and resumes
	// once the data has landed. Video free-runs in its own clock domain.
	if (!sleep_gate && !(mem_wait_s && (ce_cnt == 2'd3))) begin
		ce_cnt   <= ce_cnt + 2'd1;
		ce_cpu   <= (ce_cnt == 2'd3);
		ce_cpu_n <= (ce_cnt == 2'd1);

		// 32.768 kHz enable: polynomial (fractional accumulator) divide of
		// the 6 MHz enable grid.
		ce_32k <= 1'b0;
		if (ce_cnt == 2'd3) begin
			if (acc_32k >= 27'd5_967_232) begin   // 6e6 - 32768
				acc_32k <= acc_32k - 27'd5_967_232;
				ce_32k  <= 1'b1;
			end else begin
				acc_32k <= acc_32k + 27'd32_768;
			end
		end
	end else begin
		ce_cpu   <= 1'b0;
		ce_cpu_n <= 1'b0;
		ce_32k   <= 1'b0;
	end

end

////////////////////////////////////////////////////////////////////////////
// SDRAM: port 0 = BIOS (512 KB at 0x000000), port 1 = cart (4 MB window
// at 0x100000). Loaded over ioctl, read by the console at CPU pace.
////////////////////////////////////////////////////////////////////////////
localparam [25:0] BIOS_BASE = 26'h0000000;
localparam [25:0] CART_BASE = 26'h0100000;

// Registered (in the controller clock domain) so the index compare never
// sits in the SDRAM address cone. ioctl signals change on the 24 MHz side
// and are stable for 4+ controller clocks; ioctl_wr is edge-detected.
reg bios_loading, cart_loading, pal_loading;
reg [26:0] loader_addr;
reg [7:0]  loader_data;
reg        loader_wr, ioctl_wr_d;
always @(posedge clk_ram) begin
	bios_loading <= ioctl_download && (ioctl_index[5:0] == 6'd0);
	cart_loading <= ioctl_download && (ioctl_index[5:0] == 6'd1);
	pal_loading  <= ioctl_download && (ioctl_index[5:0] == 6'd2);
	loader_addr  <= ioctl_addr;
	loader_data  <= ioctl_dout;
	ioctl_wr_d   <= ioctl_wr;
	loader_wr    <= ioctl_wr && !ioctl_wr_d;
end

reg cart_present = 1'b0;
always @(posedge clk_sys) begin
	if (cart_loading) cart_present <= 1'b1;
end

// ce_cpu sampled into the controller domain: its rising edge marks the
// start of each CPU bus cycle (used for request strobes and refresh).
reg [1:0] ce_cpu_sync;
// Falling edge of the sampled enable: the console's request/address
// registers clock on the edge that ENDS the enable-high cycle, so the
// cycle boundary (new address valid) is the falling edge here.
wire ce_edge_ram = !ce_cpu_sync[0] && ce_cpu_sync[1];

// Scheduled refresh: one refresh pulse every 32 CPU cycles, placed just
// after the read-return window so it can never collide with a request
// (the deterministic no-stall guarantee). Held off until the controller
// finishes initialization (first busy-low observation).
reg [4:0] rf_cpu_ctr;
reg [4:0] rf_phase;
reg       sdram_refresh;
reg       rf_enable;
always @(posedge clk_ram) begin
	ce_cpu_sync <= {ce_cpu_sync[0], ce_cpu};
	if (!p0_busy) rf_enable <= 1'b1;
	if (~pll_locked) rf_enable <= 1'b0;
	sdram_refresh <= 1'b0;
	if (ce_edge_ram) begin
		rf_phase <= 5'd0;
		rf_cpu_ctr <= rf_cpu_ctr + 5'd1;
	end else if (rf_phase != 5'd31) begin
		rf_phase <= rf_phase + 5'd1;
	end
	if (rf_enable && (rf_cpu_ctr == 5'd0) && (rf_phase == 5'd11)) begin
		sdram_refresh <= 1'b1;
	end
end

wire        bios_mem_req;
wire [18:0] bios_mem_addr;
wire        cart_mem_req;
wire [21:0] cart_mem_addr;

// Read requests: edge-detect the ce-paced request flags and stage the
// addresses through registers so no console-side cone reaches the SDRAM
// controller's accept logic (the 16-clock CPU cycle absorbs the extra
// stage with room to spare).
// NES-controller strobe semantics: hold rd/wr from the pending latch
// until the channel's busy rises (accept), then drop; the request is
// re-armed once per CPU cycle while the console holds its request line.
reg p0_pend, p1_pend, p0_is_wr, p1_is_wr;
reg p0_seen_busy, p1_seen_busy;
reg [25:0] p0_addr_q, p1_addr_q;
always @(posedge clk_ram) begin
	if (bios_loading ? loader_wr : (ce_edge_ram && bios_mem_req)) begin
		p0_pend  <= 1'b1;
		p0_is_wr <= bios_loading;
		p0_addr_q <= BIOS_BASE | (bios_loading ? {7'd0, loader_addr[18:0]}
		                                       : {7'd0, bios_mem_addr});
	end else if (p0_busy) begin
		p0_pend <= 1'b0;
	end
	if (cart_loading ? loader_wr : (ce_edge_ram && cart_mem_req)) begin
		p1_pend  <= 1'b1;
		p1_is_wr <= cart_loading;
		p1_addr_q <= CART_BASE | (cart_loading ? {4'd0, loader_addr[21:0]}
		                                       : {4'd0, cart_mem_addr});
	end else if (p1_busy) begin
		p1_pend <= 1'b0;
	end
	if (~pll_locked) begin
		p0_pend <= 1'b0;
		p1_pend <= 1'b0;
	end
end
// Late-fetch detector: a read is normally accepted (busy rise) one
// controller clock after its pending latch sets. Still pending without
// busy at age 2 means it is queued behind a refresh round and the data
// will miss the consuming enable: freeze the console enables (fetch_late
// -> mem_wait_s) until it lands. Bench-proven in gk_sdram_nes_tb.
reg [2:0] p0_age, p1_age;
reg       p0_late, p1_late;
always @(posedge clk_ram) begin
	if (bios_loading ? loader_wr : (ce_edge_ram && bios_mem_req))
		p0_age <= 3'd0;
	else if (p0_pend && (p0_age != 3'd7))
		p0_age <= p0_age + 3'd1;
	if (p0_pend && !p0_busy && !p0_is_wr && (p0_age >= 3'd2))
		p0_late <= 1'b1;
	else if (!p0_pend && !p0_busy)
		p0_late <= 1'b0;

	if (cart_loading ? loader_wr : (ce_edge_ram && cart_mem_req))
		p1_age <= 3'd0;
	else if (p1_pend && (p1_age != 3'd7))
		p1_age <= p1_age + 3'd1;
	if (p1_pend && !p1_busy && !p1_is_wr && (p1_age >= 3'd2))
		p1_late <= 1'b1;
	else if (!p1_pend && !p1_busy)
		p1_late <= 1'b0;

	if (~pll_locked) begin
		p0_late <= 1'b0;
		p1_late <= 1'b0;
	end
end
wire fetch_late = p0_late || p1_late;

wire p0_rd = p0_pend && !p0_is_wr;
wire p0_wr = p0_pend && p0_is_wr;
wire p1_rd = p1_pend && !p1_is_wr;
wire p1_wr = p1_pend && p1_is_wr;
wire [7:0] p0_byte, p1_byte;
wire p0_busy, p1_busy;

assign ioctl_wait = (bios_loading && (p0_busy || p0_pend)) ||
                    (cart_loading && (p1_busy || p1_pend));

// Refresh request held for a full controller cycle so the idle-state
// sampler cannot miss it
reg [3:0] rf_hold;
always @(posedge clk_ram) begin
	// Held just long enough for one idle-state sample (a longer hold
	// re-triggers a second refresh round and doubles the collision
	// window - seen as rare stale-byte reads in gk_sdram_nes_tb)
	if (sdram_refresh) rf_hold <= 4'h7;
	else if (rf_hold != 4'd0) rf_hold <= rf_hold - 4'd1;
end
wire sdram_refresh_hold = (rf_hold != 4'd0);

sdram_nes sdram (
	.SDRAM_DQ(SDRAM_DQ),
	.SDRAM_A(SDRAM_A),
	.SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH),
	.SDRAM_BA(SDRAM_BA),
	.SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nWE(SDRAM_nWE),
	.SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_CLK(SDRAM_CLK),
	.SDRAM_CKE(SDRAM_CKE),
	.init(~pll_locked),
	.clk(clk_ram),
	.ch0_addr(p0_addr_q[24:0]),
	.ch0_rd(p0_rd),
	.ch0_wr(p0_wr),
	.ch0_din(loader_data),
	.ch0_dout(p0_byte),
	.ch0_busy(p0_busy),
	.ch1_addr(p1_addr_q[24:0]),
	.ch1_rd(p1_rd),
	.ch1_wr(p1_wr),
	.ch1_din(loader_data),
	.ch1_dout(p1_byte),
	.ch1_busy(p1_busy),
	.ch2_addr(25'd0),
	.ch2_rd(1'b0),
	.ch2_wr(1'b0),
	.ch2_din(8'd0),
	.ch2_dout(),
	.ch2_busy(),
	.refresh(sdram_refresh_hold),
	.ss_in(16'd0),
	.ss_load(1'b0),
	.ss_out()
);

// Savestate wires (stack instantiated after the video section)
wire        ssui_save, ssui_load;
wire [1:0]  ss_slot;
wire        ss_req_save, ss_req_load;
integer     ss_req_addr;
wire [63:0] ss_bus_din;
wire [9:0]  ss_bus_addr;
wire        ss_bus_wren;
wire [63:0] ss_bus_dout;
wire [24:0] ssr_addr;
wire        ssr_rden, ssr_wren;
wire [7:0]  ssr_wdata;
wire [7:0]  ssr_rdata;
wire [2:0]  ssr_type;
wire        ssr_type_q = (ssr_type == 3'd1);
wire [63:0] ss_ddr_din, ss_ddr_dout;
wire [25:0] ss_ddr_addr;
wire        ss_ddr_rnw, ss_ddr_req, ss_ddr_ack;
wire [7:0]  ss_ddr_be;

////////////////////////////////////////////////////////////////////////////
// The console
////////////////////////////////////////////////////////////////////////////
wire        power_led;
wire signed [15:0] audio_pcm;
wire [10:0] vid_addr;
wire [3:0]  vid_level;

// OSD "Disable Auto-Sleep" (status[17] = 0 selects On, the default):
// substitute the BIOS idle-sleep bytes as the console reads them, so the
// option can be changed without reloading the image.
wire       bios_patch_en = ~status[17];
wire [7:0] bios_byte;

bios_patch bios_sleep_patch (
	.clk(clk_ram),
	.enable(bios_patch_en),
	.ld_active(bios_loading),
	.ld_wr(loader_wr),
	.ld_addr(loader_addr[18:0]),
	.ld_data(loader_data),
	.rd_req(ce_edge_ram && bios_mem_req && !bios_loading),
	.rd_addr(bios_mem_addr),
	.din(p0_byte),
	.dout(bios_byte)
);

mainboard board (
	.stp_nop(stp_nop),
	.clk(clk_sys),
	.ce_osc(ce_cpu),
	.ce_osc_n(ce_cpu_n),
	.ce_32k(ce_32k),
	.stall(sleep_gate),
	.reset_n(~core_reset),
	.btn_start(joystick_0[7]),
	.btn_select(joystick_0[6]),
	.btn_a(joystick_0[4]),
	.btn_b(joystick_0[5]),
	.btn_up(btn_up),
	.btn_down(btn_down),
	.btn_left(btn_left),
	.btn_right(btn_right),
	.mode_jumper(status[7]),
	.power_led(power_led),
	.cart_present(cart_present),
	.cart_mem_req(cart_mem_req),
	.cart_mem_addr(cart_mem_addr),
	.cart_mem_rdata(p1_byte),
	.cart_mem_ready(!p1_busy),
	.bios_mem_req(bios_mem_req),
	.bios_mem_addr(bios_mem_addr),
	.bios_mem_rdata(bios_byte),
	.bios_mem_ready(!p0_busy),
	.audio_pcm(audio_pcm),
	.vid_addr(vid_addr),
	.vid_level(vid_level),
	.ss_active(sleep_gate),
	.ss_addr(ss_bus_addr),
	.ss_wdata(ss_bus_din),
	.ss_wren(ss_bus_wren),
	.ss_rdata(ss_bus_dout),
	.ss_mem_type(ssr_type_q),
	.ss_mem_addr(ssr_addr[13:0]),
	.ss_mem_wren(ssr_wren),
	.ss_mem_wdata(ssr_wdata),
	.ss_mem_rdata(ssr_rdata)
);

assign LED_USER = power_led;
assign AUDIO_L = audio_pcm;
assign AUDIO_R = audio_pcm;

////////////////////////////////////////////////////////////////////////////
// Video output: 240p 15.7 kHz raster reading the panel frame store.
////////////////////////////////////////////////////////////////////////////
wire [7:0] vr, vg, vb;
wire raw_hblank, raw_hsync, raw_vblank, raw_vsync;
reg hblank, hsync, vblank, vsync;

video_gen video_gen (
	.clk(clk_video),
	.ce_pix(ce_pix),
	.reset(reset),
	.vid_addr(vid_addr),
	.vid_level(vid_level),
	.pal_wr(pal_loading && loader_wr),
	.pal_addr(loader_addr[6:0]),
	.pal_data(loader_data),
	.pal_select(status[3:2]),
	.disp_mode(status[19:18]),
	.r(vr),
	.g(vg),
	.b(vb),
	.hsync(raw_hsync),
	.vsync(raw_vsync),
	.hblank(raw_hblank),
	.vblank(raw_vblank)
);

assign CLK_VIDEO = clk_video;

// The framebuffer finishes colour inside the pixel following the raw raster
// position. Delay sync and blanking by the same pixel enable.
always @(posedge clk_video) begin
	if (reset) begin
		hsync <= 1'b0;
		vsync <= 1'b0;
		hblank <= 1'b1;
		vblank <= 1'b1;
	end
	else if (ce_pix) begin
		hsync <= raw_hsync;
		vsync <= raw_vsync;
		hblank <= raw_hblank;
		vblank <= raw_vblank;
	end
end

// Standard framework video path: mixer (scandoubler, HQ2x, gamma) into
// video_freak (aspect, vertical crop, integer scaling). The view crops to
// the 192-line panel image so HDMI shows it full screen.
// The .gbp load item is hidden unless the Custom palette is selected.
wire       pal_custom = (status[3:2] == 2'd2);
wire [2:0] sd_fx = status[13:11];
wire [1:0] ar_sel = status[9:8];
// The console's idle auto-sleep is a BIOS software timer, not STP (see
// rtl/gameking/bios_patch.sv). STP is never executed by the BIOS, and a
// cart that did execute it would halt forever on a console with no power
// switch, so STP always runs as a NOP here.
wire       stp_nop    = 1'b1;
assign VGA_SL = sd_fx ? sd_fx[1:0] - 2'd1 : 2'd0;

wire vga_de_mixer;
wire freeze_sync;

video_mixer #(
	.LINE_LENGTH(380),
	.HALF_DEPTH(0),
	.GAMMA(1)
) video_mixer (
	.CLK_VIDEO(CLK_VIDEO),
	.CE_PIXEL(CE_PIXEL),
	.ce_pix(ce_pix),
	.scandoubler((sd_fx != 3'd0) || forced_scandoubler),
	.hq2x(sd_fx == 3'd1),
	.gamma_bus(gamma_bus),
	.R(vr),
	.G(vg),
	.B(vb),
	.HSync(hsync),
	.VSync(vsync),
	.HBlank(hblank),
	.VBlank(vblank),
	.HDMI_FREEZE(HDMI_FREEZE),
	.freeze_sync(freeze_sync),
	.VGA_R(VGA_R),
	.VGA_G(VGA_G),
	.VGA_B(VGA_B),
	.VGA_VS(VGA_VS),
	.VGA_HS(VGA_HS),
	.VGA_DE(vga_de_mixer)
);

video_freak video_freak (
	.CLK_VIDEO(CLK_VIDEO),
	.CE_PIXEL(CE_PIXEL),
	.VGA_VS(VGA_VS),
	.HDMI_WIDTH(HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE(VGA_DE),
	.VIDEO_ARX(VIDEO_ARX),
	.VIDEO_ARY(VIDEO_ARY),
	.VGA_DE_IN(vga_de_mixer),
	.ARX((ar_sel == 2'd0) ? 12'd3 : (ar_sel == 2'd2) ? 12'd16 : 12'd0),
	.ARY((ar_sel == 2'd0) ? 12'd2 : (ar_sel == 2'd2) ? 12'd9  : 12'd0),
	.CROP_SIZE(12'd0),
	.CROP_OFF(5'd0),
	.SCALE({1'b0, status[15:14]})
);

////////////////////////////////////////////////////////////////////////////
// Savestates: controller stack from VirtualBoy_MiSTer. State layout:
// 2 header words + 520 internal words (the SoC savestate register bus),
// savetype 0 = internal SRAM (10240 bytes), savetype 1 = panel frame
// store (1536 bytes). Total payload 3988 dwords; slots stride 1<<15 in
// the DDR3 scratch window via ddram.v.
////////////////////////////////////////////////////////////////////////////
// BRAM read latency adapter: ready two clocks after an access strobe
reg [1:0] ssr_pipe;
always @(posedge clk_sys) ssr_pipe <= {ssr_pipe[0], ssr_rden | ssr_wren};
wire ssr_ready = ssr_pipe[1];

// Pause acknowledge: the console is asleep a few clocks after the enables
// freeze (longest in-flight SDRAM read is well inside the settle count)
reg [3:0] ss_paused_pipe;
always @(posedge clk_sys)
	ss_paused_pipe <= {ss_paused_pipe[2:0], sleep_savestate};
wire ss_paused = ss_paused_pipe[3];

// savestate_ui owns the slot number once the user changes it with the pad
// chord or an F-key, and writes it back into status[46:45] so the OSD entry
// agrees with what the next save or load will use.
wire ss_status_update;
assign status_in = {status[127:47], ss_slot, status[44:0]};
assign status_set = ss_status_update;

// Driving BOTH joySS and joySaveState from the one Savestates button
// collapses the module's `(joySaveState || (joySS && joyStart))` condition to
// the button itself, so chord+Down saves and chord+Up loads with no Start in
// the picture. joyStart is therefore tied low rather than borrowed: Start is a
// real console button and a game is entitled to it.
//
// The DIRECTIONS come from joystick_0 raw, not from the btn_* nets - those
// have the d-pad suppressed while the chord is held.
savestate_ui #(
	.INFO_TIMEOUT_BITS(25)
) savestate_ui (
	.clk(clk_sys),
	.ps2_key(ps2_key),
	.allow_ss(allow_ss),
	.joySS(joy_ss_chord),
	.joyRight(joystick_0[0]),
	.joyLeft(joystick_0[1]),
	.joyDown(joystick_0[2]),
	.joyUp(joystick_0[3]),
	.joyStart(1'b0),
	.joySaveState(joy_ss_chord),
	.status_slot(status[46:45]),
	.OSD_saveload(status[43:42]),
	.ss_save(ssui_save),
	.ss_load(ssui_load),
	.ss_info_req(ss_info_req),
	.ss_info(ss_info),
	.statusUpdate(ss_status_update),
	.selected_slot(ss_slot)
);

statemanager #(
	.Softmap_SaveState_ADDR(0),
	.Softmap_Rewind_ADDR(0),
	.SAVESTATE_SHIFT(15)
) statemanager (
	.clk(clk_sys),
	.reset(reset),
	.rewind_on(1'b0),
	.rewind_active(1'b0),
	.savestate_number({30'd0, ss_slot}),
	.save(ssui_save),
	.load(ssui_load),
	.sleep_rewind(),
	.vsync(vsync),
	.request_savestate(ss_req_save),
	.request_loadstate(ss_req_load),
	.request_address(ss_req_addr),
	.request_busy(ss_busy)
);

savestates #(
	.STATESIZE_PARAM(3988),
	.SETTLECOUNT_PARAM(16),
	.INTERNALSCOUNT_PARAM(520),
	.SAVETYPESCOUNT_PARAM(2),
	.SAVETYPE0_SIZE(10240),
	.SAVETYPE1_SIZE(1536),
	.SAVETYPE2_SIZE(0),
	.SAVETYPE3_SIZE(0),
	.SAVETYPE4_SIZE(0),
	.SAVETYPE5_SIZE(0)
) savestates (
	.clk(clk_sys),
	.reset_in(reset),
	.reset_ss(ss_reset_ss),
	.reset_delay(),
	.restore_begin(),
	.load_done(),
	.save(ss_req_save),
	.load(ss_req_load),
	.state_size_i(32'd3988),
	.savetype3_size_i(25'd0),
	.savestate_address(ss_req_addr),
	.savestate_busy(ss_busy),
	.paused(ss_paused),
	.BUS_Din(ss_bus_din),
	.BUS_Adr(ss_bus_addr),
	.BUS_wren(ss_bus_wren),
	.BUS_rst(),
	.BUS_Dout(ss_bus_dout),
	.loading_savestate(),
	.saving_savestate(),
	.sleep_savestate(sleep_savestate),
	.Save_RAMAddr(ssr_addr),
	.Save_RAMRdEn(ssr_rden),
	.Save_RAMWrEn(ssr_wren),
	.Save_RAMWriteData(ssr_wdata),
	.Save_RAMReadData(ssr_rdata),
	.Save_RAMReady(ssr_ready),
	.Save_RAMType(ssr_type),
	.bus_out_Din(ss_ddr_din),
	.bus_out_Dout(ss_ddr_dout),
	.bus_out_Adr(ss_ddr_addr),
	.bus_out_rnw(ss_ddr_rnw),
	.bus_out_ena(ss_ddr_req),
	.bus_out_be(ss_ddr_be),
	.bus_out_done(ss_ddr_ack)
);

assign DDRAM_CLK = clk_sys;
ddram ddram (
	.DDRAM_CLK(clk_sys),
	.DDRAM_BUSY(DDRAM_BUSY),
	.DDRAM_BURSTCNT(DDRAM_BURSTCNT),
	.DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DOUT(DDRAM_DOUT),
	.DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.DDRAM_RD(DDRAM_RD),
	.DDRAM_DIN(DDRAM_DIN),
	.DDRAM_BE(DDRAM_BE),
	.DDRAM_WE(DDRAM_WE),
	// bus_out_Adr counts 32-bit dwords (it steps by 2 per 64-bit word);
	// ddram's ch_addr is a 16-bit-word address whose [27:3] slice picks the
	// 64-bit beat. Without the shift, ss_ddr_addr[25:2] selects the beat and
	// every PAIR of consecutive state words lands on the SAME DDR beat: the
	// header still reads back with the right size, so a load proceeds and
	// then restores half the machine from the wrong words. Shift as
	// VirtualBoy_MiSTer does ({savestate_ddr_addr, 1'b0}).
	.ch1_addr({ss_ddr_addr, 1'b0}),
	.ch1_dout(ss_ddr_dout),
	.ch1_din(ss_ddr_din),
	.ch1_req(ss_ddr_req),
	.ch1_rnw(ss_ddr_rnw),
	.ch1_be(ss_ddr_be),
	.ch1_ready(ss_ddr_ack),
	.ch2_addr(27'd0),
	.ch2_dout(),
	.ch2_din(64'd0),
	.ch2_req(1'b0),
	.ch2_rnw(1'b1),
	.ch2_be(8'd0),
	.ch2_ready()
);


endmodule
