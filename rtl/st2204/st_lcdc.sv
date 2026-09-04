// Copyright (c) 2026 Jamie Blanks
//
// ST2204 LCD controller ($40-$4F). Scan engine:
//
// - Pixel clock = system clock / (1,2,4,6) per LCKR[3:2] (GameKing: /6 =
//   1 MHz).
// - A line is LXMAX*8 active pixels fetched from internal RAM followed by
//   an (LFRA+1)*8 pixel-clock extension (GameKing: 48 + 208 = 256 pixel
//   clocks -> 256 us lines, 32 lines, ~122 Hz refresh, matching the
//   observed "120 Hz or so").
// - LYMAX*2 lines per refresh, with 0 meaning 512. The panel scans
//   bottom-up; memory is read sequentially from LSSA with a line stride
//   of LVPW*4 bytes (LVPW*16 pixels at 2 bpp).
// - 2 bpp packs 4 pixels/byte, bits 7:6 leftmost. 1 bpp per GL0=0.
// - 4-level gray by three-scan modulation: a pixel of level L is driven
//   ON during (L>=3, L>=1, L>=2) on scans 1/2/3 - equivalently the number
//   of ON scans equals L. REV (LCTR bit 5) inverts the level first.
// - Pins: CP pulses once per 4-pixel group with LD3-0 carrying the group
//   (LD3 = leftmost); LOAD1 pulses at the line boundary for LPWM*4+16
//   pixel clocks; FLM asserts after the last line's pulse until the next
//   line pulse; /AC toggles per LAC (frame-aligned when 0).
// - The frame interrupt (level 6) fires once per refresh.
// - fetch_req asserts on framebuffer read cycles to drive the LCDC-priority
//   CPU stall. The dual-port RAM serves both sides, so the stall only shapes
//   timing.

module st_lcdc
#(
	parameter [9:0] SS_BASE = 10'd0
)
(
	input  wire        clk,
	input  wire        ce,             // 6 MHz
	input  wire        reset,

	input  wire        sel,
	input  wire [6:0]  reg_addr,
	input  wire        reg_we,
	input  wire [7:0]  reg_wdata,
	output reg  [7:0]  reg_rdata,

	// Framebuffer fetch port (internal RAM port B)
	output reg         fetch_req,
	output reg  [15:0] fetch_addr,
	input  wire [7:0]  fetch_data,

	// LCD pins (multiplexed onto PL when running)
	output reg         lcd_active,
	output reg         pin_flm,
	output reg         pin_load1,
	output reg         pin_ac_n,
	output reg         pin_cp,
	output reg  [3:0]  pin_ld,
	output reg         pin_poff_n,
	output reg         pin_blank_n,
	output reg         pin_load2,

	output reg         int_lcd,

	input  wire [9:0]  ss_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata
);

	// Registers
	reg [15:0] lssa;
	reg [7:0]  lvpw;
	reg [7:0]  lxmax;
	reg [7:0]  lymax;
	reg [2:0]  lpan;
	reg [7:0]  lar;
	reg [7:0]  lctr;
	reg [7:0]  lckr;
	reg [5:0]  lfra;
	reg [4:0]  lac;
	reg [5:0]  lpwm;
	reg [7:0]  lmisc;
	reg [7:0]  lena;

	// Scan state
	reg [2:0]  pixdiv;
	reg [10:0] col;          // pixel-clock column within the line
	reg [9:0]  line;
	reg [1:0]  scan_phase;   // 0..2 three-scan gray cycle
	reg [15:0] line_base;    // fetch address of the current line
	reg [15:0] byte_ptr;     // running fetch pointer within the line
	reg [7:0]  pix_byte;     // latched framebuffer byte
	reg        ac_state;

	wire       bpp2      = lctr[2];
	wire       rev       = lctr[5];
	wire       running   = ~lctr[7];
	wire [10:0] active_px = {lxmax, 3'd0};            // LXMAX*8
	wire [10:0] ext_px    = {1'b0, {1'b0, lfra} + 7'd1, 3'd0}; // (LFRA+1)*8
	wire [10:0] line_px   = active_px + ext_px;
	wire [9:0] lines_tot = (lymax == 8'd0) ? 10'd512 : {1'b0, lymax, 1'b0};
	wire [10:0] load_w    = {3'd0, lpwm, 2'd0} + 11'd16; // LPWM*4+16

	// Pixel-clock divider (LCKR[3:2]: /1 /2 /4 /6). A terminal count is
	// needed rather than a bit test on a free-running mod-6 counter, which
	// cannot express /4.
	reg [2:0] pixdiv_max;
	always @* begin
		case (lckr[3:2])
			2'b00: pixdiv_max = 3'd0;
			2'b01: pixdiv_max = 3'd1;
			2'b10: pixdiv_max = 3'd3;
			2'b11: pixdiv_max = 3'd5;
		endcase
	end
	wire ce_px = ce && (pixdiv >= pixdiv_max);

	// Current pixel level from the latched byte (2 bpp, MSB first). At a
	// line's first pixel the latch of fetch_data lands on the same edge
	// that emits the pixel, so col 0 reads the fetched byte directly; the
	// latched copy still holds the previous line's last byte there.
	wire [7:0] cur_byte = (col == 11'd0) ? fetch_data : pix_byte;
	wire [1:0] px_sel = col[1:0];
	// Byte boundaries: 4 pixels/byte at 2 bpp, 8 at 1 bpp.
	wire byte_pre = bpp2 ? (col[1:0] == 2'd2) : (col[2:0] == 3'd6);
	wire byte_end = bpp2 ? (col[1:0] == 2'd3) : (col[2:0] == 3'd7);
	reg  [1:0] px_level_raw;
	always @* begin
		if (bpp2) begin
			case (px_sel)
				2'd0: px_level_raw = cur_byte[7:6];
				2'd1: px_level_raw = cur_byte[5:4];
				2'd2: px_level_raw = cur_byte[3:2];
				2'd3: px_level_raw = cur_byte[1:0];
			endcase
		end else begin
			// 1 bpp: 8 pixels/byte; col[2:0] indexes, MSB first
			px_level_raw = cur_byte[3'd7 - col[2:0]] ? 2'd3 : 2'd0;
		end
	end
	wire [1:0] px_level = rev ? ~px_level_raw : px_level_raw;

	// Three-scan modulation: ON-count over the scans equals the level.
	reg px_on;
	always @* begin
		case (scan_phase)
			2'd0: px_on = (px_level == 2'd3);
			2'd1: px_on = (px_level != 2'd0);
			default: px_on = (px_level[1] == 1'b1);   // levels 2,3
		endcase
	end

	// 4-pixel group shift register for the LD pins (LD3 = leftmost)
	reg [3:0] ld_sr;

	always @(posedge clk) begin
		if (reset) begin
			lssa       <= 16'h0000;
			lvpw       <= 8'h00;
			lxmax      <= 8'h00;
			lymax      <= 8'h00;
			lpan       <= 3'd0;
			lar        <= 8'hFF;
			lctr       <= 8'h80;
			lckr       <= 8'h00;
			lfra       <= 6'd0;
			lac        <= 5'd0;
			lpwm       <= 6'd0;
			lmisc      <= 8'h00;
			lena       <= 8'hF1;
			pixdiv     <= 3'd0;
			col        <= 11'd0;
			line       <= 10'd0;
			scan_phase <= 2'd0;
			line_base  <= 16'h0000;
			byte_ptr   <= 16'h0000;
			pix_byte   <= 8'h00;
			ac_state   <= 1'b0;
			ld_sr      <= 4'd0;
			fetch_req  <= 1'b0;
			fetch_addr <= 16'h0000;
			lcd_active <= 1'b0;
			pin_flm    <= 1'b0;
			pin_load1  <= 1'b0;
			pin_ac_n   <= 1'b1;
			pin_cp     <= 1'b0;
			pin_ld     <= 4'd0;
			pin_poff_n <= 1'b1;
			pin_blank_n <= 1'b1;
			pin_load2  <= 1'b0;
			int_lcd    <= 1'b0;
		end else begin
			if (ce) begin
				int_lcd    <= 1'b0;
				fetch_req  <= 1'b0;   // one bus cycle wide; set below at ce_px
				lcd_active <= running;
				pin_poff_n <= ~running;
				pixdiv     <= (pixdiv >= pixdiv_max) ? 3'd0 : pixdiv + 3'd1;

				if (sel && reg_we) begin
					case (reg_addr[3:0])
						4'h0: lssa[7:0]  <= reg_wdata;
						4'h1: lssa[15:8] <= reg_wdata;
						4'h2: lvpw       <= reg_wdata;
						4'h3: lxmax      <= reg_wdata;
						4'h4: lymax      <= reg_wdata;
						4'h5: lpan       <= reg_wdata[2:0];
						4'h6: lar        <= reg_wdata;
						4'h7: lctr       <= reg_wdata;
						4'h8: lckr       <= reg_wdata;
						4'h9: lfra       <= reg_wdata[5:0];
						4'hA: lac        <= reg_wdata[4:0];
						4'hB: lpwm       <= reg_wdata[5:0];
						4'hC: lmisc      <= reg_wdata;
						4'hE: lena       <= reg_wdata;
						default: ;
					endcase
				end
			end

			if (ce_px && running) begin
				if (col < active_px) begin
					// Active region: emit the current pixel into the group
					pin_load1 <= 1'b0;
					pin_flm   <= 1'b0;
					ld_sr <= {ld_sr[2:0], px_on && lena[0]};
					// CP pulses once per completed 4-pixel group
					if (col[1:0] == 2'd3) begin
						pin_cp <= lena[4];
						pin_ld <= {ld_sr[2:0], px_on && lena[0]};
					end else begin
						pin_cp <= 1'b0;
					end
					// Fetch the next byte one pixel before it is needed
					if (byte_pre) begin
						fetch_req  <= 1'b1;
						fetch_addr <= byte_ptr + 16'd1;
					end
					if (byte_end) begin
						byte_ptr <= byte_ptr + 16'd1;
						pix_byte <= fetch_data;
					end
				end else begin
					// Line extension: LOAD pulse, then quiet
					pin_cp    <= 1'b0;
					pin_load1 <= lena[6] &&
						(col < (active_px + load_w)) && (lpwm != 6'd0);
					// FLM covers the last line's extension
					pin_flm <= lena[7] && (line == lines_tot - 10'd1);
				end

				// Column/line/frame advance
				if (col == line_px - 11'd1) begin
					col <= 11'd0;
					// Reload the first byte of the next line
					if (line == lines_tot - 10'd1) begin
						// Frame boundary
						line       <= 10'd0;
						line_base  <= lssa;
						byte_ptr   <= lssa;
						fetch_req  <= 1'b1;
						fetch_addr <= lssa;
						int_lcd    <= 1'b1;
						scan_phase <= (scan_phase == 2'd2) ? 2'd0
						                                   : scan_phase + 2'd1;
						if (lac == 5'd0) begin
							ac_state <= ~ac_state;
							pin_ac_n <= lena[5] ? ac_state : 1'b1;
						end
					end else begin
						line       <= line + 10'd1;
						line_base  <= line_base + {6'd0, lvpw, 2'd0};
						byte_ptr   <= line_base + {6'd0, lvpw, 2'd0};
						fetch_req  <= 1'b1;
						fetch_addr <= line_base + {6'd0, lvpw, 2'd0};
					end
				end else begin
					col <= col + 11'd1;
				end
				// Latch the freshly fetched byte at line starts
				if (col == 11'd0) pix_byte <= fetch_data;
			end
			// Savestate restore
			if (ss_wren && (ss_addr == SS_BASE)) begin
				lssa  <= ss_wdata[58:43];
				lvpw  <= ss_wdata[42:35];
				lxmax <= ss_wdata[34:27];
				lymax <= ss_wdata[26:19];
				lpan  <= ss_wdata[18:16];
				lar   <= ss_wdata[15:8];
				lctr  <= ss_wdata[7:0];
			end
			if (ss_wren && (ss_addr == SS_BASE + 10'd1)) begin
				lckr       <= ss_wdata[57:50];
				lfra       <= ss_wdata[49:44];
				lac        <= ss_wdata[43:39];
				lpwm       <= ss_wdata[38:33];
				lmisc      <= ss_wdata[32:25];
				lena       <= ss_wdata[24:17];
				pixdiv     <= ss_wdata[16:14];
				scan_phase <= ss_wdata[13:12];
				ac_state   <= ss_wdata[11];
				col        <= ss_wdata[10:0];
			end
			if (ss_wren && (ss_addr == SS_BASE + 10'd2)) begin
				line       <= ss_wdata[61:52];
				line_base  <= ss_wdata[51:36];
				byte_ptr   <= ss_wdata[35:20];
				pix_byte   <= ss_wdata[19:12];
				ld_sr      <= ss_wdata[11:8];
				pin_flm    <= ss_wdata[7];
				pin_load1  <= ss_wdata[6];
				pin_ac_n   <= ss_wdata[5];
				pin_cp     <= ss_wdata[4];
				pin_ld     <= ss_wdata[3:0];
			end
			if (ss_wren && (ss_addr == SS_BASE + 10'd3)) begin
				fetch_addr <= ss_wdata[16:1];
				fetch_req  <= ss_wdata[0];
			end
		end
	end

	// Register read-back; $40-$42 are write-only
	always @* begin
		case (reg_addr[3:0])
			4'h3: reg_rdata = lxmax;
			4'h4: reg_rdata = lymax;
			4'h5: reg_rdata = {5'h1F, lpan};
			4'h7: reg_rdata = lctr;
			4'h8: reg_rdata = lckr;
			4'hA: reg_rdata = {3'b111, lac};
			4'hB: reg_rdata = {2'b11, lpwm};
			4'hC: reg_rdata = {lmisc[7:4], 1'b0, lmisc[2:1], scan_phase[0]};
			4'hE: reg_rdata = lena;
			default: reg_rdata = 8'hFF;
		endcase
	end

	// Savestate words: 0 = geometry registers; 1 = timing registers +
	// scan counters; 2 = line/fetch/pin state; 3 = fetch port
	assign ss_rdata =
		(ss_addr == SS_BASE) ?
			{5'd0, lssa, lvpw, lxmax, lymax, lpan, lar, lctr} :
		(ss_addr == SS_BASE + 10'd1) ?
			{6'd0, lckr, lfra, lac, lpwm, lmisc, lena, pixdiv, scan_phase,
			 ac_state, col} :
		(ss_addr == SS_BASE + 10'd2) ?
			{2'd0, line, line_base, byte_ptr, pix_byte, ld_sr, pin_flm,
			 pin_load1, pin_ac_n, pin_cp, pin_ld} :
		(ss_addr == SS_BASE + 10'd3) ?
			{47'd0, fetch_addr, fetch_req} : 64'd0;

	wire unused_ok = &{1'b0, reg_addr[6:4], ss_wdata[63:62]};

endmodule
