// Copyright (c) 2026 Jamie Blanks
// SPDX-License-Identifier: GPL-2.0-or-later
//
// GameKing video presentation.
//
// A 72 MHz video clock produces a 6 MHz 240p raster. Once per output frame a
// local BRAM is filled from the panel's 24 MHz read port, with every address
// held for more than three machine clocks. Flat output reads one cell per
// pixel. Display Mode selects Glass (the reflective-LCD renderer, which reads
// the cell and its upper-left caster during the first three of twelve fabric
// clocks and finishes by phase ten), Scaled (flat 6x), or Raw (48x32).

module video_gen
(
	input  wire        clk,
	input  wire        reset,

	// Panel frame store. The address crosses to clk_sys but is held stable
	// while the video-domain presentation BRAM is filled.
	output reg  [10:0] vid_addr,
	input  wire [3:0]  vid_level,

	input  wire        pal_wr,
	input  wire [6:0]  pal_addr,
	input  wire [7:0]  pal_data,
	input  wire [1:0]  pal_select,

	// 0 = Glass, 1 = Scaled, 2 = Raw
	input  wire [1:0]  disp_mode,

	output reg         ce_pix,
	output wire [7:0]  r,
	output wire [7:0]  g,
	output wire [7:0]  b,
	output wire        hsync,
	output wire        vsync,
	output wire        hblank,
	output wire        vblank
);

	localparam [8:0] H_TOTAL   = 9'd381;
	localparam [8:0] H_SYNC_ON = 9'd324;
	localparam [8:0] H_SYNC_W  = 9'd28;
	localparam [8:0] V_TOTAL   = 9'd262;
	localparam [8:0] V_SYNC_ON = 9'd244;
	localparam [8:0] V_SYNC_W  = 9'd3;
	localparam [3:0] PIX_DIV   = 4'd12;

	reg [3:0] pix_div;
	reg [8:0] hcnt, vcnt;
	reg [1:0] disp_q;

	wire hlast = (hcnt == H_TOTAL - 9'd1);
	wire vlast = (vcnt == V_TOTAL - 9'd1);
	wire frame_wrap = (pix_div == 4'd0) && hlast && vlast;

	always @(posedge clk) begin
		if (reset) begin
			pix_div <= 4'd0;
			ce_pix <= 1'b0;
			hcnt <= 9'd0;
			vcnt <= 9'd0;
			disp_q <= disp_mode;
		end
		else begin
			pix_div <= (pix_div == PIX_DIV - 4'd1) ? 4'd0 :
			           (pix_div + 4'd1);
			ce_pix <= (pix_div == 4'd0);

			if (pix_div == 4'd0) begin
				if (hlast) begin
					hcnt <= 9'd0;
					if (vlast) begin
						vcnt <= 9'd0;
						disp_q <= disp_mode;
					end
					else begin
						vcnt <= vcnt + 9'd1;
					end
				end
				else begin
					hcnt <= hcnt + 9'd1;
				end
			end
		end
	end

	// Glass is a sub-pixel renderer, so Raw remains one pixel per cell.
	// Mode 3 is unused and falls in with Raw.
	wire native_q = disp_q[1];
	wire glass_on = (disp_q == 2'd0);
	wire [8:0] pic_x0 = native_q ? 9'd134 : (glass_on ? 9'd13 : 9'd14);
	wire [8:0] pic_y0 = native_q ? 9'd104 : (glass_on ? 9'd23 : 9'd24);
	wire [8:0] pic_w  = native_q ? 9'd48  : (glass_on ? 9'd289 : 9'd288);
	wire [8:0] pic_h  = native_q ? 9'd32  : (glass_on ? 9'd193 : 9'd192);

	wire pic_hblank = (hcnt < pic_x0) || (hcnt >= pic_x0 + pic_w);
	wire pic_vblank = (vcnt < pic_y0) || (vcnt >= pic_y0 + pic_h);
	wire [8:0] active_x = hcnt - pic_x0;

	assign hsync = (hcnt >= H_SYNC_ON) && (hcnt < H_SYNC_ON + H_SYNC_W);
	assign vsync = (vcnt >= V_SYNC_ON) && (vcnt < V_SYNC_ON + V_SYNC_W);
	assign hblank = pic_hblank;
	assign vblank = pic_vblank;

	// ---------------------------------------------------------------------
	// Copy the machine-domain panel store once per output frame.

	localparam [3:0] PANEL_HOLD = 4'd10;

	reg [10:0] fill_addr;
	reg        filling;
	reg [3:0]  fill_wait;
	reg [10:0] bank_waddr;
	reg        bank_we;
	reg [3:0]  bank_wdata;
	wire [3:0] bank_q;
	reg [10:0] bank_raddr;

	cache_ram #(
		.ADDR_WIDTH (11),
		.DATA_WIDTH (4)
	) present_bank (
		.clk_i   (clk),
		.addr_i  (bank_we ? bank_waddr : bank_raddr),
		.wren_i  (bank_we),
		.wdata_i (bank_wdata),
		.q_o     (bank_q)
	);

	always @(posedge clk) begin
		if (reset) begin
			fill_addr <= 11'd0;
			filling <= 1'b0;
			fill_wait <= 4'd0;
			bank_waddr <= 11'd0;
			bank_we <= 1'b0;
			bank_wdata <= 4'd0;
			vid_addr <= 11'd0;
		end
		else begin
			bank_we <= 1'b0;
			if (frame_wrap) begin
				filling <= 1'b1;
				fill_addr <= 11'd0;
				fill_wait <= 4'd1;
				vid_addr <= 11'd0;
			end
			else if (filling) begin
				if (fill_wait != PANEL_HOLD) begin
					fill_wait <= fill_wait + 4'd1;
				end
				else begin
					bank_waddr <= fill_addr;
					bank_wdata <= vid_level;
					bank_we <= 1'b1;
					fill_wait <= 4'd1;
					if (fill_addr == 11'd1535) begin
						filling <= 1'b0;
					end
					else begin
						fill_addr <= fill_addr + 11'd1;
						vid_addr <= fill_addr + 11'd1;
					end
				end
			end
		end
	end

	// ---------------------------------------------------------------------
	// Whole-number scan position. The extra row and column in glass mode
	// close the light gap lattice after the last panel cell.

	wire [3:0] scale = native_q ? 4'd1 : 4'd6;
	reg [5:0] px, py;
	reg [3:0] xsub, ysub;
	reg       in_x, in_y;

	always @(posedge clk) begin
		if (reset) begin
			px <= 6'd0;
			py <= 6'd0;
			xsub <= 4'd0;
			ysub <= 4'd0;
			in_x <= 1'b0;
			in_y <= 1'b0;
		end
		else if (ce_pix) begin
			if (pic_vblank) begin
				in_y <= 1'b0;
			end
			else if (!pic_hblank && (active_x == 9'd0)) begin
				if (!in_y) begin
					in_y <= 1'b1;
					py <= 6'd0;
					ysub <= 4'd0;
				end
				else if (ysub == scale - 4'd1) begin
					ysub <= 4'd0;
					py <= py + 6'd1;
				end
				else begin
					ysub <= ysub + 4'd1;
				end
			end

			if (pic_hblank) begin
				in_x <= 1'b0;
			end
			else if (!in_x) begin
				in_x <= 1'b1;
				px <= 6'd0;
				xsub <= 4'd0;
			end
			else if (xsub == scale - 4'd1) begin
				xsub <= 4'd0;
				px <= px + 6'd1;
			end
			else begin
				xsub <= xsub + 4'd1;
			end
		end
	end

	// Phase 1 addresses the cell, phase 2 receives it and addresses the
	// caster, phase 3 receives the caster, and the renderer ends at phase 10.
	reg [3:0] ph;
	always @(posedge clk) begin
		if (reset) ph <= 4'd0;
		else ph <= ce_pix ? 4'd1 : ((ph == 4'd11) ? 4'd11 : (ph + 4'd1));
	end

	function automatic [2:0] step_back(input [3:0] sub, input int off,
	                                   input [3:0] sc);
		if (sub >= off[3:0])                              step_back = 3'd0;
		else if (({4'd0, sub} + {4'd0, sc}) >= off[7:0]) step_back = 3'd1;
		else                                               step_back = 3'd2;
	endfunction

	wire [2:0] cast_bx = step_back(xsub, 2, scale);
	wire [2:0] cast_by = step_back(ysub, 2, scale);
	wire cast_ok = (px >= {3'd0, cast_bx}) && (py >= {3'd0, cast_by});
	wire [5:0] cast_x = px - {3'd0, cast_bx};
	wire [5:0] cast_y = py - {3'd0, cast_by};
	wire [5:0] cell_x = (px >= 6'd48) ? 6'd47 : px;
	wire [4:0] cell_y = (py >= 6'd32) ? 5'd31 : py[4:0];

	wire [10:0] cell_addr = {cell_y, 6'd0} - {2'd0, cell_y, 4'd0} +
	                         {5'd0, cell_x};
	wire [10:0] cast_addr = {cast_y[4:0], 6'd0} -
	                         {2'd0, cast_y[4:0], 4'd0} + {5'd0, cast_x};

	always @(*) begin
		if (ph == 4'd1) bank_raddr = cell_addr;
		else bank_raddr = cast_ok ? cast_addr : 11'd0;
	end

	// ---------------------------------------------------------------------
	// Palette. Four RGB anchors are interpolated over the 4-bit darkness.

	// pal_select: 0 = LCD, 1 = Greyscale, 2 = Custom.
	reg [23:0] pal_rom [0:7];
	initial begin
		pal_rom[0]  = 24'hC7D8A2; pal_rom[1]  = 24'h8FA275;
		pal_rom[2]  = 24'h5A6B4A; pal_rom[3]  = 24'h27331F;
		pal_rom[4]  = 24'hF0F0F0; pal_rom[5]  = 24'hA0A0A0;
		pal_rom[6]  = 24'h505050; pal_rom[7]  = 24'h000000;
	end

	// The custom anchors power up as a copy of LCD, so selecting Custom
	// before any .gbp is loaded shows the default panel colours.
	reg [7:0] pal_cust [0:11];
	initial begin
		pal_cust[0]  = 8'hC7; pal_cust[1]  = 8'hD8; pal_cust[2]  = 8'hA2;
		pal_cust[3]  = 8'h8F; pal_cust[4]  = 8'hA2; pal_cust[5]  = 8'h75;
		pal_cust[6]  = 8'h5A; pal_cust[7]  = 8'h6B; pal_cust[8]  = 8'h4A;
		pal_cust[9]  = 8'h27; pal_cust[10] = 8'h33; pal_cust[11] = 8'h1F;
	end
	always @(posedge clk) begin
		if (pal_wr && (pal_addr < 7'd12))
			pal_cust[pal_addr[3:0]] <= pal_data;
	end

	function automatic [7:0] mix(input [7:0] a, input [7:0] bch,
	                             input [1:0] fraction);
		reg [10:0] acc;
		begin
			case (fraction)
				2'd0: acc = {1'b0, a, 2'd0};
				2'd1: acc = {1'b0, a, 1'b0} + {2'd0, a} + {3'd0, bch};
				2'd2: acc = {1'b0, a, 1'b0} + {1'b0, bch, 1'b0};
				default: acc = {3'd0, a} + {1'b0, bch, 1'b0} +
				               {2'd0, bch};
			endcase
			mix = acc[9:2];
		end
	endfunction

	wire [1:0] seg = bank_q[3:2];
	wire [1:0] frac = bank_q[1:0];
	wire [1:0] segn = (seg == 2'd3) ? 2'd3 : (seg + 2'd1);
	wire [3:0] seg3 = {2'd0, seg} + {1'd0, seg, 1'd0};
	wire [3:0] segn3 = {2'd0, segn} + {1'd0, segn, 1'd0};
	wire [23:0] c0_cust = {pal_cust[seg3], pal_cust[seg3 + 4'd1],
	                       pal_cust[seg3 + 4'd2]};
	wire [23:0] c1_cust = {pal_cust[segn3], pal_cust[segn3 + 4'd1],
	                       pal_cust[segn3 + 4'd2]};
	// Select 3 is unreachable from the three-entry menu; it falls in
	// with Custom.
	wire use_cust = pal_select[1];
	wire [23:0] c0 = use_cust ? c0_cust : pal_rom[{pal_select[0], seg}];
	wire [23:0] c1 = use_cust ? c1_cust : pal_rom[{pal_select[0], segn}];
	wire [23:0] ground = use_cust ?
		{pal_cust[0], pal_cust[1], pal_cust[2]} :
		pal_rom[{pal_select[0], 2'd0}];

	reg [7:0] base_r, base_g, base_b;
	reg [3:0] cell_level, cast_level;
	always @(posedge clk) begin
		if (reset) begin
			base_r <= 8'd0;
			base_g <= 8'd0;
			base_b <= 8'd0;
			cell_level <= 4'd0;
			cast_level <= 4'd0;
		end
		else begin
			if (ph == 4'd2) begin
				cell_level <= bank_q;
				base_r <= mix(c0[23:16], c1[23:16], frac);
				base_g <= mix(c0[15:8], c1[15:8], frac);
				base_b <= mix(c0[7:0], c1[7:0], frac);
			end
			if (ph == 4'd3) cast_level <= cast_ok ? bank_q : 4'd0;
		end
	end

	// Reflector grain is reseeded each frame, so it stays fixed on screen.
	reg [15:0] fine;
	always @(posedge clk) begin
		if (reset || frame_wrap) fine <= 16'hACE1;
		else if (ce_pix) fine <= (fine >> 1) ^
		                           (fine[0] ? 16'hB400 : 16'h0000);
	end

	wire [10:0] cell_ix = {cell_y, cell_x} ^ 11'h2D9;
	wire [26:0] mottle_w = cell_ix * 16'h6F35;
	wire [26:0] mottle_0 = mottle_w ^ {11'd0, mottle_w[26:11]};
	wire [26:0] mottle = mottle_0 ^ {5'd0, mottle_0[26:5]};
	wire signed [10:0] fine_w =
		$signed({{3{fine[7]}}, fine[7:0]}) * 11'sd3;
	wire signed [10:0] mottle_wt =
		$signed({{3{mottle[7]}}, mottle[7:0]});
	wire signed [7:0] grain = $signed(fine_w[10:3]) +
	                           $signed(mottle_wt[10:3]);

	wire [7:0] glass_r, glass_g, glass_b;
	lcd_glass glass (
		.clk        (clk),
		.reset      (reset),
		.ph         (ph),
		.sx         (xsub[2:0]),
		.sy         (ysub[2:0]),
		.base_r     (base_r),
		.base_g     (base_g),
		.base_b     (base_b),
		.cell_level (cell_level),
		.cast_level (cast_level),
		.gap_r      (ground[23:16]),
		.gap_g      (ground[15:8]),
		.gap_b      (ground[7:0]),
		.full       (1'b1),
		.grain      (grain),
		.out_r      (glass_r),
		.out_g      (glass_g),
		.out_b      (glass_b)
	);

	reg       visible_q;
	reg [7:0] flat_r, flat_g, flat_b;
	wire visible = in_x && in_y && !pic_hblank && !pic_vblank;
	always @(posedge clk) begin
		if (reset) begin
			visible_q <= 1'b0;
			flat_r <= 8'd0;
			flat_g <= 8'd0;
			flat_b <= 8'd0;
		end
		else if (ph == 4'd7) begin
			visible_q <= visible;
			if (visible) begin
				flat_r <= base_r;
				flat_g <= base_g;
				flat_b <= base_b;
			end
			else begin
				flat_r <= {2'd0, ground[23:18]};
				flat_g <= {2'd0, ground[15:10]};
				flat_b <= {2'd0, ground[7:2]};
			end
		end
	end

	wire use_glass = glass_on && visible_q;
	assign r = use_glass ? glass_r : flat_r;
	assign g = use_glass ? glass_g : flat_g;
	assign b = use_glass ? glass_b : flat_b;

endmodule
