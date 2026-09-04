// Copyright (c) 2026 Jamie Blanks
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Reflective LCD dot structure, air-gap shadow, and reflector sheet.
//
// Ported from PocketStation_MiSTer's ps_crt_glass, keeping its optical
// equations and phase schedule. The 72 MHz video clock leaves twelve fabric
// clocks around each 6 MHz output pixel, which the pipeline below spends
// one multiplier stage at a time.

module lcd_glass
(
	input  wire        clk,
	input  wire        reset,
	input  wire  [3:0] ph,
	input  wire  [2:0] sx,
	input  wire  [2:0] sy,
	input  wire  [7:0] base_r,
	input  wire  [7:0] base_g,
	input  wire  [7:0] base_b,
	input  wire  [3:0] cell_level,
	input  wire  [3:0] cast_level,
	input  wire  [7:0] gap_r,
	input  wire  [7:0] gap_g,
	input  wire  [7:0] gap_b,
	input  wire        full,
	input  wire signed [7:0] grain,
	output reg   [7:0] out_r,
	output reg   [7:0] out_g,
	output reg   [7:0] out_b
);

	localparam [7:0] K_GRIDC = 8'd243;
	localparam [7:0] K_DROP  = 8'd87;
	localparam [7:0] K_NEAR  = 8'd140;
	localparam [7:0] K_FAR   = 8'd115;

	function automatic [7:0] lift(input [7:0] v);
		lift = (v > 8'd239) ? 8'd255 : (v + 8'd16);
	endfunction

	function automatic [7:0] shade(input [7:0] v);
		shade = (v >> 1) - (v >> 4);
	endfunction

	function automatic [7:0] mix8(input [7:0] a, input [7:0] b,
	                              input [7:0] k);
		logic signed [16:0] d;
		logic         [8:0] s;
		begin
			d = ($signed({9'd0, b}) - $signed({9'd0, a})) *
			    $signed({9'd0, k});
			s = {1'b0, a} + {1'b0, d[15:8]};
			mix8 = s[7:0];
		end
	endfunction

	function automatic [7:0] sat8(input logic signed [17:0] v);
		sat8 = (v < 0) ? 8'd0 : ((v > 255) ? 8'd255 : v[7:0]);
	endfunction

	function automatic [7:0] gcon(input [7:0] v);
		logic signed [17:0] t;
		begin
			t = ((($signed({10'd0, v}) - 18'sd128) *
			      $signed({10'd0, K_GRIDC})) >>> 8) + 18'sd128;
			gcon = sat8(t);
		end
	endfunction

	function automatic [7:0] umbra_of(input [3:0] level);
		case (level)
			4'd7:    umbra_of = 8'd24;
			4'd8:    umbra_of = 8'd83;
			4'd9:    umbra_of = 8'd155;
			4'd10:   umbra_of = 8'd219;
			4'd11:   umbra_of = 8'd253;
			default: umbra_of = (level > 4'd11) ? 8'd255 : 8'd0;
		endcase
	endfunction

	function automatic [7:0] penumbra_of(input [3:0] level);
		case (level)
			4'd6:    penumbra_of = 8'd12;
			4'd7:    penumbra_of = 8'd47;
			4'd8:    penumbra_of = 8'd97;
			4'd9:    penumbra_of = 8'd152;
			4'd10:   penumbra_of = 8'd203;
			4'd11:   penumbra_of = 8'd240;
			default: penumbra_of = (level > 4'd11) ? 8'd255 : 8'd0;
		endcase
	endfunction

	function automatic [7:0] grainy(input [7:0] v,
	                                input signed [7:0] g);
		logic signed [17:0] d;
		begin
			d = ($signed({1'b0, v}) * g + 18'sd2048) >>> 12;
			grainy = sat8($signed({10'd0, v}) + d);
		end
	endfunction

	// Stage 1: cell coverage and the colour inside the dot.
	wire [7:0] body_in = ((sx == 3'd0) || (sy == 3'd0)) ? 8'd0 : 8'd255;
	wire [7:0] k_basea_in = full ? 8'd0 : 8'd26;
	wire [11:0] own_w_in = cell_level * body_in;

	reg [3:0] s1_own_level;
	reg [7:0] s1_lit_r, s1_lit_g, s1_lit_b;
	reg [7:0] s1_base_r, s1_base_g, s1_base_b;
	reg [7:0] s1_gap_r, s1_gap_g, s1_gap_b;
	reg [7:0] s1_body;
	reg [7:0] s1_ramp;
	reg       s1_full;
	reg signed [7:0] s1_grain;

	wire [4:0] sub_sum = {2'd0, sx} + {2'd0, sy};
	wire [8:0] fade_w = ({4'd0, sub_sum} << 4) +
	                      ({4'd0, sub_sum} << 3) + {4'd0, sub_sum};
	wire [7:0] ramp_in = fade_w[8] ? 8'd0 : (8'd255 - fade_w[7:0]);

	// Phase 4: contrast the two grid endpoints. Keeping this separate from
	// their interpolation avoids chaining two multipliers at 72 MHz.
	reg [7:0] s2_own;
	reg [7:0] s2_umbra;
	reg [3:0] s2_cast_level;
	reg [7:0] s2_con_gap_r, s2_con_gap_g, s2_con_gap_b;
	reg [7:0] s2_con_dot_r, s2_con_dot_g, s2_con_dot_b;
	reg [7:0] s2_base_r, s2_base_g, s2_base_b;
	reg [7:0] s2_body;
	reg [7:0] s2_ramp;
	reg [7:0] s2_sheet_r, s2_sheet_g, s2_sheet_b;
	reg       s2_full;
	reg signed [7:0] s2_grain;

	wire [7:0] s2_k_str = s2_full ? 8'd255 : 8'd159;

	// Phase 5: form the endpoints and the two shadow shares.
	reg [7:0] s3_own;
	reg [7:0] s3_gap_r, s3_gap_g, s3_gap_b;
	reg [7:0] s3_dot_r, s3_dot_g, s3_dot_b;
	reg [7:0] s3_body;
	reg [7:0] s3_near;
	reg [7:0] s3_penumbra;
	reg [7:0] s3_sheet_r, s3_sheet_g, s3_sheet_b;
	reg signed [7:0] s3_grain;

	wire [15:0] near_w = s2_umbra * K_NEAR;
	wire [15:0] pen_ramp = penumbra_of(s2_cast_level) * s2_ramp;

	// Phase 6: grid colour and coincident umbra/penumbra depth.
	reg [7:0] s4_own;
	reg [7:0] s4_grid_r, s4_grid_g, s4_grid_b;
	reg [7:0] s4_cast;
	reg [7:0] s4_sheet_r, s4_sheet_g, s4_sheet_b;
	reg signed [7:0] s4_grain;

	wire [15:0] broad_w = s3_penumbra * K_FAR;
	wire [8:0] cast_w = {1'b0, s3_near} + {1'b0, broad_w[15:8]};

	// Phases 7 and 8 apply shadow opacity and own-ink suppression separately.
	reg [7:0] s5_drop;
	reg [7:0] s5_clear;
	reg [7:0] s5_grid_r, s5_grid_g, s5_grid_b;
	reg [7:0] s5_sheet_r, s5_sheet_g, s5_sheet_b;
	reg signed [7:0] s5_grain;
	wire [15:0] drop_w = s4_cast * K_DROP;

	reg [7:0] s6_amt;
	reg [7:0] s6_grid_r, s6_grid_g, s6_grid_b;
	reg [7:0] s6_sheet_r, s6_sheet_g, s6_sheet_b;
	reg signed [7:0] s6_grain;
	wire [15:0] amt_w = s5_drop * s5_clear;

	// Phases 9 and 10 lay the shadow and reflector grain separately.
	reg [7:0] s7_shad_r, s7_shad_g, s7_shad_b;
	reg signed [7:0] s7_grain;

	always @(posedge clk) begin
		if (reset) begin
			s1_own_level <= 4'd0;
			s1_lit_r <= 8'd0;
			s1_lit_g <= 8'd0;
			s1_lit_b <= 8'd0;
			s1_base_r <= 8'd0;
			s1_base_g <= 8'd0;
			s1_base_b <= 8'd0;
			s1_gap_r <= 8'd0;
			s1_gap_g <= 8'd0;
			s1_gap_b <= 8'd0;
			s1_body <= 8'd0;
			s1_ramp <= 8'd0;
			s1_full <= 1'b0;
			s1_grain <= 8'sd0;

			s2_own <= 8'd0;
			s2_umbra <= 8'd0;
			s2_cast_level <= 4'd0;
			s2_con_gap_r <= 8'd0;
			s2_con_gap_g <= 8'd0;
			s2_con_gap_b <= 8'd0;
			s2_con_dot_r <= 8'd0;
			s2_con_dot_g <= 8'd0;
			s2_con_dot_b <= 8'd0;
			s2_base_r <= 8'd0;
			s2_base_g <= 8'd0;
			s2_base_b <= 8'd0;
			s2_body <= 8'd0;
			s2_ramp <= 8'd0;
			s2_sheet_r <= 8'd0;
			s2_sheet_g <= 8'd0;
			s2_sheet_b <= 8'd0;
			s2_full <= 1'b0;
			s2_grain <= 8'sd0;

			s3_own <= 8'd0;
			s3_gap_r <= 8'd0;
			s3_gap_g <= 8'd0;
			s3_gap_b <= 8'd0;
			s3_dot_r <= 8'd0;
			s3_dot_g <= 8'd0;
			s3_dot_b <= 8'd0;
			s3_body <= 8'd0;
			s3_near <= 8'd0;
			s3_penumbra <= 8'd0;
			s3_sheet_r <= 8'd0;
			s3_sheet_g <= 8'd0;
			s3_sheet_b <= 8'd0;
			s3_grain <= 8'sd0;

			s4_own <= 8'd0;
			s4_grid_r <= 8'd0;
			s4_grid_g <= 8'd0;
			s4_grid_b <= 8'd0;
			s4_cast <= 8'd0;
			s4_sheet_r <= 8'd0;
			s4_sheet_g <= 8'd0;
			s4_sheet_b <= 8'd0;
			s4_grain <= 8'sd0;

			s5_drop <= 8'd0;
			s5_clear <= 8'd0;
			s5_grid_r <= 8'd0;
			s5_grid_g <= 8'd0;
			s5_grid_b <= 8'd0;
			s5_sheet_r <= 8'd0;
			s5_sheet_g <= 8'd0;
			s5_sheet_b <= 8'd0;
			s5_grain <= 8'sd0;

			s6_amt <= 8'd0;
			s6_grid_r <= 8'd0;
			s6_grid_g <= 8'd0;
			s6_grid_b <= 8'd0;
			s6_sheet_r <= 8'd0;
			s6_sheet_g <= 8'd0;
			s6_sheet_b <= 8'd0;
			s6_grain <= 8'sd0;

			s7_shad_r <= 8'd0;
			s7_shad_g <= 8'd0;
			s7_shad_b <= 8'd0;
			s7_grain <= 8'sd0;
			out_r <= 8'd0;
			out_g <= 8'd0;
			out_b <= 8'd0;
		end
		else begin
			if (ph == 4'd3) begin
				s1_own_level <= own_w_in[11:8];
				s1_lit_r <= mix8(lift(gap_r), base_r, 8'd255 - k_basea_in);
				s1_lit_g <= mix8(lift(gap_g), base_g, 8'd255 - k_basea_in);
				s1_lit_b <= mix8(lift(gap_b), base_b, 8'd255 - k_basea_in);
				s1_base_r <= base_r;
				s1_base_g <= base_g;
				s1_base_b <= base_b;
				s1_gap_r <= gap_r;
				s1_gap_g <= gap_g;
				s1_gap_b <= gap_b;
				s1_body <= body_in;
				s1_ramp <= ramp_in;
				s1_full <= full;
				s1_grain <= grain;
			end

			if (ph == 4'd4) begin
				s2_own <= umbra_of(s1_own_level);
				s2_umbra <= umbra_of(cast_level);
				s2_cast_level <= cast_level;
				s2_con_gap_r <= gcon(s1_gap_r);
				s2_con_gap_g <= gcon(s1_gap_g);
				s2_con_gap_b <= gcon(s1_gap_b);
				s2_con_dot_r <= gcon(s1_lit_r);
				s2_con_dot_g <= gcon(s1_lit_g);
				s2_con_dot_b <= gcon(s1_lit_b);
				s2_base_r <= s1_base_r;
				s2_base_g <= s1_base_g;
				s2_base_b <= s1_base_b;
				s2_body <= s1_body;
				s2_ramp <= s1_ramp;
				s2_sheet_r <= s1_gap_r;
				s2_sheet_g <= s1_gap_g;
				s2_sheet_b <= s1_gap_b;
				s2_full <= s1_full;
				s2_grain <= s1_grain;
			end

			if (ph == 4'd5) begin
				s3_own <= s2_own;
				s3_gap_r <= mix8(s2_base_r, s2_con_gap_r, s2_k_str);
				s3_gap_g <= mix8(s2_base_g, s2_con_gap_g, s2_k_str);
				s3_gap_b <= mix8(s2_base_b, s2_con_gap_b, s2_k_str);
				s3_dot_r <= mix8(s2_base_r, s2_con_dot_r, s2_k_str);
				s3_dot_g <= mix8(s2_base_g, s2_con_dot_g, s2_k_str);
				s3_dot_b <= mix8(s2_base_b, s2_con_dot_b, s2_k_str);
				s3_body <= s2_body;
				s3_near <= near_w[15:8];
				s3_penumbra <= pen_ramp[15:8];
				s3_sheet_r <= s2_sheet_r;
				s3_sheet_g <= s2_sheet_g;
				s3_sheet_b <= s2_sheet_b;
				s3_grain <= s2_grain;
			end

			if (ph == 4'd6) begin
				s4_own <= s3_own;
				s4_grid_r <= mix8(s3_gap_r, s3_dot_r, s3_body);
				s4_grid_g <= mix8(s3_gap_g, s3_dot_g, s3_body);
				s4_grid_b <= mix8(s3_gap_b, s3_dot_b, s3_body);
				s4_cast <= cast_w[8] ? 8'd255 : cast_w[7:0];
				s4_sheet_r <= s3_sheet_r;
				s4_sheet_g <= s3_sheet_g;
				s4_sheet_b <= s3_sheet_b;
				s4_grain <= s3_grain;
			end

			if (ph == 4'd7) begin
				s5_drop <= drop_w[15:8];
				s5_clear <= 8'd255 - s4_own;
				s5_grid_r <= s4_grid_r;
				s5_grid_g <= s4_grid_g;
				s5_grid_b <= s4_grid_b;
				s5_sheet_r <= s4_sheet_r;
				s5_sheet_g <= s4_sheet_g;
				s5_sheet_b <= s4_sheet_b;
				s5_grain <= s4_grain;
			end

			if (ph == 4'd8) begin
				s6_amt <= amt_w[15:8];
				s6_grid_r <= s5_grid_r;
				s6_grid_g <= s5_grid_g;
				s6_grid_b <= s5_grid_b;
				s6_sheet_r <= s5_sheet_r;
				s6_sheet_g <= s5_sheet_g;
				s6_sheet_b <= s5_sheet_b;
				s6_grain <= s5_grain;
			end

			if (ph == 4'd9) begin
				s7_shad_r <= mix8(s6_grid_r, shade(s6_sheet_r), s6_amt);
				s7_shad_g <= mix8(s6_grid_g, shade(s6_sheet_g), s6_amt);
				s7_shad_b <= mix8(s6_grid_b, shade(s6_sheet_b), s6_amt);
				s7_grain <= s6_grain;
			end

			if (ph == 4'd10) begin
				out_r <= grainy(s7_shad_r, s7_grain);
				out_g <= grainy(s7_shad_g, s7_grain);
				out_b <= grainy(s7_shad_b, s7_grain);
			end
		end
	end

endmodule
