// Copyright (c) 2026 Jamie Blanks
//
// GameKing 48x32 STN LCD panel model.
//
// Consumes the pin-level stream from the ST2204 LCDC: CP strobes a 4-pixel
// group on LD3-0 (LD3 = leftmost), LOAD1 advances the line (panel scans
// bottom-up, so screen row = 31 - scan line), FLM marks the frame. The
// liquid crystal integrates the three-scan gray modulation; because a
// pixel's ON-count across the three scans equals its 2-bit level, the
// model sums ON bits over any three consecutive scans (phase-invariant)
// and scales to a 4-bit intensity.
//
// vid_level is darkness: 0 = fully light, 15 = fully dark.

module lcd_panel
(
	input  wire        clk,
	input  wire        ce,             // system enable (pin sampling rate)
	input  wire        reset,

	input  wire        pin_flm,
	input  wire        pin_load1,
	input  wire        pin_ac_n,
	input  wire        pin_cp,
	input  wire [3:0]  pin_ld,
	input  wire        pin_poff_n,

	// Video read port (asynchronous timing)
	input  wire [10:0] vid_addr,
	output wire [3:0]  vid_level,

	// Savestate memory port
	input  wire        ss_active,
	input  wire [10:0] ss_mem_addr,
	input  wire        ss_mem_wren,
	input  wire [7:0]  ss_mem_wdata,
	output wire [7:0]  ss_mem_rdata
);

	// Pin edge detection (pins change on ce)
	reg cp_d, load_d, flm_d;
	always @(posedge clk) begin
		if (ce) begin
			cp_d   <= pin_cp;
			load_d <= pin_load1;
			flm_d  <= pin_flm;
		end
	end
	wire cp_edge   = ce && pin_cp && !cp_d;
	wire load_edge = ce && pin_load1 && !load_d;
	wire flm_edge  = ce && pin_flm && !flm_d;

	// Scan position
	reg [3:0] group_x;      // 0..11 groups of 4 pixels
	reg [5:0] scan_line;    // 0..32+, bottom-up (bit 5 = overrun guard)
	reg [1:0] scan_num;     // 0..2 within the integration window

	// Capture pipeline: process 4 pixels per CP strobe
	reg [3:0] cap_bits;
	reg [3:0] cap_group;
	reg [4:0] cap_line;
	reg [2:0] pipe_k;       // 0..3 pixel within group, 4 = idle
	reg       pipe_rd;      // acc read launched
	reg [10:0] wr_index;    // registered write address: the write enable is
	                        // active one cycle after the compute cycle, by
	                        // which time px_index has moved to the next pixel

	wire [4:0] screen_y = 5'd31 - cap_line[4:0];
	wire [5:0] px_x = {cap_group, 2'd0} + {4'd0, pipe_k[1:0]};
	wire [10:0] px_index = {screen_y, 6'd0} + {5'd0, px_x} - {2'd0, screen_y, 4'd0};
	// pixel index = screen_y*48 + x = screen_y*64 - screen_y*16 + x

	wire px_bit = cap_bits[2'd3 - pipe_k[1:0]];   // LD3 = leftmost

	// Accumulator RAM (2-bit sums, stored in 4)
	wire [3:0] acc_q;
	reg  [3:0] acc_wdata;
	reg        acc_wren;

	cache_ram_dp #(
		.ADDR_WIDTH(11),
		.DATA_WIDTH(4)
	) acc_ram (
		.clk_i(clk),
		.addr_a_i(wr_index),
		.wren_a_i(acc_wren),
		.wdata_a_i(acc_wdata),
		.q_a_o(),
		.addr_b_i(px_index),
		.wren_b_i(1'b0),
		.wdata_b_i(4'h0),
		.q_b_o(acc_q)
	);

	// Frame store (integrated output)
	reg  [3:0] store_wdata;
	reg        store_wren;

	wire [10:0] a_addr  = ss_active ? ss_mem_addr : wr_index;
	wire        a_wren  = ss_active ? ss_mem_wren : store_wren;
	wire [3:0]  a_wdata = ss_active ? ss_mem_wdata[3:0] : store_wdata;
	wire [3:0]  a_q;

	assign ss_mem_rdata = {4'h0, a_q};

	cache_ram_dp #(
		.ADDR_WIDTH(11),
		.DATA_WIDTH(4)
	) frame_store (
		.clk_i(clk),
		.addr_a_i(a_addr),
		.wren_a_i(a_wren),
		.wdata_a_i(a_wdata),
		.q_a_o(a_q),
		.addr_b_i(vid_addr),
		.wren_b_i(1'b0),
		.wdata_b_i(4'h0),
		.q_b_o(vid_level)
	);

	// Level scaling: 0..3 -> 0,5,10,15 (x5 = x4 + x)
	function [3:0] scale5(input [1:0] level);
		scale5 = {level, 2'd0} + {2'd0, level};
	endfunction

	// /POFF low powers the panel. Powering it down only fades the glass,
	// which is not modeled, so the capture state follows reset alone and the
	// frame store keeps the last image.
	always @(posedge clk) begin
		if (reset) begin
			group_x   <= 4'd0;
			scan_line <= 6'd0;
			scan_num  <= 2'd0;
			pipe_k    <= 3'd4;
			pipe_rd   <= 1'b0;
			acc_wren  <= 1'b0;
			store_wren <= 1'b0;
			cap_bits  <= 4'd0;
			cap_group <= 4'd0;
			cap_line  <= 5'd0;
			wr_index    <= 11'd0;
			acc_wdata   <= 4'd0;
			store_wdata <= 4'd0;
		end else begin
			acc_wren   <= 1'b0;
			store_wren <= 1'b0;

			// Pixel pipeline: one pixel per two clocks (read then write)
			if (pipe_k != 3'd4) begin
				if (!pipe_rd) begin
					pipe_rd <= 1'b1;   // acc_q valid next clock
				end else begin
					pipe_rd <= 1'b0;
					acc_wren <= 1'b1;
					wr_index <= px_index;
					if (scan_num == 2'd0) begin
						acc_wdata <= {3'd0, px_bit};
					end else begin
						acc_wdata <= acc_q + {3'd0, px_bit};
					end
					if (scan_num == 2'd2) begin
						store_wren  <= 1'b1;
						store_wdata <=
							scale5(acc_q[1:0] + {1'b0, px_bit});
					end
					pipe_k <= pipe_k + 3'd1;
				end
			end

			// Group capture
			if (cp_edge && (group_x < 4'd12) && !scan_line[5]) begin
				cap_bits  <= pin_ld;
				cap_group <= group_x;
				cap_line  <= scan_line[4:0];
				group_x   <= group_x + 4'd1;
				pipe_k    <= 3'd0;
				pipe_rd   <= 1'b0;
			end

			if (load_edge) begin
				group_x   <= 4'd0;
				if (!scan_line[5]) scan_line <= scan_line + 6'd1;
			end

			if (flm_edge) begin
				group_x   <= 4'd0;
				scan_line <= 6'd0;
				scan_num  <= (scan_num == 2'd2) ? 2'd0 : scan_num + 2'd1;
			end
		end
	end

	wire unused_ok = &{1'b0, pin_ac_n, pin_poff_n, ss_mem_wdata[7:4]};

endmodule
