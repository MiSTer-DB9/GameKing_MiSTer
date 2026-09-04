// Copyright (c) 2026 Jamie Blanks
//
// ST2204 PSG / PWM DAC ($10-$17). The datasheet gives no formulas, so the
// behavior below follows measurements taken from real silicon.
//
// - Two square generators: 12-bit count-up counters reload from PSG0/PSG1
//   on wrap; four wraps per square cycle (toggle every second wrap).
// - Audio clock prescaler PSGC[6:4]: /1 /2 /4 /6 of the system clock, with
//   mode 7 selecting 32.768 kHz. Modes 4-6 select a x2 clock, approximated
//   here as /1; no known software selects them.
// - PWM/noise mode repurposes PSG1: bits 5:0 are the PWM prescaler
//   (count-up to $3F then reload+pulse), bits 11:6 (ordered {[11:8],[7:6]})
//   are the sample counter that fires the level-1 DAC-reload interrupt on
//   reaching $3F (not in modes 0/2/8/A/C/E/F). One PWM sample = one full
//   128-count PWM period: 6 MHz/128 = 46.9 kHz, and the GameKing's $3A
//   reload gives the characteristic ~7.8 kHz sample interrupt.
// - 7-bit PWM, single-ended (both pins, opposite phase) or double-ended
//   (signed: positive values pulse PSGO, negative PSGOB, idle high).
// - Noise: 15-bit LFSR, shift left, feedback bit13^bit14 into bit 0,
//   reset to all ones, shifted once per PWM sample.
// - Volume: 32-step PWM at clk/32 (187.5 kHz), channels interleaved on
//   alternate periods; levels 0-$E gate 0..14 of 16 slots, $F all 16.

module st_psg
#(
	parameter [9:0] SS_BASE = 10'd0
)
(
	input  wire        clk,
	input  wire        ce,             // 6 MHz
	input  wire        ce_32k,
	input  wire        reset,

	input  wire        sel,
	input  wire [6:0]  reg_addr,
	input  wire        reg_we,
	input  wire [7:0]  reg_wdata,
	output reg  [7:0]  reg_rdata,

	output reg         psgo,
	output reg         psgob,
	output reg         int_dac,

	input  wire [9:0]  ss_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata
);

	reg [11:0] psg0;        // $10/$11
	reg [11:0] psg1;        // $12/$13
	reg [7:0]  dac;         // $14
	reg [7:0]  psgc;        // $16
	reg [7:0]  vol;         // $17

	reg [11:0] sq0_count;
	reg [11:0] sq1_count;
	reg [1:0]  sq0_phase;   // 4 wraps per cycle
	reg [1:0]  sq1_phase;
	reg [14:0] lfsr;
	reg [5:0]  pwm_pre;     // PWM prescaler counter
	reg [6:0]  pwm_count;   // 7-bit PWM period counter
	reg [5:0]  smp_count;   // sample interrupt counter
	reg [4:0]  vol_slot;    // 32-step volume PWM counter
	reg [2:0]  pre_count;   // audio clock prescaler

	wire [3:0] mode = psgc[3:0];
	wire [2:0] pre_sel = psgc[6:4];

	// Audio clock enable from the prescaler. A terminal count is needed
	// rather than a bit test on a free-running mod-6 counter, which cannot
	// express /4.
	reg [2:0] pre_max;
	always @* begin
		case (pre_sel)
			3'd1: pre_max = 3'd1;    // /2
			3'd2: pre_max = 3'd3;    // /4
			3'd3: pre_max = 3'd5;    // /6
			default: pre_max = 3'd0; // /1, and the approximated x2 modes
		endcase
	end
	wire ce_audio = (pre_sel == 3'd7) ? ce_32k
	                                  : (ce && (pre_count >= pre_max));

	// Squares toggle every second counter wrap
	wire sq0_out = sq0_phase[1];
	wire sq1_out = sq1_phase[1];

	// The DAC-reload interrupt exists only in these modes
	reg mode_int_en;
	always @* begin
		case (mode)
			4'h0, 4'h2, 4'h8, 4'hA, 4'hC, 4'hE, 4'hF: mode_int_en = 1'b0;
			default: mode_int_en = 1'b1;
		endcase
	end

	wire pwm_mode   = (mode == 4'h1) || (mode == 4'h5) ||
	                  (mode == 4'h9) || (mode == 4'hD);
	wire noise_mode = (mode == 4'hC) || (mode == 4'hE);

	// PWM compare. Double-ended drives the magnitude of the signed DAC value
	// onto one leg. Single-ended treats it as a 7-bit signed value centered
	// on 50% duty: $00 = 64/128, $3F = 127/128, $40 (-64) = 0/128.
	wire        dac_neg = dac[7];
	// 8 bits wide because the magnitude of $80 is 128, one more than the
	// 7-bit sample field holds.
	wire [7:0]  dac_mag = dac_neg ? (8'd128 - {1'b0, dac[6:0]})
	                              : {1'b0, dac[6:0]};
	wire        pwm_hi  = ({1'b0, pwm_count} < dac_mag);
	wire [6:0]  se_duty = 7'd64 + dac[6:0];   // two's-complement wrap intended
	wire        pwm_hi_se = ({1'b0, pwm_count} < {1'b0, se_duty});

	// Volume gating: interleaved channels on alternate 16-slot periods
	wire [3:0] slot = vol_slot[3:0];
	wire       ch_sel = vol_slot[4];   // 0 = channel 1 period, 1 = channel 2
	wire [3:0] vol1 = vol[3:0];
	wire [3:0] vol2 = vol[7:4];
	// Levels 0-$E gate 0..14 of the 16 slots; $F is fully on (15 skipped)
	wire vol1_on = (vol1 == 4'hF) || (slot < vol1);
	wire vol2_on = (vol2 == 4'hF) || (slot < vol2);

	// Channel source per mode
	reg ch1_src, ch2_src;
	always @* begin
		ch1_src = 1'b0;
		ch2_src = 1'b0;
		case (mode)
			4'h2, 4'hA: begin ch1_src = sq0_out; ch2_src = sq0_out; end
			4'h4:       begin ch1_src = sq1_out; ch2_src = sq1_out; end
			4'h6:       begin ch1_src = sq0_out; ch2_src = sq1_out; end
			4'hC:       begin ch1_src = lfsr[14]; ch2_src = lfsr[14]; end
			4'hE:       begin ch1_src = sq0_out; ch2_src = lfsr[14]; end
			default: ;
		endcase
	end

	always @(posedge clk) begin
		if (reset) begin
			psg0      <= 12'h000;
			psg1      <= 12'h000;
			dac       <= 8'h00;
			psgc      <= 8'h0D;
			vol       <= 8'h00;
			sq0_count <= 12'd0;
			sq1_count <= 12'd0;
			sq0_phase <= 2'd0;
			sq1_phase <= 2'd0;
			lfsr      <= 15'h7FFF;
			pwm_pre   <= 6'd0;
			pwm_count <= 7'd0;
			smp_count <= 6'd0;
			vol_slot  <= 5'd0;
			pre_count <= 3'd0;
			psgo      <= 1'b1;
			psgob     <= 1'b1;
			int_dac   <= 1'b0;
		end else begin
			if (ce) begin
				int_dac <= 1'b0;

				// Prescaler phase counter (terminal count per PSGC[6:4])
				pre_count <= (pre_count >= pre_max) ? 3'd0 : pre_count + 3'd1;
				// Volume PWM runs at the raw system clock / 32
				vol_slot <= vol_slot + 5'd1;

				// Register writes
				if (sel && reg_we) begin
					case (reg_addr[2:0])
						3'd0: psg0[7:0]  <= reg_wdata;
						3'd1: psg0[11:8] <= reg_wdata[3:0];
						3'd2: psg1[7:0]  <= reg_wdata;
						3'd3: psg1[11:8] <= reg_wdata[3:0];
						3'd4: dac        <= reg_wdata;
						3'd6: psgc       <= reg_wdata;
						3'd7: vol        <= reg_wdata;
						default: ;
					endcase
				end
			end

			if (ce_audio) begin
				// Square counters: count up, wrap at $FFF, reload
				if (sq0_count == 12'hFFF) begin
					sq0_count <= psg0;
					sq0_phase <= sq0_phase + 2'd1;
				end else begin
					sq0_count <= sq0_count + 12'd1;
				end
				if (!pwm_mode && !noise_mode) begin
					if (sq1_count == 12'hFFF) begin
						sq1_count <= psg1;
						sq1_phase <= sq1_phase + 2'd1;
						// The interrupt fires when the square-2 output
						// toggles, i.e. every second counter wrap
						if (mode_int_en && sq1_phase[0]) int_dac <= 1'b1;
					end else begin
						sq1_count <= sq1_count + 12'd1;
					end
				end else begin
					// PWM/noise: PSG1[5:0] is the PWM prescaler
					if (pwm_pre == 6'h3F) begin
						pwm_pre <= psg1[5:0];
						// PWM period counter: one sample per 128 counts
						pwm_count <= pwm_count + 7'd1;
						if (pwm_count == 7'd127) begin
							// Sample boundary: shift noise, count samples
							lfsr <= {lfsr[13:0], lfsr[13] ^ lfsr[14]};
							if (smp_count == 6'h3F) begin
								smp_count <= {psg1[11:8], psg1[7:6]};
								if (mode_int_en) int_dac <= 1'b1;
							end else begin
								smp_count <= smp_count + 6'd1;
							end
						end
					end else begin
						pwm_pre <= pwm_pre + 6'd1;
					end
				end
			end

			// Pin outputs (updated at the system rate)
			if (ce) begin
				case (mode)
					4'h0, 4'h8: begin
						// Silence; VOL bit 7 drives the negative leg
						psgo  <= 1'b1;
						psgob <= (mode == 4'h0) ? ~vol[7] : vol[7];
					end
					4'h1, 4'h9: begin
						// Single-ended PWM: both pins, opposite phase
						psgo  <= pwm_hi_se;
						psgob <= ~pwm_hi_se;
					end
					4'h5, 4'hD: begin
						// Double-ended signed PWM: idle high
						psgo  <= dac_neg ? 1'b1 : (dac_mag == 8'd0 ? 1'b1 : ~pwm_hi);
						psgob <= dac_neg ? (~pwm_hi) : 1'b1;
					end
					4'h3, 4'h7, 4'hB, 4'hF: begin
						psgo  <= (mode == 4'h7);
						psgob <= 1'b0;
					end
					default: begin
						// Tone/noise through the interleaved volume PWM
						if (!ch_sel) begin
							psgo  <= ch1_src && vol1_on;
							psgob <= ~(ch1_src && vol1_on);
						end else begin
							psgo  <= ch2_src && vol2_on;
							psgob <= ~(ch2_src && vol2_on);
						end
					end
				endcase
			end
			// Savestate restore
			if (ss_wren && (ss_addr == SS_BASE)) begin
				psg0      <= ss_wdata[60:49];
				psg1      <= ss_wdata[48:37];
				dac       <= ss_wdata[36:29];
				psgc      <= ss_wdata[28:21];
				vol       <= ss_wdata[20:13];
				pwm_pre   <= ss_wdata[12:7];
				pwm_count <= ss_wdata[6:0];
			end
			if (ss_wren && (ss_addr == SS_BASE + 10'd1)) begin
				smp_count <= ss_wdata[58:53];
				sq0_count <= ss_wdata[52:41];
				sq1_count <= ss_wdata[40:29];
				sq0_phase <= ss_wdata[28:27];
				sq1_phase <= ss_wdata[26:25];
				lfsr      <= ss_wdata[24:10];
				vol_slot  <= ss_wdata[9:5];
				pre_count <= ss_wdata[4:2];
				psgo      <= ss_wdata[1];
				psgob     <= ss_wdata[0];
			end
		end
	end

	// Reads: $10-$13 return instantaneous counters, $16/$17 latches
	always @* begin
		case (reg_addr[2:0])
			3'd0: reg_rdata = sq0_count[7:0];
			3'd1: reg_rdata = {4'hF, sq0_count[11:8]};
			3'd2: reg_rdata = sq1_count[7:0];
			3'd3: reg_rdata = {4'hF, sq1_count[11:8]};
			3'd4: reg_rdata = dac;
			3'd6: reg_rdata = {1'b1, psgc[6:0]};
			3'd7: reg_rdata = vol;
			default: reg_rdata = 8'hFF;
		endcase
	end

	// Savestate words: 0 = {psg0, psg1, dac, psgc, vol, pwm_pre, pwm_count};
	// 1 = {smp_count, sq counters/phases, lfsr, vol_slot, pre_count, pins}
	assign ss_rdata =
		(ss_addr == SS_BASE) ?
			{3'd0, psg0, psg1, dac, psgc, vol, pwm_pre, pwm_count} :
		(ss_addr == SS_BASE + 10'd1) ?
			{5'd0, smp_count, sq0_count, sq1_count, sq0_phase, sq1_phase,
			 lfsr, vol_slot, pre_count, psgo, psgob} : 64'd0;

	wire unused_ok = &{1'b0, reg_addr[6:3], ss_wdata[63:61]};

endmodule
