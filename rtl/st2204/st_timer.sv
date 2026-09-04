// Copyright (c) 2026 Jamie Blanks
//
// ST2204 timers ($20-$27) and watchdog counter.
//
// Base timer: ripple chain from the 32.768 kHz domain with taps at 2048,
// 256, 64, 8, and 2 Hz gated by BTEN; a firing tap sets its BTSR bit and
// raises the level-5 interrupt only if BTSR was clear (software clears via
// a bit-7 write to $21 to re-arm).
//
// PRS ($23): a 16-bit count-up prescaler on the system clock. Writing bit 7
// pulses a counter reset (the bit is self-clearing), bit 6 enables it and
// bit 5 picks the source. Nothing on the GameKing drives the INTX
// event-counter input, so only the SYSCK source is implemented. Reads return
// the low count byte.
//
// Timer 0: an 8-bit count-up at the PRS tap selected by T0M[2:0]
// (/65536 /32768 /8192 /2048 /256 /32 /8 /2), run on bit 5 and auto-reload
// from T0C on bit 4. The GameKing's $34/$49 values give the observed
// 128.07 Hz.
//
// Timer 1: with T1M bit 3 set it clocks from OSCX and interrupts every
// (256 >> T1M[1:0]) ticks, divided by a further 256 when bit 4 is clear,
// which reproduces every rate measured on hardware. Mode 3 raises no
// interrupts. The non-OSCX path mirrors Timer 0 on the PRS taps.
//
// Watchdog: enabled by MISC.3 through st_sys, period 2 s or 8 s by MISC.2,
// clocked from the 2 Hz base tap. Any MISC write clears it and a timeout
// pulses wdt_reset.

module st_timer
#(
	parameter [9:0] SS_BASE = 10'd0
)
(
	input  wire        clk,
	input  wire        ce,             // 6 MHz system enable
	input  wire        ce_32k,         // 32.768 kHz enable
	input  wire        reset,

	input  wire        sel,
	input  wire [6:0]  reg_addr,
	input  wire        reg_we,
	input  wire [7:0]  reg_wdata,
	output reg  [7:0]  reg_rdata,

	input  wire        intx_pin,       // PC0 as the PRS event source

	output reg         int_t0,
	output reg         int_t1,
	output reg         int_bt,
	output reg         wdt_reset,
	input  wire        wdt_en,
	input  wire        wdt_ps,
	input  wire        wdt_kick,

	input  wire [9:0]  ss_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata
);

	// Registers
	reg [4:0]  bten;        // $20
	reg [4:0]  btsr;        // $21
	reg [7:0]  prs;         // $23 control ($E0)
	reg [7:0]  t0m;         // $24
	reg [7:0]  t0c;         // $25 (write: counter + reload)
	reg [7:0]  t1m;         // $26
	reg [7:0]  t1c;         // $27

	// Counters
	reg [15:0] pres;        // PRS count chain
	reg [7:0]  t0_count;
	reg [7:0]  t1_count;    // non-OSCX mode
	reg [7:0]  t1_oscx_div; // OSCX tick divider
	reg [7:0]  t1_oscx_256; // optional /256 stage (T1M bit 4 clear)
	reg [14:0] bt_chain;
	reg [4:0]  wdt_count;

	wire prs_run = (prs[6:5] == 2'b10);   // enabled, SYSCK source

	// PRS tap pulses: carry into bit k when the low k bits are all ones.
	// T0M/T1M tap ladder: 0 = /65536 ... 7 = /2.
	function tap_pulse(input [2:0] mode, input [15:0] count);
		case (mode)
			3'd0: tap_pulse = (count[15:0] == 16'hFFFF);
			3'd1: tap_pulse = (count[14:0] == 15'h7FFF);
			3'd2: tap_pulse = (count[12:0] == 13'h1FFF);
			3'd3: tap_pulse = (count[10:0] == 11'h7FF);
			3'd4: tap_pulse = (count[7:0]  == 8'hFF);
			3'd5: tap_pulse = (count[4:0]  == 5'h1F);
			3'd6: tap_pulse = (count[2:0]  == 3'h7);
			3'd7: tap_pulse = (count[0]    == 1'h1);
		endcase
	endfunction

	// Base-timer tap pulses from the 32.768 kHz ripple chain
	wire bt_tap_2048 = (bt_chain[3:0]  == 4'hF);
	wire bt_tap_256  = (bt_chain[6:0]  == 7'h7F);
	wire bt_tap_64   = (bt_chain[8:0]  == 9'h1FF);
	wire bt_tap_8    = (bt_chain[11:0] == 12'hFFF);
	wire bt_tap_2    = (bt_chain[13:0] == 14'h3FFF);
	wire [4:0] bt_fire = {bt_tap_2048, bt_tap_256, bt_tap_64, bt_tap_8,
	                      bt_tap_2} & bten;

	// T1 OSCX-mode tick target: 256 >> T1M[1:0] ticks of 32.768 kHz
	wire [7:0] t1_tick_target = (8'd255 >> t1m[1:0]);

	// BTCLR ($21 bit 7). The clear and a firing base-timer tap can land on
	// the same clock, since ce and ce_32k coincide, so both feed one merged
	// next-state for BTSR. Splitting them lets the 32 kHz arm overwrite the
	// clear, leaving BTSR non-zero and int_bt - which only fires from a zero
	// BTSR - permanently unarmed.
	wire btclr = ce && sel && reg_we && (reg_addr[2:0] == 3'd1) && reg_wdata[7];
	wire [4:0] btsr_pre = btclr ? 5'd0 : btsr;

	always @(posedge clk) begin
		if (reset) begin
			bten        <= 5'd0;
			btsr        <= 5'd0;
			prs         <= 8'h00;
			t0m         <= 8'h00;
			t0c         <= 8'h00;
			t1m         <= 8'h00;
			t1c         <= 8'h00;
			pres        <= 16'h0000;
			t0_count    <= 8'h00;
			t1_count    <= 8'h00;
			t1_oscx_div <= 8'h00;
			t1_oscx_256 <= 8'h00;
			bt_chain    <= 15'd0;
			wdt_count   <= 5'd0;
			int_t0      <= 1'b0;
			int_t1      <= 1'b0;
			int_bt      <= 1'b0;
			wdt_reset   <= 1'b0;
		end else begin
			if (ce) begin
				int_t0    <= 1'b0;
				int_t1    <= 1'b0;
				int_bt    <= 1'b0;
				wdt_reset <= 1'b0;

				// Prescaler and system-clock timers
				if (prs_run) begin
					pres <= pres + 16'd1;
					// Timer 0
					if (t0m[5] && tap_pulse(t0m[2:0], pres)) begin
						if (t0_count == 8'hFF) begin
							t0_count <= t0m[4] ? t0c : 8'h00;
							int_t0   <= 1'b1;
						end else begin
							t0_count <= t0_count + 8'd1;
						end
					end
					// Timer 1, non-OSCX path (unused by the GameKing)
					if (!t1m[3] && (t1m[1:0] != 2'b11) &&
					    tap_pulse(t1m[2:0], pres)) begin
						if (t1_count == 8'hFF) begin
							t1_count <= t1m[4] ? t1c : 8'h00;
							int_t1   <= 1'b1;
						end else begin
							t1_count <= t1_count + 8'd1;
						end
					end
				end

				// Register writes
				if (sel && reg_we) begin
					case (reg_addr[2:0])
						3'd0: bten <= reg_wdata[4:0];
						3'd1: ;   // BTCLR: see btclr/btsr_pre
						3'd3: begin
							prs <= {1'b0, reg_wdata[6:5], 5'd0};
							if (reg_wdata[7]) pres <= 16'h0000;
						end
						3'd4: t0m <= reg_wdata & 8'h37;
						3'd5: begin
							t0c      <= reg_wdata;
							t0_count <= reg_wdata;
						end
						3'd6: t1m <= reg_wdata & 8'h1F;
						3'd7: begin
							t1c      <= reg_wdata;
							t1_count <= reg_wdata;
						end
						default: ;
					endcase
				end
			end

			// 32.768 kHz domain
			if (ce_32k) begin
				bt_chain <= bt_chain + 15'd1;
				if (|bt_fire) begin
					if (btsr_pre == 5'd0) int_bt <= 1'b1;
					btsr <= btsr_pre | bt_fire;
				end else begin
					btsr <= btsr_pre;
				end
				// Timer 1 OSCX path
				if (t1m[3] && (t1m[1:0] != 2'b11)) begin
					if (t1_oscx_div >= t1_tick_target) begin
						t1_oscx_div <= 8'h00;
						if (t1m[4]) begin
							int_t1 <= 1'b1;
						end else begin
							t1_oscx_256 <= t1_oscx_256 + 8'd1;
							if (t1_oscx_256 == 8'hFF) int_t1 <= 1'b1;
						end
					end else begin
						t1_oscx_div <= t1_oscx_div + 8'd1;
					end
				end
				// Watchdog on the 2 Hz tap
				if (bt_tap_2 && wdt_en) begin
					wdt_count <= wdt_count + 5'd1;
					if (wdt_count == (wdt_ps ? 5'd15 : 5'd3)) begin
						wdt_reset <= 1'b1;
						wdt_count <= 5'd0;
					end
				end
			end else if (btclr) begin
				btsr <= 5'd0;
			end
			if (wdt_kick || !wdt_en) wdt_count <= 5'd0;
			// Savestate restore
			if (ss_wren && (ss_addr == SS_BASE)) begin
				pres <= ss_wdata[57:42];
				t0m  <= ss_wdata[41:34];
				t0c  <= ss_wdata[33:26];
				t1m  <= ss_wdata[25:18];
				t1c  <= ss_wdata[17:10];
				bten <= ss_wdata[9:5];
				btsr <= ss_wdata[4:0];
			end
			if (ss_wren && (ss_addr == SS_BASE + 10'd1)) begin
				prs         <= ss_wdata[59:52];
				t0_count    <= ss_wdata[51:44];
				t1_count    <= ss_wdata[43:36];
				t1_oscx_div <= ss_wdata[35:28];
				t1_oscx_256 <= ss_wdata[27:20];
				bt_chain    <= ss_wdata[19:5];
				wdt_count   <= ss_wdata[4:0];
			end
		end
	end

	// Register read-back
	always @* begin
		case (reg_addr[2:0])
			3'd0: reg_rdata = {3'b111, bten};
			3'd1: reg_rdata = {3'b111, btsr};
			3'd3: reg_rdata = pres[7:0];
			3'd4: reg_rdata = {2'b11, t0m[5:0]};
			3'd5: reg_rdata = t0_count;
			3'd6: reg_rdata = {3'b111, t1m[4:0]};
			3'd7: reg_rdata = t1_count;
			default: reg_rdata = 8'hFF;
		endcase
	end

	// Savestate words: 0 = {pres, t0m, t0c, t1m, t1c, bten, btsr};
	// 1 = {prs, t0_count, t1_count, t1_oscx_div, t1_oscx_256, bt_chain,
	//      wdt_count}
	assign ss_rdata =
		(ss_addr == SS_BASE) ?
			{6'd0, pres, t0m, t0c, t1m, t1c, bten, btsr} :
		(ss_addr == SS_BASE + 10'd1) ?
			{4'd0, prs, t0_count, t1_count, t1_oscx_div, t1_oscx_256,
			 bt_chain, wdt_count} : 64'd0;

	wire unused_ok = &{1'b0, intx_pin, reg_addr[6:3], ss_wdata[63:60]};

endmodule
