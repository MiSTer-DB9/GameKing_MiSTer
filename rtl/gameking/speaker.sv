// Copyright (c) 2026 Jamie Blanks
//
// GameKing speaker path.
//
// The PSGO/PSGOB pad pair drives the speaker push-pull through the volume
// pot, so the acoustic signal is the pin difference mechanically low-passed
// by the speaker. Model: a 128-sample boxcar accumulate-and-dump over the
// 6 MHz pin stream (one dump per 46.9 kHz PWM frame, so full PWM periods
// integrate exactly), followed by a one-pole low-pass to settle the
// 187.5 kHz volume-PWM interleave. Shifts and adds only.
//
// Output is signed 16-bit PCM; the framework resamples from ~46.9 kHz.

module speaker
(
	input  wire        clk,
	input  wire        ce,             // 6 MHz
	input  wire        reset,

	input  wire        psgo,
	input  wire        psgob,

	output reg  signed [15:0] pcm
);

	// Pin difference: +1, 0, -1
	wire signed [1:0] diff = {1'b0, psgo} - {1'b0, psgob};

	reg signed [8:0]  acc;       // +-128 range
	reg [6:0]         phase;
	reg signed [15:0] boxcar;    // dumped value, scaled to 16 bits
	reg signed [15:0] lp;
	wire signed [15:0] acc_ext = {{7{acc[8]}}, acc};

	always @(posedge clk) begin
		if (reset) begin
			acc    <= 9'sd0;
			phase  <= 7'd0;
			boxcar <= 16'sd0;
			lp     <= 16'sd0;
			pcm    <= 16'sd0;
		end else if (ce) begin
			phase <= phase + 7'd1;
			if (phase == 7'd127) begin
				// Dump: acc in [-128,+128], x128 = half-scale 16-bit
				boxcar <= acc_ext <<< 7;
				acc    <= {{7{diff[1]}}, diff};
			end else begin
				acc <= acc + {{7{diff[1]}}, diff};
			end
			// One-pole low-pass: lp += (x - lp)/4
			lp  <= lp + ((boxcar - lp) >>> 2);
			pcm <= lp;
		end
	end

endmodule
