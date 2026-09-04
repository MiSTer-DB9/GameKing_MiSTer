// Copyright (c) 2026 Jamie Blanks
//
// BIOS auto-sleep patch (OSD "Disable Auto-Sleep").
//
// The GameKing's idle "power off" is not a hardware feature. The console
// has no power switch and the BIOS never executes STP: what happens after
// about two minutes of no input is that the BIOS blanks the panel, stops
// the sound and parks the CPU in WAI. Three sites in the resident BIOS
// bank drive it (internal-ROM address = CPU address for that bank):
//
//   $42D7  API $52, the BIOS sleep service: IENA = $14, LCTR = $C0 (panel
//          off), sound off, sleep flags set in $90 so the BIOS main loop
//          stops calling the game. 37 of the 41 known cartridges call it
//          from their own idle timers, so most sleeping starts here.
//   $4886  the boot shell's own idle timer. A deadline byte at $0E10 is
//          set to the 2 Hz base-timer count minus one whenever there is
//          input, so it matches again 255 ticks (127.5 s) later and the
//          branch here falls through into the API $52 call.
//   $6C0D  the wake test in the key scan. Waking needs START held for 255
//          ticks of the 128 Hz counter (~2 s), which reads as a lockup.
//
// Patching makes the shell branch skip the sleep call unconditionally,
// makes API $52 return immediately, and drops the long-press requirement
// so START wakes on the first scan that sees it held. The bytes are
// substituted on the fly as the BIOS is read, so the OSD option takes
// effect immediately and nothing in SDRAM is modified.
//
// The sites are only patched when the loaded image really holds the
// original bytes there, which is checked while the image streams in. Any
// other BIOS dump runs untouched.
//
// Timing: the site decode is registered when the read is launched (the
// address is stable for the whole bus cycle), so only a 2:1 mux sits in
// the returned-data path.

module bios_patch
(
	input  wire        clk,          // SDRAM controller clock
	input  wire        enable,       // OSD: Disable Auto-Sleep

	// BIOS image load (ioctl), used to identify the image
	input  wire        ld_active,    // BIOS download in progress
	input  wire        ld_wr,        // one-clock write strobe
	input  wire [18:0] ld_addr,
	input  wire [7:0]  ld_data,

	// BIOS read path
	input  wire        rd_req,       // read launched this clock
	input  wire [18:0] rd_addr,      // address of that read
	input  wire [7:0]  din,          // byte returned by the SDRAM
	output wire [7:0]  dout
);

	// Patch sites, as flat internal-ROM addresses.
	localparam [18:0] SITE_WAKE_LO = 19'h06C0D;   // BNE over the 2 s hold
	localparam [18:0] SITE_WAKE_HI = 19'h06C0E;
	localparam [18:0] SITE_SHELL   = 19'h04886;   // shell idle-timer branch
	localparam [18:0] SITE_SLEEP   = 19'h042D7;   // API $52 entry

	// Original byte at each site (identity check) and its replacement.
	localparam [7:0] ORIG_WAKE_LO = 8'hD0;        // BNE  -> NOP
	localparam [7:0] ORIG_WAKE_HI = 8'h48;        // (its offset)
	localparam [7:0] ORIG_SHELL   = 8'hD0;        // BNE  -> BRA
	localparam [7:0] ORIG_SLEEP   = 8'hB2;        // LDA ($80) -> RTS

	localparam [7:0] PTCH_WAKE_LO = 8'hEA;
	localparam [7:0] PTCH_WAKE_HI = 8'hEA;
	localparam [7:0] PTCH_SHELL   = 8'h80;
	localparam [7:0] PTCH_SLEEP   = 8'h60;

	// Image identity: one bit per site, set when that site streams in
	// holding its original byte. Cleared at the start of every download.
	reg [3:0] site_seen = 4'd0;
	reg       ld_active_d = 1'b0;
	always @(posedge clk) begin
		ld_active_d <= ld_active;

		if (ld_active && !ld_active_d) begin
			site_seen <= 4'd0;
		end else if (ld_active && ld_wr) begin
			if ((ld_addr == SITE_WAKE_LO) && (ld_data == ORIG_WAKE_LO))
				site_seen[0] <= 1'b1;
			if ((ld_addr == SITE_WAKE_HI) && (ld_data == ORIG_WAKE_HI))
				site_seen[1] <= 1'b1;
			if ((ld_addr == SITE_SHELL) && (ld_data == ORIG_SHELL))
				site_seen[2] <= 1'b1;
			if ((ld_addr == SITE_SLEEP) && (ld_data == ORIG_SLEEP))
				site_seen[3] <= 1'b1;
		end
	end

	wire known_image = &site_seen;
	wire patch_live  = enable && known_image;

	// Decode the launched read; the substituted byte is ready long before
	// the SDRAM returns data for it.
	reg       hit = 1'b0;
	reg [7:0] hit_data = 8'h00;
	always @(posedge clk) begin
		if (rd_req) begin
			hit      <= 1'b0;
			hit_data <= 8'h00;
			if (patch_live) begin
				case (rd_addr)
					SITE_WAKE_LO: begin hit <= 1'b1; hit_data <= PTCH_WAKE_LO; end
					SITE_WAKE_HI: begin hit <= 1'b1; hit_data <= PTCH_WAKE_HI; end
					SITE_SHELL:   begin hit <= 1'b1; hit_data <= PTCH_SHELL;   end
					SITE_SLEEP:   begin hit <= 1'b1; hit_data <= PTCH_SLEEP;   end
					default: ;
				endcase
			end
		end
	end

	assign dout = hit ? hit_data : din;

endmodule
