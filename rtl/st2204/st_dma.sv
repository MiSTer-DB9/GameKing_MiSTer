// Copyright (c) 2026 Jamie Blanks
//
// ST2204 DMA engine ($28-$2D, $36-$37).
//
// Writing DCNTL launches a transfer of count+1 bytes (a 12-bit count, so up
// to 4096). A byte takes two CPU cycles - one read, one write - with the CPU
// halted throughout, as measured on hardware. The 15-bit source walks CPU
// space, or the DMR-banked data window when DMSH bit 7 is set; crossing $FFFF
// wraps to $8000 and increments DMR. The destination increments unless DCNTH
// bit 4 (DMAM) holds it. Source and destination registers update live and are
// readable; the count registers read $FF. A transfer in progress yields its
// cycle to an LCDC fetch, which has bus priority.

module st_dma
#(
	parameter [9:0] SS_BASE = 10'd0
)
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	input  wire        sel,
	input  wire [6:0]  reg_addr,
	input  wire        reg_we,
	input  wire [7:0]  reg_wdata,
	output reg  [7:0]  reg_rdata,

	// Bus mastering (through st_bus)
	output reg         dma_active,
	output wire [15:0] dma_addr,
	output reg  [7:0]  dma_dout,
	output wire        dma_we,
	output wire        dma_use_bank,
	output wire [10:0] dma_bank,
	input  wire [7:0]  bus_rdata,
	input  wire        lcdc_fetch,     // LCDC bus priority: pause transfers

	input  wire [9:0]  ss_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata
);

	reg [15:0] dms;         // $28/$29 source (bit 15 = banked)
	reg [15:0] dmd;         // $2A/$2B destination
	reg [3:0]  dcnth;       // $2D bits 3:0 (persists after transfer)
	reg        dmam;        // $2D bit 4
	reg [10:0] dmr;         // $36/$37 source bank
	reg [12:0] remaining;   // counts count+1 .. 0 during a transfer
	reg        phase_wr;    // 0 = read cycle, 1 = write cycle

	assign dma_use_bank = dms[15];
	assign dma_bank     = dmr;
	assign dma_we       = dma_active && phase_wr;
	assign dma_addr     = phase_wr ? {1'b0, dmd[14:0]} : dms;

	always @(posedge clk) begin
		if (reset) begin
			dms        <= 16'h0000;
			dmd        <= 16'h0000;
			dcnth      <= 4'h0;
			dmam       <= 1'b0;
			dmr        <= 11'h000;
			remaining  <= 13'd0;
			phase_wr   <= 1'b0;
			dma_active <= 1'b0;
			dma_dout   <= 8'h00;
		end else if (ce) begin
			if (dma_active) begin
				// LCDC fetches steal the cycle; hold the current phase.
				if (!lcdc_fetch) begin
					if (!phase_wr) begin
						// Read cycle completes: capture and advance source
						dma_dout <= bus_rdata;
						phase_wr <= 1'b1;
						// The source counter is 15 bits wide; DMSH bit 7
						// is the bank-mode latch, not a counter bit, so
						// the carry out of $7FFF must not reach it.
						if (dms[14:0] == 15'h7FFF) begin
							dms <= {dms[15], 15'h0000};
							if (dms[15]) dmr <= dmr + 11'd1;
						end else begin
							dms <= {dms[15], dms[14:0] + 15'd1};
						end
					end else begin
						// Write cycle completes: advance destination/count
						phase_wr <= 1'b0;
						if (!dmam) dmd <= dmd + 16'd1;
						remaining <= remaining - 13'd1;
						if (remaining == 13'd1) dma_active <= 1'b0;
					end
				end
			end else if (sel && reg_we) begin
				case (reg_addr[3:0])
					4'h8: dms[7:0]   <= reg_wdata;
					4'h9: dms[15:8]  <= reg_wdata;
					4'hA: dmd[7:0]   <= reg_wdata;
					4'hB: dmd[15:8]  <= reg_wdata;
					4'hC: begin
						// Launch: count+1 bytes
						remaining  <= {1'b0, dcnth, reg_wdata} + 13'd1;
						phase_wr   <= 1'b0;
						dma_active <= 1'b1;
					end
					4'hD: begin
						dcnth <= reg_wdata[3:0];
						dmam  <= reg_wdata[4];
					end
					4'h6: dmr[7:0]  <= reg_wdata;
					4'h7: dmr[10:8] <= reg_wdata[2:0];
					default: ;
				endcase
			end
		end
		// Savestate restore
		if (ss_wren && (ss_addr == SS_BASE)) begin
			dms   <= ss_wdata[47:32];
			dmd   <= ss_wdata[31:16];
			dmr   <= ss_wdata[15:5];
			dcnth <= ss_wdata[4:1];
			dmam  <= ss_wdata[0];
		end
		if (ss_wren && (ss_addr == SS_BASE + 10'd1)) begin
			remaining  <= ss_wdata[22:10];
			phase_wr   <= ss_wdata[9];
			dma_active <= ss_wdata[8];
			dma_dout   <= ss_wdata[7:0];
		end
	end

	// $28-$2B and $36/$37 read back live; the count registers read $FF.
	always @* begin
		case (reg_addr[3:0])
			4'h8: reg_rdata = dms[7:0];
			4'h9: reg_rdata = dms[15:8];
			4'hA: reg_rdata = dmd[7:0];
			4'hB: reg_rdata = dmd[15:8];
			4'h6: reg_rdata = dmr[7:0];
			4'h7: reg_rdata = {5'h1F, dmr[10:8]};
			default: reg_rdata = 8'hFF;
		endcase
	end

	// Savestate words: 0 = {dms, dmd, dmr, dcnth, dmam};
	// 1 = {remaining, phase_wr, dma_active, dma_dout}
	assign ss_rdata =
		(ss_addr == SS_BASE) ?
			{16'd0, dms, dmd, dmr, dcnth, dmam} :
		(ss_addr == SS_BASE + 10'd1) ?
			{41'd0, remaining, phase_wr, dma_active, dma_dout} : 64'd0;

	wire unused_ok = &{1'b0, reg_addr[6:4], ss_wdata[63:48]};

endmodule
