// Copyright (c) 2026 Jamie Blanks
//
// ST2204 SPI region ($50-$5F): inert latches with the read-back observed on
// real silicon - $50 reads $FF, $51 reads $BE, $52 reads $FF, $53 has its low
// 7 bits writable, and $54-$5F read $FF. The GameKing has nothing on the SPI
// pins, so there is no shifter and the SPI interrupt can never fire.

module st_spi
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	input  wire        sel,
	input  wire [6:0]  reg_addr,
	input  wire        reg_we,
	input  wire [7:0]  reg_wdata,
	output reg  [7:0]  reg_rdata
);

	reg [6:0] r53;

	always @(posedge clk) begin
		if (reset) begin
			r53 <= 7'h00;
		end else if (ce) begin
			if (sel && reg_we && (reg_addr[3:0] == 4'h3)) begin
				r53 <= reg_wdata[6:0];
			end
		end
	end

	always @* begin
		case (reg_addr[3:0])
			4'h1: reg_rdata = 8'hBE;
			4'h3: reg_rdata = {1'b1, r53};
			default: reg_rdata = 8'hFF;
		endcase
	end

	wire unused_ok = &{1'b0, reg_addr[6:4], reg_wdata[7]};

endmodule
