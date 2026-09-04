// Copyright (c) 2026 Jamie Blanks
//
// ST2204 UART/BRG/IrDA region ($60-$67): inert latches with the documented
// read-back masks. Nothing on the GameKing drives the link port, so there is
// no shifter and TXD idles high.

module st_uart
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	input  wire        sel,
	input  wire [6:0]  reg_addr,
	input  wire        reg_we,
	input  wire [7:0]  reg_wdata,
	output reg  [7:0]  reg_rdata,

	input  wire        rxd,
	output wire        txd
);

	reg [3:0] uctr;         // $60 mask $0F
	reg [7:0] usr;          // $61
	reg [7:0] irctr;        // $62 mask $C7
	reg [7:0] bctr;         // $63 mask $87
	reg [7:0] brs;          // $66
	reg [7:0] bdiv;         // $67

	assign txd = 1'b1;

	always @(posedge clk) begin
		if (reset) begin
			uctr  <= 4'h0;
			usr   <= 8'h00;
			irctr <= 8'h00;
			bctr  <= 8'h00;
			brs   <= 8'h00;
			bdiv  <= 8'h00;
		end else if (ce) begin
			if (sel && reg_we) begin
				case (reg_addr[2:0])
					3'd0: uctr  <= reg_wdata[3:0];
					3'd1: usr   <= (usr & 8'h7A) | (reg_wdata & 8'h05);
					3'd2: irctr <= reg_wdata & 8'hC7;
					3'd3: bctr  <= reg_wdata & 8'h87;
					3'd6: brs   <= reg_wdata;
					3'd7: bdiv  <= reg_wdata;
					default: ;
				endcase
			end
		end
	end

	always @* begin
		case (reg_addr[2:0])
			3'd0: reg_rdata = {4'hF, uctr};
			3'd1: reg_rdata = usr | 8'h80;
			3'd2: reg_rdata = irctr | 8'h38;
			3'd3: reg_rdata = bctr | 8'h78;
			3'd6: reg_rdata = brs;
			3'd7: reg_rdata = bdiv;
			default: reg_rdata = 8'hFF;
		endcase
	end

	wire unused_ok = &{1'b0, rxd, reg_addr[6:3]};

endmodule
