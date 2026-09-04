// Copyright (c) 2026 Jamie Blanks
//
// ST2204 interrupt controller: IREQ/IENA ($3C-$3F), priority arbitration,
// vector address generation into the $7Fxx program window, in-service
// tracking for the IRR bank override.
//
// Pending & enabled drives a level-sensitive IRQ. Priority: the IREQ high
// byte outranks the low byte, then the lowest set bit wins. A pending bit
// auto-clears on the vector high-byte fetch; irq_service sets at the low-byte
// fetch of any non-reset vector and clears on RTI. Writing 0 bits to IREQ
// clears them.
//
// The CPU issues standard 65C02 vector addresses ($FFFA/$FFFC/$FFFE); this
// block remaps them into the $7Fxx window and substitutes the per-level IRQ
// vector ($7FF8 - 2*level) for hardware interrupts, which puts the LCD frame
// interrupt (level 6) on $7FEC.

module st_irq
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

	// Interrupt sources (level pulses, ce-qualified)
	input  wire        int_intx,       // level 0: PC0 edge
	input  wire        int_dac,        // level 1: PSG/DAC reload
	input  wire        int_t0,         // level 2
	input  wire        int_t1,         // level 3
	input  wire        int_pa,         // level 4: port A transition
	input  wire        int_bt,         // level 5: base timer
	input  wire        int_lcd,        // level 6: LCD frame

	// CPU interface
	output wire        irq_n,
	input  wire        vector_pull,    // CPU fetching $FFFA-$FFFF
	input  wire        int_seq,        // hardware interrupt (not BRK)
	input  wire [15:0] cpu_addr,
	input  wire        rti_done,
	output reg         irq_service,
	output wire [15:0] vector_addr,    // remapped $7Fxx address for st_bus

	input  wire [9:0]  ss_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata
);

	localparam [11:0] IREQ_MASK = 12'h77F;   // levels 0-6, 8-11 exist

	reg [11:0] ireq;
	reg [11:0] iena;
	reg [3:0]  irq_level;   // latched at vector low fetch

	wire [11:0] active = ireq & iena;
	assign irq_n = ~|active;

	// Priority: high byte (levels 8-11) first, lowest bit wins in each byte.
	reg [3:0] top_level;
	always @* begin
		if      (active[8])  top_level = 4'd8;
		else if (active[9])  top_level = 4'd9;
		else if (active[10]) top_level = 4'd10;
		else if (active[11]) top_level = 4'd11;
		else if (active[0])  top_level = 4'd0;
		else if (active[1])  top_level = 4'd1;
		else if (active[2])  top_level = 4'd2;
		else if (active[3])  top_level = 4'd3;
		else if (active[4])  top_level = 4'd4;
		else if (active[5])  top_level = 4'd5;
		else if (active[6])  top_level = 4'd6;
		else                 top_level = 4'd7;   // spurious -> reserved slot
	end

	// Vector remap. CPU base addresses: $FFFA/B = NMI, $FFFC/D = RESET,
	// $FFFE/F = IRQ/BRK. Hardware IRQ substitutes $7FF8 - 2*level; the
	// level is latched at the low-byte fetch so both bytes agree.
	wire is_reset = (cpu_addr[2:1] == 2'b10);
	wire is_irqv  = (cpu_addr[2:1] == 2'b11);

	// Pending-bit set/clear merge (see clocked block)
	wire [11:0] ireq_set_bits = {5'd0, int_lcd, int_bt, int_pa, int_t1,
		int_t0, int_dac, int_intx} & IREQ_MASK;
	reg [11:0] ireq_clear_mask;
	always @* begin
		ireq_clear_mask = 12'hFFF;
		if (sel && reg_we) begin
			if (reg_addr[1:0] == 2'd0) ireq_clear_mask[7:0]  = reg_wdata;
			if (reg_addr[1:0] == 2'd1) ireq_clear_mask[11:8] = reg_wdata[3:0];
		end
		if (vector_pull && cpu_addr[0] && is_irqv && int_seq) begin
			ireq_clear_mask[irq_level] = 1'b0;   // hardware auto-clear
		end
	end
	wire [3:0] vec_level = cpu_addr[0] ? irq_level : top_level;
	wire [15:0] irq_vec = 16'h7FF8 - {11'd0, vec_level, 1'b0};
	assign vector_addr = (is_irqv && int_seq)
		? {irq_vec[15:1], cpu_addr[0]}
		: {1'b0, cpu_addr[14:0]};   // $FFFx -> $7FFx window address

	always @(posedge clk) begin
		if (reset) begin
			ireq        <= 12'd0;
			iena        <= 12'd0;
			irq_level   <= 4'd0;
			irq_service <= 1'b0;
		end else if (ce) begin
			// One merged update: software write-clears and the hardware
			// auto-clear apply first, then fresh source pulses set (a set
			// arriving in the same cycle as a clear wins, like the silicon
			// OR into the latch).
			ireq <= (ireq & ireq_clear_mask) | ireq_set_bits;
			if (sel && reg_we) begin
				case (reg_addr[1:0])
					2'd2: iena[7:0]  <= reg_wdata & IREQ_MASK[7:0];
					2'd3: iena[11:8] <= reg_wdata[3:0] & IREQ_MASK[11:8];
					default: ;
				endcase
			end
			// Vector fetch side effects
			if (vector_pull && !cpu_addr[0] && !is_reset) begin
				irq_service <= 1'b1;
				if (is_irqv && int_seq) irq_level <= top_level;
			end
			if (rti_done) irq_service <= 1'b0;
		end
		// Savestate restore
		if (ss_wren && (ss_addr == SS_BASE)) begin
			ireq        <= ss_wdata[28:17];
			iena        <= ss_wdata[16:5];
			irq_level   <= ss_wdata[4:1];
			irq_service <= ss_wdata[0];
		end
	end

	always @* begin
		case (reg_addr[1:0])
			2'd0: reg_rdata = ireq[7:0];
			2'd1: reg_rdata = {4'hF, ireq[11:8]};
			2'd2: reg_rdata = iena[7:0];
			2'd3: reg_rdata = {4'hF, iena[11:8]};
		endcase
	end

	// Savestate word: [28:17]=IREQ [16:5]=IENA [4:1]=level [0]=service
	assign ss_rdata = (ss_addr == SS_BASE)
		? {35'd0, ireq, iena, irq_level, irq_service} : 64'd0;

	wire unused_ok = &{1'b0, reg_addr[6:2], cpu_addr[15], irq_vec[0],
		ss_wdata[63:29]};

endmodule
