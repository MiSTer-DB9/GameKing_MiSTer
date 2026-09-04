// Copyright (c) 2026 Jamie Blanks
//
// ST2204 bank control and bus decode.
//
// The bank low registers supply the external address high bits, with the
// program window inverting A14 through PRRL bit 0, and the low 3 bits of the
// bank high registers pick the chip enable the access asserts: internal ROM,
// /CE0 (the cart), /CE2/3/4, or "/CE8". During interrupt service, and through
// the vector pull itself, SYS.IRREN swaps the program window over to IRR with
// the chip select forced to internal ROM; PRR is masked for the duration, not
// modified.
//
// Open bus reads return $FF, the observed idle level.

module st_bus
#(
	parameter [9:0] SS_BASE = 10'd0
)
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	// CPU side
	input  wire [15:0] cpu_addr,
	input  wire [7:0]  cpu_dout,
	output reg  [7:0]  cpu_din,
	input  wire        cpu_we,
	input  wire        vector_pull,
	input  wire [15:0] vector_addr,   // remapped $7Fxx address from st_irq
	input  wire        irq_service,
	input  wire        irren,          // SYS bit 1

	// DMA master
	input  wire        dma_active,
	input  wire [15:0] dma_addr,
	input  wire [7:0]  dma_dout,
	input  wire        dma_we,
	input  wire        dma_use_bank,   // source bit 15: translate via DMR
	input  wire [10:0] dma_bank,       // {DMRH[2:0], DMRL[7:0]}

	// Register window peripheral selects ($0000-$007F)
	output wire        sel_gpio,
	output wire        sel_psg,
	output wire        sel_timer,
	output wire        sel_dma,
	output wire        sel_sys,
	output wire        sel_bank,
	output wire        sel_irq,
	output wire        sel_lcdc,
	output wire        sel_spi,
	output wire        sel_uart,
	output wire [6:0]  reg_addr,
	output wire        reg_we,
	output wire        reg_re,
	output wire [7:0]  reg_wdata,
	input  wire [7:0]  gpio_rdata,
	input  wire [7:0]  psg_rdata,
	input  wire [7:0]  timer_rdata,
	input  wire [7:0]  dma_rdata,
	input  wire [7:0]  sys_rdata,
	input  wire [7:0]  irq_rdata,
	input  wire [7:0]  lcdc_rdata,
	input  wire [7:0]  spi_rdata,
	input  wire [7:0]  uart_rdata,

	// Internal RAM ($0080-$27FF as observed)
	output wire        ram_sel,
	output wire [13:0] ram_addr,
	input  wire [7:0]  ram_rdata,

	// Internal mask ROM (512 KB)
	output wire        irom_sel,
	output wire [18:0] irom_addr,
	input  wire [7:0]  irom_rdata,

	// External bus (to SoC pads)
	output wire [22:0] ext_addr,
	output wire        ext_sel,
	output wire [6:0]  ext_cs_n,       // [0]=/CE0 [2]=/CE2 [3]=/CE3 [4]=/CE4 [5]=/CE8
	input  wire [7:0]  ext_rdata,

	// Savestate register bus
	input  wire [9:0]  ss_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata
);

	// Chip-enable select codes (decoded targets)
	localparam [2:0] T_IROM = 3'd0;
	localparam [2:0] T_CE0  = 3'd1;
	localparam [2:0] T_CE2  = 3'd2;
	localparam [2:0] T_CE3  = 3'd3;
	localparam [2:0] T_CE4  = 3'd4;
	localparam [2:0] T_CE8  = 3'd5;
	localparam [2:0] T_NONE = 3'd6;

	// Bank registers
	reg [7:0] irr;      // $31
	reg [7:0] prrl;     // $32: A21-A15 + A14 invert
	reg [2:0] prrh;     // $33 low 3 bits: chip-enable select
	reg [7:0] drrl;     // $34: bit7 = mode select, A21-A15
	reg [2:0] drrh;     // $35 low 3 bits: chip-enable select

	// Active bus master; vector fetches use the st_irq remapped address
	wire [15:0] master_addr = dma_active ? dma_addr
	                        : vector_pull ? vector_addr
	                        : cpu_addr;
	wire        bus_we    = dma_active ? dma_we : cpu_we;
	wire [7:0]  bus_wdata = dma_active ? dma_dout : cpu_dout;

	// Region decode
	wire in_regs = (master_addr[15:7] == 9'd0);
	wire in_ram  = !in_regs && (master_addr < 16'h2800);
	wire in_prog = (master_addr[15:14] == 2'b01);
	wire in_data = master_addr[15];

	// Register window selects
	assign reg_addr  = master_addr[6:0];
	assign reg_we    = in_regs && bus_we && ce;
	assign reg_re    = in_regs && !bus_we && ce;
	assign reg_wdata = bus_wdata;
	assign sel_gpio  = in_regs && (master_addr[6:4] == 3'h0);
	assign sel_psg   = in_regs && (master_addr[6:3] == 4'h2);
	assign sel_timer = in_regs && (master_addr[6:3] == 4'h4);
	assign sel_dma   = in_regs && ((master_addr[6:3] == 4'h5) ||
	                               (master_addr[6:1] == 6'h1B));
	assign sel_sys   = in_regs && ((master_addr[6:0] == 7'h30) ||
	                               (master_addr[6:2] == 5'h0E));
	assign sel_bank  = in_regs && (master_addr[6:0] >= 7'h31) &&
	                              (master_addr[6:0] <= 7'h35);
	assign sel_irq   = in_regs && (master_addr[6:2] == 5'h0F);
	assign sel_lcdc  = in_regs && (master_addr[6:4] == 3'h4);
	assign sel_spi   = in_regs && (master_addr[6:4] == 3'h5);
	assign sel_uart  = in_regs && (master_addr[6:3] == 4'hC);

	assign ram_sel  = in_ram;
	assign ram_addr = master_addr[13:0];

	// PRRH ($33) chip-enable decode
	function [2:0] cs_decode_prog(input [2:0] mode);
		case (mode)
			3'd0: cs_decode_prog = T_IROM;
			3'd1: cs_decode_prog = T_CE0;
			3'd2: cs_decode_prog = T_CE4;
			3'd4: cs_decode_prog = T_CE3;
			3'd5: cs_decode_prog = T_CE2;
			3'd6: cs_decode_prog = T_CE8;
			default: cs_decode_prog = T_NONE;   // 3/7 not bonded
		endcase
	endfunction

	// DRRL bit 7 + DRRH ($35) chip-enable decode
	function [2:0] cs_decode_data(input mode7, input [2:0] mode);
		if (mode7) begin
			case (mode)
				3'd0: cs_decode_data = T_CE0;
				3'd2: cs_decode_data = T_CE2;
				3'd3: cs_decode_data = T_CE3;
				3'd4: cs_decode_data = T_CE4;
				3'd7: cs_decode_data = T_IROM;
				default: cs_decode_data = T_NONE;
			endcase
		end else begin
			cs_decode_data = (mode == 3'd0) ? T_IROM : T_CE8;
		end
	endfunction

	// -------------------------------------------------------------------
	// Window translation
	// -------------------------------------------------------------------
	// Program window: with IRREN set, IRR takes over through interrupt
	// service and the vector pull and forces internal ROM, PRRL/PRRH
	// reading as if masked to 0. Otherwise PRRL provides the bank, with
	// A14 inverted.
	wire use_irr = irren && (irq_service || vector_pull);
	wire [7:0] prog_bank = use_irr ? irr : prrl;
	wire [2:0] prog_tgt_raw = use_irr ? T_IROM[2:0] : cs_decode_prog(prrh);
	wire [21:0] prog_ext_addr = {prog_bank[7:1], ~prog_bank[0], master_addr[13:0]};
	wire [18:0] prog_irom_addr = {prog_bank[4:1], ~prog_bank[0], master_addr[13:0]};

	// Data window: DRRL bit 7 selects between the two DRRH decode tables.
	wire [2:0] data_tgt = cs_decode_data(drrl[7], drrh);
	wire [21:0] data_ext_addr = {drrl[6:0], master_addr[14:0]};
	wire [18:0] data_irom_addr = {drrl[3:0], master_addr[14:0]};

	// DMA banked source: same rules as the data window, via DMR.
	wire [2:0] dmab_tgt = cs_decode_data(dma_bank[7], dma_bank[10:8]);
	wire [21:0] dmab_ext_addr = {dma_bank[6:0], master_addr[14:0]};
	wire [18:0] dmab_irom_addr = {dma_bank[3:0], master_addr[14:0]};

	// Which translation applies this access
	wire dma_banked = dma_active && dma_use_bank && in_data;
	reg [2:0]  tgt;
	reg [21:0] tgt_ext_addr;
	reg [18:0] tgt_irom_addr;
	always @* begin
		if (dma_banked) begin
			tgt           = dmab_tgt;
			tgt_ext_addr  = dmab_ext_addr;
			tgt_irom_addr = dmab_irom_addr;
		end else if (in_prog) begin
			tgt           = prog_tgt_raw;
			tgt_ext_addr  = prog_ext_addr;
			tgt_irom_addr = prog_irom_addr;
		end else begin
			tgt           = data_tgt;
			tgt_ext_addr  = data_ext_addr;
			tgt_irom_addr = data_irom_addr;
		end
	end

	wire ext_window = in_prog || in_data;
	assign irom_sel  = ext_window && (tgt == T_IROM);
	assign irom_addr = tgt_irom_addr;

	// External bus pads: a live decode, like the internal-ROM path. The
	// address and selects are stable for the whole 6 MHz bus cycle and the
	// asynchronous-ROM read returns within it, so registering them here
	// would present the address a full bus cycle late.
	assign ext_addr = {(tgt == T_CE8) ? 1'b1 : 1'b0, tgt_ext_addr};
	assign ext_sel  = ext_window && (tgt != T_IROM) && (tgt != T_NONE);
	assign ext_cs_n = {1'b1,
		~(ext_window && (tgt == T_CE8)),
		~(ext_window && (tgt == T_CE4)),
		~(ext_window && (tgt == T_CE3)),
		~(ext_window && (tgt == T_CE2)),
		1'b1,
		~(ext_window && (tgt == T_CE0))};

	always @(posedge clk) begin
		if (reset) begin
			irr      <= 8'h00;
			prrl     <= 8'h00;
			prrh     <= 3'd0;
			drrl     <= 8'h00;
			drrh     <= 3'd0;
		end else if (ce) begin
			if (reg_we && sel_bank) begin
				case (master_addr[2:0])
					3'd1: irr  <= bus_wdata;
					3'd2: prrl <= bus_wdata;
					3'd3: prrh <= bus_wdata[2:0];
					3'd4: drrl <= bus_wdata;
					3'd5: drrh <= bus_wdata[2:0];
					default: ;
				endcase
			end
		end
		// Savestate restore. ext_addr/ext_sel/ext_cs_n are live decodes
		// of the bank registers and the master address, so restoring the
		// bank registers restores them too; their fields in the savestate
		// word are kept for layout compatibility.
		if (ss_wren && (ss_addr == SS_BASE)) begin
			irr      <= ss_wdata[60:53];
			prrl     <= ss_wdata[52:45];
			prrh     <= ss_wdata[44:42];
			drrl     <= ss_wdata[41:34];
			drrh     <= ss_wdata[33:31];
		end
	end

	// Bank register read-back. PRRH/DRRH unused bits must read 0: the
	// GameKing SDK far-call stubs end with PLY/STY $33, leaving Y equal
	// to this readback, and cart code then uses that Y for (zp),Y stores
	// into the $0104 parameter block (e.g. Penguin cart $4056). Ones in
	// the upper bits turn those stores into stack-frame corruption.
	reg [7:0] bank_rdata;
	always @* begin
		case (reg_addr[2:0])
			3'd1: bank_rdata = irr;
			3'd2: bank_rdata = prrl;
			3'd3: bank_rdata = {5'd0, prrh};
			3'd4: bank_rdata = drrl;
			3'd5: bank_rdata = {5'd0, drrh};
			default: bank_rdata = 8'hFF;
		endcase
	end

	// Read mux, open bus $FF. Reads route combinationally so data launched
	// with the address returns within the same bus cycle.
	always @* begin
		cpu_din = 8'hFF;
		if (in_regs) begin
			if      (sel_gpio)  cpu_din = gpio_rdata;
			else if (sel_psg)   cpu_din = psg_rdata;
			else if (sel_timer) cpu_din = timer_rdata;
			else if (sel_dma)   cpu_din = dma_rdata;
			else if (sel_sys)   cpu_din = sys_rdata;
			else if (sel_bank)  cpu_din = bank_rdata;
			else if (sel_irq)   cpu_din = irq_rdata;
			else if (sel_lcdc)  cpu_din = lcdc_rdata;
			else if (sel_spi)   cpu_din = spi_rdata;
			else if (sel_uart)  cpu_din = uart_rdata;
		end
		else if (in_ram)      cpu_din = ram_rdata;
		else if (irom_sel)    cpu_din = irom_rdata;
		else if (ext_window && (tgt != T_NONE)) cpu_din = ext_rdata;
	end

	// Savestate word: [60:53]=IRR [52:45]=PRRL [44:42]=PRRH [41:34]=DRRL
	// [33:31]=DRRH [30:8]=ext_addr [7]=ext_sel [6:0]=ext_cs_n
	assign ss_rdata = (ss_addr == SS_BASE)
		? {3'd0, irr, prrl, prrh, drrl, drrh, ext_addr, ext_sel, ext_cs_n}
		: 64'd0;

	wire unused_ok = &{1'b0, ss_wdata[63:61], ss_wdata[30:0]};

endmodule
