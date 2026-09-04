// Copyright (c) 2026 Jamie Blanks
//
// Sitronix ST2204 system-on-chip (GameKing configuration).
//
// Ports mirror the 106-pad die, with the usual fabric substitutions:
// bidirectional pads split into DIN/DOUT/OE, the crystals arrive as clock
// enables on clk, and the internal 512 KB mask ROM is backed by external
// storage through irom_* (the BIOS image).
//
// Pad groups: PA/PB/PC/PD/PE GPIO, PL (LCD-muxed), the external bus
// A/D//RD//WR, chip select /CS0 (the MMD pad; /CS1-6 leave through PD),
// PSGO/PSGOB, the dedicated LCD pads /POFF //BLANK/LOAD2, and /RESET. The
// TEST/ICE/XMD/MMD-strap pads are omitted: they are unbonded or strapped on
// the GameKing board.

module st2204
(
	input  wire        clk,
	input  wire        ce_osc,         // 6 MHz phase 2 enable (OSCI crystal)
	input  wire        ce_osc_n,       // mid-cycle enable (phase 1)
	input  wire        ce_32k,         // 32.768 kHz enable (OSCX crystal)
	input  wire        stall,          // savestate sleep
	input  wire        stp_nop,        // execute STP as NOP (no power switch)

	input  wire        reset_n,        // /RESET pad, active low

	// GPIO pads
	input  wire [7:0]  pa_din,
	output wire [7:0]  pa_dout,
	output wire [7:0]  pa_oe,
	input  wire [7:0]  pb_din,
	output wire [7:0]  pb_dout,
	output wire [7:0]  pb_oe,
	input  wire [7:0]  pc_din,
	output wire [7:0]  pc_dout,
	output wire [7:0]  pc_oe,
	input  wire [7:0]  pd_din,
	output wire [7:0]  pd_dout,
	output wire [7:0]  pd_oe,
	input  wire [7:0]  pe_din,
	output wire [7:0]  pe_dout,
	output wire [7:0]  pe_oe,

	// Port L pads (GPIO when the LCDC is off, LCD signals when on)
	input  wire [7:0]  pl_din,
	output wire [7:0]  pl_dout,
	output wire [7:0]  pl_oe,

	// External memory bus
	output wire [22:0] ext_a,
	input  wire [7:0]  ext_d_din,
	output wire [7:0]  ext_d_dout,
	output wire        ext_d_oe,
	output wire        rd_n,
	output wire        wr_n,
	output wire        cs0_n,          // MMD pad after reset

	// Audio pads
	output wire        psgo,
	output wire        psgob,

	// Dedicated LCD pads
	output wire        poff_n,
	output wire        blank_n,
	output wire        load2,

	// Internal mask ROM backend (FPGA consideration)
	output wire        irom_req,
	output wire [18:0] irom_addr_o,
	input  wire [7:0]  irom_data,
	input  wire        irom_ready,

	// Savestate spine
	input  wire        ss_active,      // core asleep, memories owned by savestates
	input  wire [9:0]  ss_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata,
	input  wire [13:0] ss_mem_addr,
	input  wire        ss_mem_wren,
	input  wire [7:0]  ss_mem_wdata,
	output wire [7:0]  ss_mem_rdata
);

	// Synchronous active-high reset from the pad. It powers up asserted so
	// the st_ram cold-boot sweep always sees an assertion edge.
	reg reset = 1'b1;
	always @(posedge clk) reset <= ~reset_n;

	// -------------------------------------------------------------------
	// Interconnect
	// -------------------------------------------------------------------
	wire [15:0] cpu_addr;
	wire [7:0]  cpu_dout;
	wire [7:0]  cpu_din;
	wire        cpu_we;
	wire        cpu_sync;
	wire        cpu_vector_pull;
	wire        cpu_ml_n;
	wire        cpu_phi1o;
	wire        cpu_phi2o;
	wire        cpu_bus_oe;
	wire        cpu_dout_oe;
	wire        cpu_int_seq;
	wire [15:0] vector_addr;
	wire        cpu_rti_done;
	wire        cpu_wai;
	wire        cpu_stp;
	wire        irq_n;
	wire        irq_service;
	wire        irren;
	wire        wait_mode;

	wire        sel_gpio, sel_psg, sel_timer, sel_dma, sel_sys;
	wire        sel_bank, sel_irq, sel_lcdc, sel_spi, sel_uart;
	wire [6:0]  reg_addr;
	wire        reg_we;
	wire        reg_re;
	wire [7:0]  reg_wdata;
	wire [7:0]  gpio_rdata, psg_rdata, timer_rdata, dma_rdata, sys_rdata;
	wire [7:0]  irq_rdata, lcdc_rdata_bus, spi_rdata, uart_rdata;

	wire        ram_sel;
	wire [13:0] ram_addr;
	wire [7:0]  ram_rdata;

	wire        irom_sel;
	wire [18:0] irom_addr;
	wire [7:0]  irom_rdata;
	wire        ext_sel;
	wire [6:0]  ext_cs_n;

	wire        dma_active;
	wire [15:0] dma_addr;
	wire [7:0]  dma_dout;
	wire        dma_we;
	wire        dma_use_bank;
	wire [10:0] dma_bank;

	wire        lcdc_fetch;
	wire [15:0] lcdc_fetch_addr;
	wire [7:0]  lcdc_fetch_data;
	wire        lcd_active;
	wire        pin_flm, pin_load1, pin_ac_n, pin_cp;
	wire [3:0]  pin_ld;

	wire        int_dac, int_t0, int_t1, int_pa, int_bt, int_lcd;
	wire        wdt_reset;
	wire        wdt_en, wdt_ps, wdt_kick;

	wire [63:0] ss_rdata_cpu, ss_rdata_bus, ss_rdata_gpio, ss_rdata_irq;
	wire [63:0] ss_rdata_timer, ss_rdata_psg, ss_rdata_dma, ss_rdata_lcdc;
	wire [63:0] ss_rdata_sys;

	// The CPU stalls for DMA, for an LCDC RAM collision, and for savestate
	// sleep. The dual-port RAM serves both sides of a collision, so that
	// term only shapes timing.
	wire cpu_stall = stall | dma_active | (lcdc_fetch && ram_sel);

	// GPIO pads: PL belongs to the LCDC when running (LPWR), else GPIO.
	wire [7:0] gpio_pl_dout;
	wire [7:0] gpio_pl_oe;
	assign pl_dout = lcd_active ? {pin_flm, pin_load1, pin_ac_n, pin_cp, pin_ld}
	                            : gpio_pl_dout;
	assign pl_oe   = lcd_active ? 8'hFF : gpio_pl_oe;
	// Nothing on the GameKing uses PL as GPIO, so it idles as an input.
	assign gpio_pl_dout = 8'hFF;
	assign gpio_pl_oe   = 8'h00;

	// External bus: /RD and /WR frame the second half of the cycle. The
	// strobes and write data must follow the active bus master, since the
	// DMA drives the address and chip select while the CPU is stalled.
	wire       bus_we    = dma_active ? dma_we   : cpu_we;
	wire [7:0] bus_wdata = dma_active ? dma_dout : cpu_dout;
	assign ext_d_dout = bus_wdata;
	assign ext_d_oe   = ext_sel && bus_we;
	assign rd_n       = ~(ext_sel && !bus_we);
	assign wr_n       = ~(ext_sel && bus_we);
	assign cs0_n      = ext_cs_n[0];

	// -------------------------------------------------------------------
	// Blocks
	// -------------------------------------------------------------------
	// The ST2204's core is a 65C02, so it takes the W65C02S bus and flag
	// behaviors rather than the NMOS 6502 ones.
	cpu_65c02 #(.SS_BASE(10'd0), .WDC_MODE(1'b1)) cpu (
		.clk(clk),
		.ce(ce_osc),
		.ce_n(ce_osc_n),
		.reset(reset | wdt_reset),
		.stall(cpu_stall),
		.irq_n(irq_n),
		.nmi_n(1'b1),
		.rdy(1'b1),
		.stp_nop(stp_nop),
		// The ST2204 embeds the core: SOB and BE are not brought out,
		// so SOB idles high and BE stays asserted. ML has no second
		// bus master to arbitrate against here.
		.so_n(1'b1),
		.be(1'b1),
		.addr(cpu_addr),
		.dout(cpu_dout),
		.din(cpu_din),
		.we(cpu_we),
		.sync(cpu_sync),
		.vector_pull(cpu_vector_pull),
		.ml_n(cpu_ml_n),
		.phi1o(cpu_phi1o),
		.phi2o(cpu_phi2o),
		.bus_oe(cpu_bus_oe),
		.dout_oe(cpu_dout_oe),
		.int_seq(cpu_int_seq),
		.rti_done(cpu_rti_done),
		.in_wai(cpu_wai),
		.in_stp(cpu_stp),
		.ss_addr(ss_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wren),
		.ss_rdata(ss_rdata_cpu)
	);

	st_bus #(.SS_BASE(10'd64)) bus (
		.clk(clk),
		.ce(ce_osc),
		.reset(reset | wdt_reset),
		.cpu_addr(cpu_addr),
		.cpu_dout(cpu_dout),
		.cpu_din(cpu_din),
		.cpu_we(cpu_we),
		.vector_pull(cpu_vector_pull),
		.vector_addr(vector_addr),
		.irq_service(irq_service),
		.irren(irren),
		.dma_active(dma_active),
		.dma_addr(dma_addr),
		.dma_dout(dma_dout),
		.dma_we(dma_we),
		.dma_use_bank(dma_use_bank),
		.dma_bank(dma_bank),
		.sel_gpio(sel_gpio),
		.sel_psg(sel_psg),
		.sel_timer(sel_timer),
		.sel_dma(sel_dma),
		.sel_sys(sel_sys),
		.sel_bank(sel_bank),
		.sel_irq(sel_irq),
		.sel_lcdc(sel_lcdc),
		.sel_spi(sel_spi),
		.sel_uart(sel_uart),
		.reg_addr(reg_addr),
		.reg_we(reg_we),
		.reg_re(reg_re),
		.reg_wdata(reg_wdata),
		.gpio_rdata(gpio_rdata),
		.psg_rdata(psg_rdata),
		.timer_rdata(timer_rdata),
		.dma_rdata(dma_rdata),
		.sys_rdata(sys_rdata),
		.irq_rdata(irq_rdata),
		.lcdc_rdata(lcdc_rdata_bus),
		.spi_rdata(spi_rdata),
		.uart_rdata(uart_rdata),
		.ram_sel(ram_sel),
		.ram_addr(ram_addr),
		.ram_rdata(ram_rdata),
		.irom_sel(irom_sel),
		.irom_addr(irom_addr),
		.irom_rdata(irom_rdata),
		.ext_addr(ext_a),
		.ext_sel(ext_sel),
		.ext_cs_n(ext_cs_n),
		.ext_rdata(ext_d_din),
		.ss_addr(ss_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wren),
		.ss_rdata(ss_rdata_bus)
	);

	st_gpio #(.SS_BASE(10'd128)) gpio (
		.clk(clk),
		.ce(ce_osc),
		.reset(reset | wdt_reset),
		.sel(sel_gpio),
		.reg_addr(reg_addr),
		.reg_we(reg_we),
		.reg_re(reg_re),
		.reg_wdata(reg_wdata),
		.reg_rdata(gpio_rdata),
		.pa_din(pa_din),
		.pa_dout(pa_dout),
		.pa_oe(pa_oe),
		.pb_din(pb_din),
		.pb_dout(pb_dout),
		.pb_oe(pb_oe),
		.pc_din(pc_din),
		.pc_dout(pc_dout),
		.pc_oe(pc_oe),
		.pd_din(pd_din),
		.pd_dout(pd_dout),
		.pd_oe(pd_oe),
		.pe_din(pe_din),
		.pe_dout(pe_dout),
		.pe_oe(pe_oe),
		.cs_n(ext_cs_n[6:1]),
		.pa_transition(int_pa),
		.ss_addr(ss_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wren),
		.ss_rdata(ss_rdata_gpio)
	);

	st_irq #(.SS_BASE(10'd192)) irq (
		.clk(clk),
		.ce(ce_osc),
		.reset(reset | wdt_reset),
		.sel(sel_irq),
		.reg_addr(reg_addr),
		.reg_we(reg_we),
		.reg_wdata(reg_wdata),
		.reg_rdata(irq_rdata),
		.int_intx(1'b0),                // PC0 edge, unused on the GameKing
		.int_dac(int_dac),
		.int_t0(int_t0),
		.int_t1(int_t1),
		.int_pa(int_pa),
		.int_bt(int_bt),
		.int_lcd(int_lcd),
		.irq_n(irq_n),
		.vector_pull(cpu_vector_pull),
		.int_seq(cpu_int_seq),
		.cpu_addr(cpu_addr),
		.rti_done(cpu_rti_done),
		.irq_service(irq_service),
		.vector_addr(vector_addr),
		.ss_addr(ss_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wren),
		.ss_rdata(ss_rdata_irq)
	);

	st_timer #(.SS_BASE(10'd256)) timer (
		.clk(clk),
		.ce(ce_osc),
		.ce_32k(ce_32k),
		.reset(reset | wdt_reset),
		.sel(sel_timer),
		.reg_addr(reg_addr),
		.reg_we(reg_we),
		.reg_wdata(reg_wdata),
		.reg_rdata(timer_rdata),
		.intx_pin(pc_din[0]),
		.int_t0(int_t0),
		.int_t1(int_t1),
		.int_bt(int_bt),
		.wdt_reset(wdt_reset),
		.wdt_en(wdt_en),
		.wdt_ps(wdt_ps),
		.wdt_kick(wdt_kick),
		.ss_addr(ss_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wren),
		.ss_rdata(ss_rdata_timer)
	);

	st_psg #(.SS_BASE(10'd320)) psg (
		.clk(clk),
		.ce(ce_osc),
		.ce_32k(ce_32k),
		.reset(reset | wdt_reset),
		.sel(sel_psg),
		.reg_addr(reg_addr),
		.reg_we(reg_we),
		.reg_wdata(reg_wdata),
		.reg_rdata(psg_rdata),
		.psgo(psgo),
		.psgob(psgob),
		.int_dac(int_dac),
		.ss_addr(ss_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wren),
		.ss_rdata(ss_rdata_psg)
	);

	st_dma #(.SS_BASE(10'd384)) dma (
		.clk(clk),
		.ce(ce_osc),
		.reset(reset | wdt_reset),
		.sel(sel_dma),
		.reg_addr(reg_addr),
		.reg_we(reg_we),
		.reg_wdata(reg_wdata),
		.reg_rdata(dma_rdata),
		.dma_active(dma_active),
		.dma_addr(dma_addr),
		.dma_dout(dma_dout),
		.dma_we(dma_we),
		.dma_use_bank(dma_use_bank),
		.dma_bank(dma_bank),
		.bus_rdata(cpu_din),
		.lcdc_fetch(lcdc_fetch),
		.ss_addr(ss_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wren),
		.ss_rdata(ss_rdata_dma)
	);

	st_lcdc #(.SS_BASE(10'd448)) lcdc (
		.clk(clk),
		.ce(ce_osc),
		.reset(reset | wdt_reset),
		.sel(sel_lcdc),
		.reg_addr(reg_addr),
		.reg_we(reg_we),
		.reg_wdata(reg_wdata),
		.reg_rdata(lcdc_rdata_bus),
		.fetch_req(lcdc_fetch),
		.fetch_addr(lcdc_fetch_addr),
		.fetch_data(lcdc_fetch_data),
		.lcd_active(lcd_active),
		.pin_flm(pin_flm),
		.pin_load1(pin_load1),
		.pin_ac_n(pin_ac_n),
		.pin_cp(pin_cp),
		.pin_ld(pin_ld),
		.pin_poff_n(poff_n),
		.pin_blank_n(blank_n),
		.pin_load2(load2),
		.int_lcd(int_lcd),
		.ss_addr(ss_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wren),
		.ss_rdata(ss_rdata_lcdc)
	);

	st_ram ram (
		.clk(clk),
		.reset(reset),
		.sel(ram_sel),
		.addr(ram_addr),
		.we((dma_active ? dma_we : cpu_we) && ce_osc),
		.wdata(dma_active ? dma_dout : cpu_dout),
		.rdata(ram_rdata),
		.lcdc_addr(lcdc_fetch_addr[13:0]),
		.lcdc_rdata(lcdc_fetch_data),
		.ss_active(ss_active),
		.ss_mem_addr(ss_mem_addr),
		.ss_mem_wren(ss_mem_wren),
		.ss_mem_wdata(ss_mem_wdata),
		.ss_mem_rdata(ss_mem_rdata)
	);

	st_rom rom (
		.clk(clk),
		.ce(ce_osc),
		.reset(reset),
		.sel(irom_sel),
		.addr(irom_addr),
		.rdata(irom_rdata),
		.mem_req(irom_req),
		.mem_addr(irom_addr_o),
		.mem_rdata(irom_data),
		.mem_ready(irom_ready)
	);

	st_sys #(.SS_BASE(10'd512)) sys_blk (
		.clk(clk),
		.ce(ce_osc),
		.reset(reset | wdt_reset),
		.sel(sel_sys),
		.reg_addr(reg_addr),
		.reg_we(reg_we),
		.reg_wdata(reg_wdata),
		.reg_rdata(sys_rdata),
		.irren(irren),
		.wait_mode(wait_mode),
		.wdt_en(wdt_en),
		.wdt_ps(wdt_ps),
		.wdt_kick(wdt_kick),
		.ss_addr(ss_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wren),
		.ss_rdata(ss_rdata_sys)
	);

	st_uart uart (
		.clk(clk),
		.ce(ce_osc),
		.reset(reset | wdt_reset),
		.sel(sel_uart),
		.reg_addr(reg_addr),
		.reg_we(reg_we),
		.reg_wdata(reg_wdata),
		.reg_rdata(uart_rdata),
		.rxd(pd_din[7]),
		.txd()
	);

	st_spi spi (
		.clk(clk),
		.ce(ce_osc),
		.reset(reset | wdt_reset),
		.sel(sel_spi),
		.reg_addr(reg_addr),
		.reg_we(reg_we),
		.reg_wdata(reg_wdata),
		.reg_rdata(spi_rdata)
	);

	// Savestate read OR-merge
	assign ss_rdata = ss_rdata_cpu | ss_rdata_bus | ss_rdata_gpio |
		ss_rdata_irq | ss_rdata_timer | ss_rdata_psg | ss_rdata_dma |
		ss_rdata_lcdc | ss_rdata_sys;

	wire unused_ok = &{1'b0, cpu_ml_n, cpu_phi1o, cpu_phi2o, cpu_bus_oe,
		cpu_dout_oe, cpu_sync, cpu_wai, cpu_stp, wait_mode, sel_bank,
		lcdc_fetch_addr[15:14], pl_din};

endmodule
