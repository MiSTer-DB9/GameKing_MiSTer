derive_pll_clocks
derive_clock_uncertainty

# core specific constraints
#
# The console fabric runs at 24 MHz with all console logic paced by 6 MHz
# clock enables, so console paths meet timing single-cycle and need no
# exceptions. The SDRAM controller runs on its own 96 MHz PLL output; the
# exceptions below cover only that domain and its boundary, each justified
# by construction.

# cart_present is quasi-static: it is set once while the core is held in
# reset during a cartridge download and never changes during execution.
set_false_path -from [get_registers {emu|cart_present*}]

# The staged SDRAM request addresses update at least two clocks before the
# request is presented (p*_pend_d delays presentation one cycle behind the
# address capture), so the controller's accept-to-pin cone legitimately has
# two cycles from these registers.

# and, via retiming, the SDRAM output ports) sees data that has been
# stable for at least two cycles before any accept can sample it.


# the same port arrives at CPU pace (16 clocks) or loader handshake pace,
# both >= 2 clocks later, so the tag compare has two cycles at any accept.


# console's consuming clock-enable edge (the deterministic-window design:
# accept at ce+3, publish by ce+14, consumed at ce+16).
