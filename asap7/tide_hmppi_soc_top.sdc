# ==============================================================================
# SDC Timing Constraints: Combined TIDE + H-MPPI Integrated SoC
# Target Technology: ASAP7 7nm FinFET (ASAP7SC7p5RVT)
# Clock Spec       : 500.0 MHz (Period: 2.000 ns, 50% duty cycle)
# ==============================================================================

# Primary System Clock
create_clock -period 2.000 -name core_clk -waveform {0.000 1.000} [get_ports clk]

# Clock Uncertainty & Jitter
set_clock_uncertainty 0.050 [get_clocks core_clk]

# Clock Transition / Slew
set_clock_transition 0.040 [get_clocks core_clk]

# Asynchronous Reset Path
set_false_path -from [get_ports rst_n]

# Input / Output Timing Budget (20% of 2.0 ns clock cycle = 0.400 ns)
set_input_delay -clock [get_clocks core_clk] -max 0.400 [get_ports {tide_* mppi_* soc_* ext_*}]
set_input_delay -clock [get_clocks core_clk] -min 0.050 [get_ports {tide_* mppi_* soc_* ext_*}]

set_output_delay -clock [get_clocks core_clk] -max 0.400 [get_ports {tide_* mppi_* soc_*}]
set_output_delay -clock [get_clocks core_clk] -min 0.050 [get_ports {tide_* mppi_* soc_*}]

