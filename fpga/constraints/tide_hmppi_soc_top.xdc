# ==============================================================================
# Timing and Physical Constraints: Combined TIDE + H-MPPI Integrated SoC
# Top Module : tide_hmppi_soc_top
# Target FPGA: Xilinx Artix-7 (xc7a35t-cpg236-1 / xc7a200tfbg676-2)
# Clock Spec : 50.0 MHz (Period: 20.000 ns, 50% duty cycle)
# ==============================================================================

# Primary System Clock
create_clock -period 20.000 -name sys_clk -waveform {0.000 10.000} [get_ports clk]

# Clock Uncertainty & Jitter
set_clock_uncertainty 0.200 [get_clocks sys_clk]

# Asynchronous Reset False Path
set_false_path -from [get_ports rst_n]

# Input / Output Timing Budget (3.0 ns max / 0.5 ns min interface budget)
set_input_delay -clock [get_clocks sys_clk] -max 3.000 [get_ports {tide_* mppi_* soc_* ext_*}]
set_input_delay -clock [get_clocks sys_clk] -min 0.500 [get_ports {tide_* mppi_* soc_* ext_*}]

set_output_delay -clock [get_clocks sys_clk] -max 3.000 [get_ports {tide_* mppi_* soc_*}]
set_output_delay -clock [get_clocks sys_clk] -min 0.500 [get_ports {tide_* mppi_* soc_*}]

