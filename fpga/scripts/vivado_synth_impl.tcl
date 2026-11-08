# ==============================================================================
# Vivado Full Flow Script: Combined TIDE + H-MPPI Integrated SoC
# Target Devices : Xilinx Artix-7 xc7a35t-cpg236-1 & xc7a200tfbg676-2
# Flow Stages    : In-Memory Project -> Read Sources -> Synthesis ->
#                  Placement & Physical Opt -> Detailed Route -> Sign-off Reports
# ==============================================================================

set script_dir [file dirname [file normalize [info script]]]
set fpga_dir   [file normalize [file join $script_dir ".."]]
set top_dir    [file normalize [file join $fpga_dir ".."]]
set repo_root  [file normalize [file join $top_dir ".."]]
set reports_dir [file join $fpga_dir "reports"]
set dcp_dir    [file join $fpga_dir "checkpoints"]

file mkdir $reports_dir
file mkdir $dcp_dir

set top_module "tide_hmppi_soc_top"
set target_part_35t  "xc7a35tcpg236-1"
set target_part_200t "xc7a200tfbg676-2"

if {[info exists ::env(TARGET_PART)]} {
    set part $::env(TARGET_PART)
} else {
    set part $target_part_35t
}

puts "========================================================================"
puts "  Combined TIDE + H-MPPI SoC: Artix-7 Implementation Flow"
puts "  Target Part   : $part"
puts "  Top Module    : $top_module"
puts "  Reports Dir   : $reports_dir"
puts "========================================================================"

# Create in-memory project
create_project -in_memory -part $part

# Read TIDE RTL
read_verilog -sv [file join $repo_root "tide_final/tide/02_fp32_arithmetic_lanes/rtl/tide_const_rom.sv"]
read_verilog -sv [file join $repo_root "tide_final/tide/02_fp32_arithmetic_lanes/rtl/tide_fp32_add.sv"]
read_verilog -sv [file join $repo_root "tide_final/tide/02_fp32_arithmetic_lanes/rtl/tide_fp32_divsqrt.sv"]
read_verilog -sv [file join $repo_root "tide_final/tide/02_fp32_arithmetic_lanes/rtl/tide_fp32_intalu.sv"]
read_verilog -sv [file join $repo_root "tide_final/tide/02_fp32_arithmetic_lanes/rtl/tide_fp32_mul.sv"]
read_verilog -sv [file join $repo_root "tide_final/tide/03_regfile_sequencer_ucode/rtl/tide_regfile.sv"]
read_verilog -sv [file join $repo_root "tide_final/tide/03_regfile_sequencer_ucode/rtl/tide_sequencer.sv"]
read_verilog -sv [file join $repo_root "tide_final/tide/03_regfile_sequencer_ucode/rtl/tide_ucode_ram.sv"]
read_verilog -sv [file join $repo_root "tide_final/tide/04_temporal_controller/rtl/tide_temporal.sv"]
read_verilog -sv [file join $repo_root "tide_final/tide/04_temporal_controller/rtl/tide_lsu.sv"]
read_verilog -sv [file join $repo_root "tide_final/tide/04_temporal_controller/rtl/tide_ram.sv"]
read_verilog -sv [file join $repo_root "tide_final/tide/06_engine_top/rtl/tide_engine.sv"]

# Read H-MPPI RTL
read_verilog -sv [file join $repo_root "tier1_hmppi_accelerator/rtl/chebyshev_slosh_eval.sv"]
read_verilog -sv [file join $repo_root "tier1_hmppi_accelerator/rtl/fexp2_unit.sv"]
read_verilog -sv [file join $repo_root "tier1_hmppi_accelerator/rtl/sobol_qmc_4d.sv"]
read_verilog -sv [file join $repo_root "tier1_hmppi_accelerator/rtl/mppi_rollout_lane.sv"]
read_verilog -sv [file join $repo_root "tier1_hmppi_accelerator/rtl/hmppi_core_top.sv"]

# Read SoC Top
read_verilog -sv [file join $top_dir "rtl/tide_hmppi_soc_top.sv"]

# Read Constraints
read_xdc [file join $fpga_dir "constraints/tide_hmppi_soc_top.xdc"]

# Synthesis
puts "------------------------------------------------------------------------"
puts "  Stage 1: Logic Synthesis (Target Clock: 50 MHz / 20.0 ns)"
puts "------------------------------------------------------------------------"
synth_design -top $top_module -part $part -flatten_hierarchy rebuilt -keep_equivalent_registers

# Placement & Routing
puts "------------------------------------------------------------------------"
puts "  Stage 2: Implementation & Timing Closure"
puts "------------------------------------------------------------------------"
opt_design
place_design
phys_opt_design
route_design

# Sign-off Reports
puts "------------------------------------------------------------------------"
puts "  Stage 3: Sign-off Verification Reports"
puts "------------------------------------------------------------------------"
report_utilization -file [file join $reports_dir "impl_utilization_${part}.rpt"]
report_timing_summary -delay_type min_max -report_unconstrained -max_paths 20 -file [file join $reports_dir "impl_timing_summary_${part}.rpt"]
report_power -file [file join $reports_dir "impl_power_${part}.rpt"]
report_drc -file [file join $reports_dir "impl_drc_${part}.rpt"]

puts "========================================================================"
puts "  Combined SoC Implementation on $part Complete Successfully"
puts "========================================================================"

