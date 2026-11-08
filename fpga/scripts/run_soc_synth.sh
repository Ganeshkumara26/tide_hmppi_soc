#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FPGA_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "================================================================="
echo "  Starting Xilinx Artix-7 Synthesis for TIDE + H-MPPI SoC Top"
echo "  Working directory: $FPGA_DIR"
echo "================================================================="

cd "$FPGA_DIR"

/home/ganeshkumara/oss-cad-suite/bin/yosys -l reports/yosys_synth.log -s scripts/synth_artix7.ys

echo "================================================================="
echo "  Yosys Synthesis Complete. Log saved to reports/yosys_synth.log"
echo "================================================================="

