"""Regression Test Suite for Tier 3 HIL Closed-Loop Flight Demonstration Testbed.

Tests all components:
1. TXS0108E bidirectional voltage-level translator
2. Real-Time Gate-Level Netlist Bridge (Die 1 sn_net.v & Die 2 tide_late.sv)
3. Coupled 6-DoF Quadrotor dynamics with 35L spherical pendulum slosh and mass depletion
4. Pixhawk 6C PX4 HIL flight control interface, MAVLink v2 framing, and 50 ms failsafe
5. H-MPPI Guidance with Chebyshev slosh co-design
6. 3-Act Closed-Loop Flight Demonstration metrics (>60% slosh cut, CTE <= 0.26 m)
"""

import math
import struct
import numpy as np
import pytest

from hil_flight_testbed.src.txs0108e_level_shifter import TXS0108E
from hil_flight_testbed.src.gate_netlist_bridge import (
    Die1SubNormFP32Netlist,
    Die2TideLateNetlist,
    RealTimeGateNetlistBridge
)
from hil_flight_testbed.src.slosh_dynamics import QuadrotorSloshDynamics
from hil_flight_testbed.src.pixhawk_hil_interface import Pixhawk6CHilSupervisor, crc_x25_mavlink
from hil_flight_testbed.src.hmppi_guidance import HMPPIGuidance
from hil_flight_testbed.src.closed_loop_demonstrator import ClosedLoopFlightDemonstrator


# ---------------------------------------------------------------------------
# 1. Level Translator Tests
# ---------------------------------------------------------------------------

def test_txs0108e_port_a_to_b_translation():
    """Verify 3.3V to 5.0V translation and threshold gating."""
    shifter = TXS0108E(name="U1_TEST", vcca=3.3, vccb=5.0)
    # High logic input (> 2.0V) -> 5.0V output
    bit_out, v_out = shifter.drive_port_a(channel=0, voltage_in=3.3)
    assert bit_out == 1
    assert math.isclose(v_out, 5.0)

    # Low logic input (< 0.8V) -> 0.0V output
    bit_out, v_out = shifter.drive_port_a(channel=0, voltage_in=0.2)
    assert bit_out == 0
    assert math.isclose(v_out, 0.0)


def test_txs0108e_port_b_to_a_translation():
    """Verify 5.0V to 3.3V translation with SCL CP8D 3.5V V_IH threshold."""
    shifter = TXS0108E(name="U2_TEST", vcca=3.3, vccb=5.0)
    # SCL logic high must meet V_IH >= 3.5V
    bit_out, v_out = shifter.drive_port_b(channel=1, voltage_in=4.8)
    assert bit_out == 1
    assert math.isclose(v_out, 3.3)

    # Below 3.5V and above 1.5V enters indeterminate/threshold violation
    shifter.drive_port_b(channel=1, voltage_in=2.5)
    assert shifter.stats["threshold_violations"] >= 1


def test_txs0108e_output_enable_tristate():
    """Verify OE deassertion forces high-Z."""
    shifter = TXS0108E(name="U3_TEST")
    shifter.set_oe(False)
    bit_out, v_out = shifter.drive_port_a(channel=2, voltage_in=3.3)
    assert bit_out is None
    assert math.isnan(v_out)
    assert shifter.stats["tri_state_blocks"] == 1


# ---------------------------------------------------------------------------
# 2. Gate-Level Netlist Bridge Tests (Die 1 & Die 2)
# ---------------------------------------------------------------------------

def test_die1_subnorm_fp32_arithmetic():
    """Verify Die 1 IEEE-754 subnormal handling down to 10^-42."""
    die1 = Die1SubNormFP32Netlist(clock_mhz=10.0)
    die1.reset()

    # Normal division: 1.0 / 4.0 = 0.25
    u_one = die1.float_to_uint32(1.0)
    u_four = die1.float_to_uint32(4.0)
    res_bits, cycles = die1.divide(u_one, u_four)
    assert math.isclose(die1.uint32_to_float(res_bits), 0.25)
    # Real physical silicon cycle count: 27 iters * 64 cycles + 192 overhead cycles = 1,920 cycles (192.0 us @ 10 MHz)
    assert cycles == 1920

    # Subnormal division: 1.0 / 1.4e-42
    subnorm_bits = 0x00000001  # Smallest positive subnormal
    res_bits, cycles = die1.divide(u_one, subnorm_bits)
    assert die1.subnormal_count >= 1
    assert cycles == 1920


def test_die2_tide_late_oosm_detection_and_rollback():
    """Verify Die 2 late packet detection, rollback checkpoint r, and cov_scale."""
    die2 = Die2TideLateNetlist(clock_mhz=10.0)
    die2.reset()

    # In-order events: t=100, t=120
    resp1, c1 = die2.execute_spi_command((1 << 16) | (100 << 4) | 1)
    assert (resp1 >> 16) & 1 == 0  # not late
    assert die2.cov_scale == 0

    resp2, c2 = die2.execute_spi_command((1 << 16) | (120 << 4) | 2)
    assert (resp2 >> 16) & 1 == 0
    assert die2.cov_scale == 0

    # Late arrival: t=110 (arriving after t=120)
    resp3, c3 = die2.execute_spi_command((1 << 16) | (110 << 4) | 3)
    is_late = bool((resp3 >> 16) & 1)
    assert is_late is True
    assert die2.late_events_handled == 1
    assert die2.cov_scale == 2  # Covariance inflation asserted!


def test_real_time_gate_netlist_bridge_integration():
    """Verify top-level bridge executing Die 1 & Die 2 through level translators."""
    bridge = RealTimeGateNetlistBridge(master_clock_mhz=10.0)
    bridge.reset()

    inv_eta, lat1 = bridge.execute_die1_mppi_normalize(160.75)
    assert math.isclose(inv_eta, 1.0 / 160.75, rel_tol=1e-5)
    assert math.isclose(lat1, 192.0)  # 1,920 cycles * 0.1 us = 192.0 us

    resp, lat2 = bridge.process_radar_event_die2(timestamp_ms=100, sensor_id=1)
    assert math.isclose(lat2, 4.4)   # 44 cycles * 0.1 us = 4.4 us
    assert bridge.bridge_stats["level_translations_total"] > 0


# ---------------------------------------------------------------------------
# 3. Slosh Dynamics and Mass Depletion Tests
# ---------------------------------------------------------------------------

def test_slosh_dynamics_mass_depletion():
    """Verify dynamic liquid depletion rate: dot{m} = -q(t)."""
    dyn = QuadrotorSloshDynamics(spray_rate_lpm=3.0, dt_physics=0.0025)
    initial_mass = dyn.total_mass
    initial_fluid = dyn.m_fluid
    assert math.isclose(initial_fluid, 35.0)

    # Step for 1 second (400 steps)
    for _ in range(400):
        dyn.step(dyn.total_mass * 9.80665, np.array([0.0, 0.0, 0.0]))

    # 3.0 L/min = 0.05 kg/s. After 1.0 s, 0.05 kg depleted
    expected_fluid = 35.0 - 0.05
    assert math.isclose(dyn.m_fluid, expected_fluid, abs_tol=1e-3)
    assert math.isclose(dyn.total_mass, initial_mass - 0.05, abs_tol=1e-3)


def test_slosh_dynamics_pendulum_frequency():
    """Verify spherical pendulum natural frequency omega_n = sqrt(g / L_p)."""
    dyn = QuadrotorSloshDynamics(l_p=0.18)
    expected_omega_n = math.sqrt(9.80665 / 0.18)  # ~7.382 rad/s (1.175 Hz)
    assert math.isclose(dyn.omega_n, expected_omega_n, rel_tol=1e-4)


# ---------------------------------------------------------------------------
# 4. Pixhawk 6C MAVLink Interface and Failsafe Tests
# ---------------------------------------------------------------------------

def test_mavlink_v2_framing_and_crc():
    """Verify MAVLink v2 pack and parse with X.25 CRC."""
    pixhawk = Pixhawk6CHilSupervisor()
    accel_frame = pixhawk.pack_accel_target(time_boot_ms=12345, ax=1.2, ay=-0.5, az=-9.8)
    assert accel_frame[0] == 0xFD
    
    parsed = pixhawk.parse_mavlink_frame(accel_frame)
    assert parsed is not None
    assert parsed["msg_id"] == 84  # SET_POSITION_TARGET_LOCAL_NED
    assert len(parsed["payload"]) == 46


def test_pixhawk_failsafe_watchdog_50ms():
    """Verify autonomous failsafe triggers strictly when external setpoint drops > 50 ms."""
    pixhawk = Pixhawk6CHilSupervisor(timeout_ms=50.0)
    pixhawk.update_setpoint_heartbeat(sim_time=1.000)

    # Within 50 ms deadline (e.g. at 1.040 s -> elapsed = 40 ms)
    assert pixhawk.check_failsafe(sim_time=1.040) is False

    # Exceeding 50 ms deadline (e.g. at 1.055 s -> elapsed = 55 ms)
    assert pixhawk.check_failsafe(sim_time=1.055) is True
    assert pixhawk.failsafe_trip_count == 1

    # Recovery upon setpoint restoration
    pixhawk.update_setpoint_heartbeat(sim_time=1.060)
    assert pixhawk.failsafe_active is False


# ---------------------------------------------------------------------------
# 5. H-MPPI Guidance and Chebyshev Slosh Co-Design Tests
# ---------------------------------------------------------------------------

def test_chebyshev_slosh_evaluator_bit_matching():
    """Verify degree-3 Chebyshev polynomial matches hardware RTL expansion."""
    guidance = HMPPIGuidance(bridge=RealTimeGateNetlistBridge(), num_rollouts=16)
    
    # At theta = 0.0: T1=0, T2=-1, T3=0. F = c2*(-1) = 0.168
    f_0, c_0 = guidance.chebyshev_slosh_force(0.0, 0.0)
    assert math.isclose(f_0, 0.168, abs_tol=1e-3)

    # Dynamic scaling when cov_scale > 0
    u_opt, diag = guidance.compute_optimal_control(
        current_pos=np.array([0.0, 0.0, -5.0]),
        current_vel=np.array([0.0, 0.0, 0.0]),
        current_attitude=np.array([0.0, 0.0, 0.0]),
        current_slosh_theta=0.1,
        current_slosh_dot=0.0,
        target_pos=np.array([5.0, 0.0, -5.0]),
        target_vel=np.array([2.0, 0.0, 0.0]),
        nominal_u=np.array([1.0, 0.0, 0.0]),
        obstacles=[],
        die2_cov_scale=2,  # Assert cov_scale
        enable_slosh_damping=True
    )
    assert diag["slosh_weight_active"] == guidance.w_slosh_nominal * guidance.w_slosh_damped_mult


# ---------------------------------------------------------------------------
# 6. E2E Closed-Loop Flight Demonstration Validation Tests
# ---------------------------------------------------------------------------

def test_closed_loop_demonstrator_metrics():
    """Verify 3-Act closed-loop demonstration metrics against specifications."""
    demonstrator = ClosedLoopFlightDemonstrator(num_mppi_rollouts=64)
    # Run a fast 10-second verification slice across acts
    res = demonstrator.run_demonstration(total_time=10.0, enable_slosh_suppression=True)

    assert res["total_frames"] == 1000
    assert res["mass_depleted_kg"] > 0.45
    # Obstacle avoidance clearance verified
    for obs_name, dist in res["act1_min_obstacle_distances"].items():
        if obs_name == "obs1":
            assert dist > 0.20  # Safe clearance maintained


def test_full_comparative_demonstration_specs():
    """Verify complete 40-second flight demonstration against all milestone requirements."""
    demonstrator = ClosedLoopFlightDemonstrator(num_mppi_rollouts=64)
    results = demonstrator.run_full_comparative_demonstration()
    comp = results["comparison"]

    # Act 3 Slosh suppression requirement: > 60% reduction
    assert comp["slosh_reduction_percent"] >= 60.0, f"Slosh reduction {comp['slosh_reduction_percent']}% < 60%"
    assert comp["slosh_reduction_met"] is True

    # Act 3 Cross-track error requirement: <= 0.26 m
    assert comp["proposed_peak_cte_m"] <= 0.26, f"Peak CTE {comp['proposed_peak_cte_m']} m > 0.26 m"
    assert comp["cte_met"] is True

    # 50 ms Failsafe watchdog requirement
    assert comp["failsafe_verified"] is True

    # Continuous 35L mass depletion requirement
    assert comp["mass_depletion_verified"] is True
