"""Hardware-Accelerated Model Predictive Path Integral (H-MPPI) Guidance.

Implements the 100 Hz stochastic trajectory optimization engine co-designed with:
- Die 1 (OS0005 SubNorm FP32): Hardware divider computing weight normalization eta^-1
- Die 2 (OS0035 TIDE-LATE): Out-of-sequence measurement (OOSM) uncertainty gateway
  exporting `cov_scale` to dynamically scale the Chebyshev slosh stage cost
- Division-Free degree-3 Chebyshev polynomial fluid slosh evaluator
- Dynamic obstacle avoidance corridor navigation
Vectorized via NumPy for high-performance real-time execution (<1 ms / frame).
"""

import math
import numpy as np
from typing import List, Tuple, Dict, Any, Optional

from .gate_netlist_bridge import RealTimeGateNetlistBridge

class HMPPIGuidance:
    """H-MPPI Stochastic Guidance Engine with Chebyshev Slosh Co-Design."""

    def __init__(
        self,
        bridge: RealTimeGateNetlistBridge,
        num_rollouts: int = 128,
        horizon_steps: int = 16,
        dt_horizon: float = 0.05,
        temperature_lambda: float = 8.0,
        l_p: float = 0.18
    ):
        self.bridge = bridge
        self.num_rollouts = num_rollouts
        self.horizon_steps = horizon_steps
        self.dt = dt_horizon
        self.temperature_lambda = temperature_lambda
        self.l_p = l_p
        self.g = 9.80665
        self.omega_n = math.sqrt(self.g / self.l_p)
        self.zeta = 0.05

        # Cost weights
        self.w_pos = np.array([2.5, 4.0, 3.0])      # Position tracking penalty [x, y, z]
        self.w_vel = np.array([0.8, 1.2, 0.8])      # Velocity tracking penalty
        self.w_ctrl = 0.05                          # Acceleration effort penalty
        self.w_obs = 800.0                          # Obstacle avoidance repulsive penalty
        self.w_slosh_nominal = 30.0                 # Slosh deflection penalty (nominal)
        self.w_slosh_damped_mult = 4.0              # Die 2 cov_scale multiplier (4.0x)

        # Statistics
        self.stats = {
            "frames_computed": 0,
            "die1_calls": 0,
            "die1_subnormals": 0,
            "mean_frame_min_cost": 0.0,
            "damped_frames": 0
        }

    @staticmethod
    def chebyshev_slosh_force(theta: float, theta_dot: float) -> Tuple[float, float]:
        """Evaluates degree-3 Chebyshev polynomial for nonlinear fluid slosh restoration.
        
        Matches `tier1_hmppi_accelerator/rtl/chebyshev_slosh_eval.sv`:
        T1(x) = x
        T2(x) = 2*x^2 - 1
        T3(x) = 2*x*T2 - T1
        F_slosh = c1*T1 + c2*T2 + c3*T3 (c1=1.0, c2=-0.168, c3=0.051)
        Input is clamped to [-1.0, 1.0] matching the hardware Q8.8 normalization.
        """
        x = max(-1.0, min(1.0, float(theta)))
        t1 = x
        t2 = 2.0 * (x**2) - 1.0
        t3 = 2.0 * x * t2 - t1
        c1, c2, c3 = 1.0, -0.168, 0.051
        f_slosh = c1 * t1 + c2 * t2 + c3 * t3
        base_cost = (f_slosh**2) + 0.5 * (float(theta_dot)**2)
        return f_slosh, base_cost

    def compute_optimal_control(
        self,
        current_pos: np.ndarray,
        current_vel: np.ndarray,
        current_attitude: np.ndarray,
        current_slosh_theta: float,
        current_slosh_dot: float,
        target_pos: np.ndarray,
        target_vel: np.ndarray,
        nominal_u: np.ndarray,
        obstacles: List[Dict[str, float]],
        die2_cov_scale: int = 0,
        enable_slosh_damping: bool = True
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """100 Hz MPPI optimization frame returning (u_opt, diagnostics).
        
        Fully vectorized across M rollouts via NumPy for hard real-time determinism.
        """
        M = self.num_rollouts
        H = self.horizon_steps
        dt = self.dt

        # Determine slosh cost weight
        # When Die 2 asserts cov_scale > 0, dynamically increase slosh penalty by 3.0x
        if enable_slosh_damping:
            if die2_cov_scale > 0:
                slosh_weight = self.w_slosh_nominal * self.w_slosh_damped_mult
                self.stats["damped_frames"] += 1
            else:
                slosh_weight = self.w_slosh_nominal
        else:
            slosh_weight = 0.0  # Unsuppressed baseline mode

        # Vectorized state initialization across M rollouts
        p = np.tile(current_pos, (M, 1))      # (M, 3)
        v = np.tile(current_vel, (M, 1))      # (M, 3)
        th = np.full(M, current_slosh_theta)   # (M,)
        th_dot = np.full(M, current_slosh_dot) # (M,)
        total_costs = np.zeros(M, dtype=np.float64)

        # Pre-generate pseudo-random perturbation tensor
        np.random.seed((self.stats["frames_computed"] * 37 + 101) & 0xFFFFFFFF)
        all_perturbations = np.random.normal(0.0, 1.8, (M, H, 3))
        # Attenuate vertical noise to preserve altitude tracking
        all_perturbations[:, :, 2] *= 0.35

        perturbations_0 = all_perturbations[:, 0, :]

        # Forward rollout horizon
        for t in range(H):
            eps_t = all_perturbations[:, t, :]
            u_t = nominal_u + eps_t  # (M, 3)

            # Kinematics forward integration
            p += v * dt
            v += u_t * dt

            # Vectorized Chebyshev slosh evaluation
            x = np.clip(th, -1.0, 1.0)
            t1 = x
            t2 = 2.0 * (x**2) - 1.0
            t3 = 2.0 * x * t2 - t1
            f_slosh = 1.0 * t1 - 0.168 * t2 + 0.051 * t3
            base_slosh_c = (f_slosh**2) + 0.5 * (th_dot**2)

            # Slosh pendulum rate update
            a_lat = u_t[:, 1]
            th_ddot = -2.0 * self.zeta * self.omega_n * th_dot - (self.g / self.l_p) * f_slosh * 0.1 - (a_lat / self.l_p) * 0.1
            th_dot = np.clip(th_dot + th_ddot * dt, -4.0, 4.0)
            th = np.clip(th + th_dot * dt, -1.0, 1.0)

            # Tracking stage cost
            pos_err = p - target_pos
            vel_err = v - target_vel
            c_track = np.sum(self.w_pos * (pos_err**2), axis=1) + np.sum(self.w_vel * (vel_err**2), axis=1)
            c_ctrl = self.w_ctrl * np.sum(u_t**2, axis=1)

            # Obstacle avoidance repulsion
            c_obs = np.zeros(M, dtype=np.float64)
            for obs in obstacles:
                ox, oy, r_obs = obs["x"], obs["y"], obs["r"]
                d_xy = np.sqrt((p[:, 0] - ox)**2 + (p[:, 1] - oy)**2)
                margin = r_obs + 1.2
                viol = np.maximum(0.0, margin - d_xy)
                c_obs += self.w_obs * (viol**2)

            c_slosh = slosh_weight * base_slosh_c
            total_costs += c_track + c_ctrl + c_obs + c_slosh

        # Soft-min weight calculation
        min_cost = float(np.min(total_costs))
        exp_weights = np.exp(-(total_costs - min_cost) / self.temperature_lambda)
        eta = float(np.sum(exp_weights))

        # Invoke Die 1 Hardware Divider for eta^-1
        self.stats["die1_calls"] += 1
        inv_eta, die1_latency_us = self.bridge.execute_die1_mppi_normalize(eta)

        # Normalize weights using Die 1 result
        norm_weights = exp_weights * inv_eta
        norm_sum = float(np.sum(norm_weights))
        if norm_sum > 0:
            norm_weights /= norm_sum

        # Compute optimal acceleration setpoint u_opt
        opt_perturbation = np.sum(norm_weights[:, np.newaxis] * perturbations_0, axis=0)
        u_opt = nominal_u + opt_perturbation

        self.stats["frames_computed"] += 1
        self.stats["mean_frame_min_cost"] = (
            (self.stats["mean_frame_min_cost"] * (self.stats["frames_computed"] - 1) + min_cost)
            / self.stats["frames_computed"]
        )

        diagnostics = {
            "min_cost": min_cost,
            "eta": eta,
            "inv_eta": inv_eta,
            "die1_latency_us": die1_latency_us,
            "slosh_weight_active": slosh_weight,
            "u_opt": np.copy(u_opt),
            "opt_perturbation": np.copy(opt_perturbation)
        }
        return u_opt, diagnostics
