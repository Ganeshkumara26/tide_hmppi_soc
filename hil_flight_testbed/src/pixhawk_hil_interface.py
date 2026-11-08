"""Pixhawk 6C PX4 Hardware-In-The-Loop (HIL) Flight Control Interface.

Implements the native flight control interface:
- High-speed MAVLink v2 telemetry at 921,600 baud
- 100 Hz outer control loop executing trajectory acceleration tracking
- Autonomous Failsafe Supervisor with strict 50 ms timeout watch:
  Seamlessly reverts to internal EKF / level-hover if external ASIC stream stalls > 50 ms.
"""

import time
import struct
import math
import numpy as np
from typing import Dict, Any, Tuple, Optional, List

# MAVLink v2 CRC extra table
CRC_EXTRA = {
    0: 50,      # HEARTBEAT
    30: 39,     # ATTITUDE
    32: 185,    # LOCAL_POSITION_NED
    84: 143,    # SET_POSITION_TARGET_LOCAL_NED
    107: 191,   # HIL_ACTUATOR_CONTROLS
    0x4001: 0x5A # Custom RADAR frame
}


def crc_x25_mavlink(data: bytes, extra: int) -> int:
    """Computes MAVLink v2 X.25 checksum including msg CRC extra."""
    crc = 0xFFFF
    for b in bytes(data) + bytes([extra]):
        t = b ^ (crc & 0xFF)
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


class Pixhawk6CHilSupervisor:
    """Flight controller supervisor and 100 Hz outer control loop for Pixhawk 6C."""

    def __init__(self, baud: int = 921600, loop_rate_hz: float = 100.0, timeout_ms: float = 50.0):
        self.baud = baud
        self.loop_rate_hz = loop_rate_hz
        self.loop_dt = 1.0 / loop_rate_hz  # 10.0 ms
        self.timeout_sec = timeout_ms / 1000.0  # 50 ms

        self.last_packet_time = 0.0
        self.failsafe_active = False
        self.failsafe_trip_count = 0
        self.seq = 0

        # Attitude PID gains (inner loop attitude to body rates)
        self.kp_att = np.array([6.5, 6.5, 3.0])  # [roll, pitch, yaw]

        # Statistics
        self.stats = {
            "mavlink_packets_tx": 0,
            "mavlink_packets_rx": 0,
            "crc_errors": 0,
            "timeouts_detected": 0,
            "failsafe_transitions": 0
        }

    def pack_mavlink_v2(self, msg_id: int, payload: bytes) -> bytes:
        """Packs a valid MAVLink v2 frame with 0xFD framing and X.25 CRC."""
        self.seq = (self.seq + 1) & 0xFF
        extra = CRC_EXTRA.get(msg_id, 0)
        # Header: SOF=0xFD, len, incompat_flags=0, compat_flags=0, seq, sys_id=1, comp_id=1, msg_id (3 bytes)
        hdr = bytes([len(payload), 0, 0, self.seq, 1, 1]) + msg_id.to_bytes(3, 'little')
        body = hdr + payload
        crc = crc_x25_mavlink(body, extra)
        frame = b'\xFD' + body + struct.pack('<H', crc)
        self.stats["mavlink_packets_tx"] += 1
        return frame

    def pack_attitude(self, time_boot_ms: int, roll: float, pitch: float, yaw: float, p: float, q: float, r: float) -> bytes:
        """Packs MAVLink v2 ATTITUDE message (id 30)."""
        payload = struct.pack('<Iffffff', time_boot_ms & 0xFFFFFFFF, roll, pitch, yaw, p, q, r)
        return self.pack_mavlink_v2(30, payload)

    def pack_local_position_ned(self, time_boot_ms: int, x: float, y: float, z: float, vx: float, vy: float, vz: float) -> bytes:
        """Packs MAVLink v2 LOCAL_POSITION_NED message (id 32)."""
        payload = struct.pack('<Iffffff', time_boot_ms & 0xFFFFFFFF, x, y, z, vx, vy, vz)
        return self.pack_mavlink_v2(32, payload)

    def pack_accel_target(self, time_boot_ms: int, ax: float, ay: float, az: float) -> bytes:
        """Packs MAVLink v2 SET_POSITION_TARGET_LOCAL_NED message (id 84) containing acceleration targets."""
        # Bitmask 0b0000111111000111: use acceleration only (afx, afy, afz)
        type_mask = 0b0000111111000111
        # Format: time_boot_ms(I), x(f), y(f), z(f), vx(f), vy(f), vz(f), afx(f), afy(f), afz(f), yaw(f), yaw_rate(f), type_mask(H), target_system(B), target_component(B), coordinate_frame(B)
        payload = struct.pack('<IfffffffffHHBB', time_boot_ms & 0xFFFFFFFF, 0, 0, 0, 0, 0, 0, ax, ay, az, type_mask, 0, 1, 1)
        return self.pack_mavlink_v2(84, payload)

    def parse_mavlink_frame(self, frame_bytes: bytes) -> Optional[Dict[str, Any]]:
        """Parses and validates a MAVLink v2 frame."""
        if len(frame_bytes) < 12 or frame_bytes[0] != 0xFD:
            return None
        payload_len = frame_bytes[1]
        if len(frame_bytes) != 12 + payload_len:
            return None
        
        body = frame_bytes[1:-2]
        msg_id = body[6] | (body[7] << 8) | (body[8] << 16)
        extra = CRC_EXTRA.get(msg_id, 0)
        
        want_crc = struct.unpack('<H', frame_bytes[-2:])[0]
        calc_crc = crc_x25_mavlink(body, extra)
        if want_crc != calc_crc:
            self.stats["crc_errors"] += 1
            return None
            
        self.stats["mavlink_packets_rx"] += 1
        payload = body[9:]
        return {"msg_id": msg_id, "payload": payload}

    def update_setpoint_heartbeat(self, sim_time: float) -> None:
        """Resets the watchdog timer upon receiving a valid external control packet."""
        self.last_packet_time = sim_time
        if self.failsafe_active:
            # External stream restored
            self.failsafe_active = False

    def check_failsafe(self, sim_time: float) -> bool:
        """Checks if external stream exceeded 50 ms timeout."""
        if self.last_packet_time > 0.0 and (sim_time - self.last_packet_time) > self.timeout_sec:
            if not self.failsafe_active:
                self.failsafe_active = True
                self.failsafe_trip_count += 1
                self.stats["timeouts_detected"] += 1
                self.stats["failsafe_transitions"] += 1
        return self.failsafe_active

    def compute_control_actuation(
        self,
        sim_time: float,
        u_opt: np.ndarray,
        current_mass: float,
        current_attitude: np.ndarray,
        current_omega: np.ndarray,
        desired_yaw: float = 0.0
    ) -> Tuple[float, np.ndarray]:
        """100 Hz Outer Control Loop: Maps optimal acceleration u_opt to thrust and body rates.
        
        Reverts to safe level-hover if failsafe is triggered.
        Returns: (thrust_n, commanded_body_rates)
        """
        g = 9.80665
        is_failsafe = self.check_failsafe(sim_time)

        if is_failsafe:
            # Revert to Pixhawk onboard position hold / auto-level EKF
            # Safe hover: 0 lateral accel, gravity cancellation only
            u_cmd = np.array([0.0, 0.0, 0.0])
            phi_des = 0.0
            theta_des = 0.0
            psi_des = current_attitude[2]  # hold current heading
            thrust_n = current_mass * g
        else:
            u_cmd = np.copy(u_opt)
            # Desired acceleration in NED
            a_des = u_cmd - np.array([0.0, 0.0, g])
            thrust_n = current_mass * float(np.linalg.norm(a_des))
            
            # Attitude setpoint extraction via exact flight dynamics inversion
            psi_des = desired_yaw
            ax_prime = u_cmd[0] * math.cos(psi_des) + u_cmd[1] * math.sin(psi_des)
            ay_prime = -u_cmd[0] * math.sin(psi_des) + u_cmd[1] * math.cos(psi_des)
            az_prime = u_cmd[2] - g  # negative downwards

            # Pitch down (-ax_prime) for forward acceleration, roll right (+ay_prime) for rightward acceleration
            theta_des = math.atan2(-ax_prime, -az_prime)
            phi_des = math.atan2(ay_prime, math.sqrt(ax_prime**2 + az_prime**2))

            # Clamp tilt angles to +/- 35 deg (+/- 0.61 rad) for flight safety
            max_tilt = 0.61
            theta_des = max(-max_tilt, min(max_tilt, theta_des))
            phi_des = max(-max_tilt, min(max_tilt, phi_des))

        # Inner attitude rate PID
        att_err = np.array([phi_des, theta_des, psi_des]) - current_attitude
        # Wrap yaw error to [-pi, pi]
        att_err[2] = (att_err[2] + math.pi) % (2.0 * math.pi) - math.pi
        
        commanded_rates = self.kp_att * att_err
        # Rate limit commanded rates
        commanded_rates = np.clip(commanded_rates, -4.0, 4.0)

        return thrust_n, commanded_rates
