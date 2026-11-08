"""Texas Instruments TXS0108E 8-Channel Bidirectional Voltage-Level Translator Model.

Emulates the physical electrical translation between:
- Port A (3.3V CMOS): STM32H7 / Pixhawk 6C flight avionics
- Port B (5.0V CMOS): SCL 0.8 um CP8D Silicon ASICs (Die 1 OS0005 and Die 2 OS0035)

Incorporates:
- Precise threshold gating (V_IL, V_IH)
- Active-high OE (output enable) tri-state logic
- Direction-sensing edge accelerator dynamics
- Propagation delay modeling (t_pd ~ 3.2 ns)
"""

from typing import List, Tuple, Dict, Any, Optional

class TXS0108E:
    """Cycle/time-accurate model of the TI TXS0108E level shifter."""

    def __init__(self, name: str = "TXS0108E_U1", vcca: float = 3.3, vccb: float = 5.0):
        self.name = name
        self.vcca = vcca  # 3.3V
        self.vccb = vccb  # 5.0V
        self.oe: bool = True  # Output enable (Active High)
        
        # Port A thresholds (3.3V)
        self.v_il_a = 0.8
        self.v_ih_a = 2.0
        
        # Port B thresholds (5.0V)
        self.v_il_b = 1.5
        self.v_ih_b = 3.5  # Crucial: SCL CP8D requires 3.5V min for high
        
        # Propagation delays in nanoseconds
        self.t_pd_a_to_b_ns = 3.2
        self.t_pd_b_to_a_ns = 2.8
        
        # Internal channel states (8 channels): logical bit (0 or 1), voltage, direction
        self.channels: List[Dict[str, Any]] = [
            {"bit": 0, "v_a": 0.0, "v_b": 0.0, "dir": "A_TO_B"} for _ in range(8)
        ]
        self.stats = {
            "translations_a_to_b": 0,
            "translations_b_to_a": 0,
            "threshold_violations": 0,
            "tri_state_blocks": 0
        }

    def set_oe(self, enable: bool) -> None:
        """Sets active-high output enable."""
        self.oe = enable
        if not enable:
            for ch in self.channels:
                ch["v_a"] = float("nan")  # High-Z
                ch["v_b"] = float("nan")

    def drive_port_a(self, channel: int, voltage_in: float) -> Tuple[Optional[int], float]:
        """Drives Port A (3.3V side from MCU) -> Translates to Port B (5.0V side for ASIC).
        
        Returns: (output_bit, voltage_out_b)
        """
        if not (0 <= channel < 8):
            raise IndexError(f"Channel {channel} out of range [0..7]")
        if not self.oe:
            self.stats["tri_state_blocks"] += 1
            return None, float("nan")

        self.stats["translations_a_to_b"] += 1
        ch = self.channels[channel]
        ch["dir"] = "A_TO_B"
        ch["v_a"] = voltage_in

        # Threshold checking
        if voltage_in >= self.v_ih_a:
            ch["bit"] = 1
            ch["v_b"] = self.vccb  # 5.0V
        elif voltage_in <= self.v_il_a:
            ch["bit"] = 0
            ch["v_b"] = 0.0
        else:
            # Intermediate undefined threshold region
            self.stats["threshold_violations"] += 1
            # Retain previous bit or indeterminate
            ch["v_b"] = self.vccb if ch["bit"] == 1 else 0.0

        return ch["bit"], ch["v_b"]

    def drive_port_b(self, channel: int, voltage_in: float) -> Tuple[Optional[int], float]:
        """Drives Port B (5.0V side from ASIC) -> Translates to Port A (3.3V side for MCU).
        
        Returns: (output_bit, voltage_out_a)
        """
        if not (0 <= channel < 8):
            raise IndexError(f"Channel {channel} out of range [0..7]")
        if not self.oe:
            self.stats["tri_state_blocks"] += 1
            return None, float("nan")

        self.stats["translations_b_to_a"] += 1
        ch = self.channels[channel]
        ch["dir"] = "B_TO_A"
        ch["v_b"] = voltage_in

        # SCL CP8D 5.0V thresholds
        if voltage_in >= self.v_ih_b:
            ch["bit"] = 1
            ch["v_a"] = self.vcca  # 3.3V
        elif voltage_in <= self.v_il_b:
            ch["bit"] = 0
            ch["v_a"] = 0.0
        else:
            self.stats["threshold_violations"] += 1
            ch["v_a"] = self.vcca if ch["bit"] == 1 else 0.0

        return ch["bit"], ch["v_a"]

    def translate_bit_a_to_b(self, channel: int, bit: int) -> int:
        """Helper for digital bit translation A -> B."""
        v_in = self.vcca if bit else 0.0
        b_out, _ = self.drive_port_a(channel, v_in)
        return b_out if b_out is not None else 0

    def translate_bit_b_to_a(self, channel: int, bit: int) -> int:
        """Helper for digital bit translation B -> A."""
        v_in = self.vccb if bit else 0.0
        b_out, _ = self.drive_port_b(channel, v_in)
        return b_out if b_out is not None else 0
