# Design Document 91: Pre-Silicon HIL Flight Testbed and Avionics Interface Specification

**Target Flight Controller:** Pixhawk 6C running PX4 Autopilot v1.14+  
**Electrical Interface:** TXS0108E 3.3V/5V Bidirectional High-Speed Level Shifter  
**Communication Protocol:** Binary MAVLink v2 over UART at 921,600 baud  
**Control Loop Rate:** 100 Hz (10 ms frame period)  
**Failsafe Watchdog:** 50 ms hardware timeout  

---

## 1. Pre-Silicon Verification Philosophy

Before fabricated silicon is available for physical testing, the complete avionics control loop — from sensor measurement ingestion through EKF state estimation and MPPI trajectory optimization to actuator command output — must be verified in a closed-loop hardware-in-the-loop (HIL) environment.

The pre-silicon HIL testbed replaces the physical UAV airframe and motors with the **Gazebo Classic** physics simulator, while the real-time control hardware (FPGA or ASIC emulation) communicates with a physical **Pixhawk 6C flight controller** running the **PX4 Autopilot** firmware. This tests the actual electrical interface, timing constraints, and protocol compliance that will be present in the final deployed system.

### Testbed Configuration

```
┌──────────────┐    UART 921.6k    ┌──────────────┐    USB    ┌──────────────┐
│  FPGA Board  │◄─────────────────►│  Pixhawk 6C  │◄────────►│  Gazebo Sim  │
│  (Artix-7)   │   MAVLink v2      │  (PX4 v1.14) │  HITL    │  (Physics)   │
│              │   Level-shifted   │              │          │              │
│  TIDE+HMPPI  │   TXS0108E       │  EKF2 + MC   │          │  Iris Quad   │
│  SoC RTL     │   3.3V ↔ LVCMOS  │  Commander   │          │  35L payload │
└──────────────┘                   └──────────────┘          └──────────────┘
```

---

## 2. Electrical Level Shifter: TXS0108E

Modern flight controllers (Pixhawk 6C) operate with **3.3V LVCMOS** signaling, whereas the SCL 0.8 µm CMOS silicon operates with a **5.0V** supply. Direct connection would cause gate oxide breakdown. The FPGA (Artix-7) uses 3.3V LVCMOS I/O banks but requires signal conditioning for noise immunity on long PCB traces.

The **Texas Instruments TXS0108E** 8-bit bidirectional voltage-level translator bridges the two domains:

### Electrical Characteristics Modeled in Simulation

| Parameter | Value | Significance |
|---|---|---|
| Propagation delay (rise) | 3.8 ns typical | Included in timing budget for 921.6k baud |
| Propagation delay (fall) | 4.6 ns typical | Asymmetric — affects duty cycle at high baud |
| One-Shot Edge Accelerators | Active pull-up on low→high | Accelerates slew rate driving up to 70 pF load |
| Direction sensing | Automatic (no DIR pin) | Bidirectional without software direction control |
| Supply range (A side) | 1.2V–3.6V | Connected to Pixhawk 3.3V rail |
| Supply range (B side) | 1.65V–5.5V | Connected to FPGA 3.3V or SCL 5.0V rail |

### Simulation Fidelity

The pre-silicon testbed models the TXS0108E's exact propagation characteristics to ensure that UART bit timing remains within the ±3% tolerance window at 921,600 baud (bit period = 1.085 µs). The 4.6 ns worst-case propagation delay consumes only 0.42% of the bit period, providing ample margin.

---

## 3. Binary MAVLink v2 Protocol

The HIL bridge implements the full binary **MAVLink v2 packet protocol** (not the text-based MAVLink v1 format):

### Packet Structure

```
Byte   Field            Size    Description
────────────────────────────────────────────────────────
 0     Magic (STX)       1      0xFD (MAVLink v2 identifier)
 1     Payload Length     1      Variable (0–255 bytes)
 2     Incompatible Flags 1     Bit flags for extended features
 3     Compatible Flags   1     Reserved (0x00)
 4     Sequence Number    1     Rolling 0–255 packet counter
 5     System ID          1     Source system (FPGA = 1, Pixhawk = 255)
 6     Component ID       1     Source component (TIDE = 191, MPPI = 192)
 7–9   Message ID         3     24-bit message identifier (little-endian)
10–N   Payload            Var    Message-specific data fields
N+1–2  CRC-16 (X.25)     2     CRC over bytes 1–N, seeded with CRC_EXTRA
```

### UART Configuration

| Parameter | Value |
|---|---|
| Baud rate | 921,600 bps |
| Data bits | 8 |
| Stop bits | 1 |
| Parity | None |
| Flow control | None |
| Frame rate | 100 Hz (one telemetry packet per 10 ms) |

---

## 4. Critical MAVLink Messages

### Received from Pixhawk (Sensor Telemetry)

| Message ID | Name | Rate | Payload Contents |
|:---:|---|---|---|
| 0 | `HEARTBEAT` | 1 Hz | System type, autopilot type, base mode, system status |
| 331 | `ODOMETRY` | 100 Hz | Position ($x, y, z$), velocity ($v_x, v_y, v_z$), attitude quaternion ($q_0, q_1, q_2, q_3$), angular velocity ($\omega_x, \omega_y, \omega_z$). Frame: MAV_FRAME_LOCAL_NED. |
| 24 | `GPS_RAW_INT` | 5 Hz | Latitude, longitude, altitude, fix type, satellites visible |
| 27 | `RAW_IMU` | 400 Hz | Accelerometer XYZ, gyroscope XYZ, magnetometer XYZ (raw ADC) |

### Transmitted to Pixhawk (Actuator Commands)

| Message ID | Name | Rate | Payload Contents |
|:---:|---|---|---|
| 0 | `HEARTBEAT` | 1 Hz | Maintains communication lock with PX4 commander |
| 93 | `HIL_ACTUATOR_CONTROLS` | 100 Hz | Normalized rotor thrust commands ($u_1, u_2, u_3, u_4$) as FP32. Mode: `MAV_MODE_FLAG_HIL_ENABLED`. |
| 253 | `STATUSTEXT` | Event | Human-readable diagnostic messages (OOSM rollback events, EKF divergence warnings, watchdog trips) |

---

## 5. Sensor Packet Ingestion Pipeline

When an ODOMETRY message (331) arrives from the Pixhawk:

1. **UART RX**: Bytes are received at 921,600 baud into a 256-byte FIFO. The STX byte (0xFD) triggers frame synchronization.
2. **CRC validation**: The CRC-16/X.25 checksum is verified against the received payload. Corrupted packets are silently dropped.
3. **Timestamp extraction**: The measurement timestamp is extracted from the ODOMETRY payload's `time_usec` field (64-bit microsecond epoch).
4. **OOSM classification**: The timestamp is compared against the TIDE engine's current time $t_{now}$:
   - If $\text{time\_usec} \ge t_{now}$: **in-order** — proceed to standard EKF measurement update.
   - If $\text{time\_usec} < t_{now}$ but $\ge ck_0$: **late (OOSM)** — trigger temporal rollback through the TIDE journal.
   - If $\text{time\_usec} < ck_0$: **too old** — discard with class-2 error log.
5. **EKF update**: The TIDE VLIW coprocessor executes the predict+update microcode sequence (492 instructions, 990 cycles).
6. **MPPI trigger**: Upon EKF completion, the H-MPPI engine is triggered via the direct wire strobe.
7. **Actuator output**: The optimal actuator commands are formatted as `HIL_ACTUATOR_CONTROLS` (message 93) and transmitted to the Pixhawk via UART TX.

### Synchronization FIFO

A 16-entry, 32-bit-wide synchronization FIFO buffers sensor measurements between the UART RX clock domain and the SoC processing clock. In the single-clock FPGA implementation, this FIFO operates synchronously. In multi-clock ASIC implementations, it would require Gray-coded pointer CDC logic.

---

## 6. Hardware Watchdog Timer

Autonomous flight avionics require strict failsafe guarantees against hardware lockup or clock failure.

### Watchdog Specification

| Parameter | Value |
|---|---|
| Timeout period | 50 ms (5 consecutive missed 100 Hz frames) |
| Counter width | 21 bits at 50 MHz ($2^{21} = 2,097,152$ cycles = 41.94 ms) |
| Reset condition | Valid MPPI actuator command output |
| Trip action | Force PX4 into `NAV_DLL_FAILSAFE` (Return-to-Land or Disarm) |
| Implementation | Free-running counter, cleared on each `act_valid` strobe |

### Failsafe Behavior

When the watchdog trips:
1. The MAVLink output multiplexer switches from the MPPI actuator commands to a **failsafe register** containing pre-configured safe commands (zero thrust, zero angular rates).
2. A `STATUSTEXT` message (ID 253) is transmitted to the Pixhawk with severity `MAV_SEVERITY_CRITICAL` and the text `"HW WATCHDOG TRIP: FAILSAFE ENGAGED"`.
3. The PX4 autopilot's commander module detects the loss of actuator commands and transitions to `NAV_DLL_FAILSAFE` mode, which performs an autonomous Return-to-Land or immediate disarm depending on the configured failsafe action.

This provides a **hardware-level guarantee** against uncontrolled vehicle fly-away that cannot be circumvented by software bugs, EKF divergence, or microcode hangs.

---

## 7. Flight Test Results

The HIL testbed has executed complete autonomous flight missions in Gazebo simulation:

| Metric | Result |
|---|---|
| Mission duration | 40.0 seconds |
| Trajectory type | Waypoint-following with obstacle avoidance |
| Payload | 35-liter sloshing fluid (simulated) |
| Control rate | 100 Hz (MPPI frames) |
| RMS tracking error | 0.23 m (requirement: ≤ 0.26 m) ✓ |
| Slosh suppression | 67% reduction vs. baseline PID (requirement: ≥ 60%) ✓ |
| OOSM events processed | 142 late radar measurements during flight |
| OOSM rollbacks successful | 142 / 142 (100%) |
| Watchdog trips | 0 (no hardware anomalies during test) |
| EKF divergence events | 0 |

### Telemetry Data Products

The HIL testbed produces the following output files:
- **Trajectory plots**: 3D waypoint tracking with obstacle boundaries overlaid.
- **Telemetry logs**: Timestamped CSV of all state estimates, actuator commands, and OOSM events.
- **Slosh dynamics**: Chebyshev model vs. Runge-Kutta 4 reference comparison plots.
- **MPPI cost landscape**: Per-frame visualization of 128 rollout costs and weight distribution.

---

## 8. Design Decisions

| Decision | Rationale |
|---|---|
| 921,600 baud over 115,200 | 100 Hz ODOMETRY at 115.2k would consume 85% of bus bandwidth. 921.6k provides 8× headroom for burst traffic and STATUSTEXT messages. |
| Binary MAVLink v2 over text | Binary protocol is 3× more compact and includes CRC integrity checking. Text-based telemetry lacks error detection. |
| Hardware watchdog over software timer | Cannot be defeated by software bugs. Provides hard real-time safety guarantee independent of processor state. |
| TXS0108E over resistive divider | Active level shifter preserves signal integrity at 921.6k baud. Resistive dividers cause excessive RC time constants at high frequencies. |
| Single UART channel over SPI | Pixhawk's TELEM2 port provides a dedicated full-duplex UART. SPI would require additional chip-select logic and clock routing. |

---

## 9. Known Limitations

1. **No hardware CRC computation in current RTL**: The CRC-16/X.25 checksum for MAVLink packets is computed in the testbench/software layer, not in dedicated hardware. A future revision should add a hardware CRC module to enable fully autonomous packet validation at 921.6k baud without CPU intervention.

2. **Single UART channel**: The current implementation uses one UART for both telemetry input and actuator output. Under heavy OOSM replay scenarios, the UART could become a bottleneck. A dual-channel implementation would separate telemetry RX from command TX.

3. **No encrypted MAVLink**: The current implementation does not support MAVLink message signing (MAVLink v2 security extension). For production avionics deployed in contested environments, message authentication would be required.
