# Stepping Stone 5: Hardware Bringup, Calibration & Low-Level Control Protocol

This document serves as the technical specification and laboratory handbook for **Stepping Stone 5: Hardware Bringup, Servo Calibration, and Low-Level Firmware Interfacing**. It covers the binary and ASCII serial protocols of the **LewanSoul / Hiwonder xArm 1s**, the **Arduino Uno R4 WiFi** electrical interfacing, the step-by-step motor deviation zeroing workflow, and the empirical joint limit envelopes.

---

## 1. Low-Level Communication Architecture

The xArm 1s manipulators are driven by digital bus servos (**LX-15D**) interconnected via a half-duplex daisy-chain bus. The arm base incorporates an onboard secondary microcontroller board that provides two communication modalities:

```text
    ┌───────────────────────────┐
    │  PC (ROS 2) or Laptop     │
    │  (/dev/ttyUSB0 @ 115200)   │
    └─────────────┬─────────────┘
                  │ (USB Serial)
                  ▼
    ┌───────────────────────────┐
    │ xArm 1s Onboard Controller │ ◄── [ Hardware Serial1 (Pins 0/1) ] ── [ Arduino Uno R4 WiFi ]
    │ (STC / STM32 Processor)   │
    └─────────────┬─────────────┘
                  │ (1-Wire Half-Duplex Bus @ 115200 Baud)
                  ▼
    [ Motor 1 ] ──► [ Motor 2 ] ──► [ Motor 3 ] ──► [ Motor 4 ] ──► [ Motor 5 ] ──► [ Motor 6 ]
     (Base Yaw)     (Shoulder)       (Elbow)       (Wrist Pitch)   (Wrist Roll)     (Gripper)
```

### 1.1 Binary Packet Protocol (Hiwonder / LewanSoul Bus Protocol)

Each binary command packet transmitted across the UART interface is framed as follows:

| Field | Size | Value / Formula | Description |
| :--- | :---: | :---: | :--- |
| **Header 1** | 1 byte | `0x55` | Packet start delimiter byte 1 |
| **Header 2** | 1 byte | `0x55` | Packet start delimiter byte 2 |
| **Servo ID** | 1 byte | `0x01` to `0x06` (`0xFE` for broadcast) | Target motor identifier |
| **Data Length** | 1 byte | $N + 2$ | Total bytes following this field up to Checksum |
| **Command ID** | 1 byte | e.g., `0x03` (`CMD_SERVO_MOVE`) | Operation code |
| **Parameters** | $N$ bytes | Command dependent | Arguments (e.g., target ticks, duration) |
| **Checksum** | 1 byte | $\sim(\text{ID} + \text{Length} + \text{Cmd} + \sum \text{Params}) \ \& \ \text{0xFF}$ | Bitwise NOT of arithmetic byte sum |

#### Key Command Identifiers:
* `0x03 (CMD_SERVO_MOVE)`: Move servo to target position ($0\text{--}1000\text{ ticks}$) in duration $T$ ($0\text{--}30000\text{ ms}$).
  * Params: `Pos_Low`, `Pos_High`, `Time_Low`, `Time_High` ($4\text{ bytes}$).
* `0x0E (CMD_SERVO_LOAD_OR_UNLOAD_WRITE)`: Enable or disable motor torque output ($1\text{ byte}: 0 = \text{unload}, 1 = \text{load}$).
* `0x15 (CMD_SERVO_POS_READ)`: Query current real-time potentiometer position ($0\text{--}1000\text{ ticks}$).
* `0x1B (CMD_SERVO_ANGLE_OFFSET_ADJUST)`: Adjust runtime deviation offset without writing to flash.
* `0x1D (CMD_SERVO_ANGLE_OFFSET_WRITE)`: Permanently save current deviation offset into internal EEPROM.

### 1.2 ASCII Serial Command Protocol (Secondary UART Header)

For rapid human debugging and microcontroller interfacing, the onboard controller parses high-level ASCII strings terminated by an exclamation mark (`!`):

$$\text{Format: } \texttt{\#<ID>P<Position>T<Time\_ms>!}$$

* **Single Servo Move**: `"#001P0600T0500!"` $\implies$ Move Motor 1 to $600\text{ ticks}$ over $500\text{ ms}$.
* **Synchronized Multi-Servo Move**: `"#001P0500T1000!#002P0700T1000!#003P0300T1000!"`
* **Stop Motion**: `"#001PDST!"` $\implies$ Immediately decelerates and holds current position.

Both `arduino/xarm_zero_calibration/xarm_zero_calibration.ino` and `arduino/xarm_test_bench/xarm_test_bench.ino` dual-transmit both formats to guarantee plug-and-play compatibility across all xArm 1s firmware revisions.

---

## 2. Electrical Wiring & Power Isolation Architecture

```text
    ┌───────────────────────────┐                ┌───────────────────────────┐
    │    External Bench PSU     │                │   Arduino Uno R4 WiFi     │
    │     (7.4V @ 5A DC)        │                │   (5V USB from Laptop)    │
    └──────┬─────────────┬──────┘                └──────┬─────────────┬──────┘
           │ (+)         │ (-)                          │             │
        +7.4V           GND                            GND          Serial1 (TX/RX)
           │             │                              │             │
           │             └──────────────────────────────┴──────┐      │
           │             │ (Common Ground Reference)           │      │
           ▼             ▼                                     ▼      ▼
    ┌────────────────────────────────────────────────────────────────────────┐
    │                     xArm 1s Base Controller Board                      │
    │  Power In: [ 7.4V ] [ GND ]        UART Port: [ 5V ] [ GND ] [ RX ] [ TX ]
    └────────────────────────────────────────────────────────────────────────┘
```

> [!CAUTION]
> **Brownout & Back-EMF Protection Rules:**
> 1. **Never Power Servos from Arduino 5V**: The 6 bus servos draw up to $3.5\text{ A}$ under peak stall acceleration. Attempting to power them from the Arduino $5\text{V}$ rail will permanently damage the onboard Renesas MCU.
> 2. **Strict Common Ground**: The Arduino `GND` **must** be connected directly to the xArm controller `GND`. Floating grounds cause floating UART logic levels and corrupt packets.
> 3. **Logic Level Compatibility**: The Arduino Uno R4 WiFi features $5\text{V}$ tolerant I/O on `Serial1` (`Pin 0 = RX`, `Pin 1 = TX`), which natively mates with the xArm controller's $5\text{V}$ TTL UART header without requiring a logic level shifter.

### 2.1 Test Bench Pinout Table (Arduino Uno R4 WiFi)

| Component | Pin on Uno R4 | Signal Type | Description |
| :--- | :---: | :---: | :--- |
| **xArm UART TX** | `Pin 0 (RX)` | Digital Input (Serial1) | Receives status/telemetry packets from xArm |
| **xArm UART RX** | `Pin 1 (TX)` | Digital Output (Serial1) | Transmits motion packets to xArm |
| **Joystick X-Axis** | `A0` | Analog Input ($0\text{--}5\text{V}$) | Proportional jog control forward/backward |
| **Joystick Y-Axis** | `A1` | Analog Input ($0\text{--}5\text{V}$) | Secondary trim / speed modifier |
| **Potentiometer** | `A2` | Analog Input ($0\text{--}5\text{V}$) | Active Joint Selector ($1\text{ to }6$) |
| **Pushbutton (Pose)** | `D2` | Digital Input (`INPUT_PULLUP`)| Cycles through 4 predefined safety poses |
| **Emergency Stop Button** | `D3` | Digital Input (`INPUT_PULLUP`)| Hardware Interrupt (INT1) — Immediately halts motion & cuts servo torque |
| **Green LED** | `D8` | Digital Output ($5\text{V}$) | System Ready / Idle indicator |
| **Yellow LED** | `D9` | Digital Output ($5\text{V}$) | Motion Active indicator |
| **Red LED** | `D10` | Digital Output ($5\text{V}$) | Joint Limit / E-Stop Active indicator (flashes at 4 Hz) |

---

## 3. Motor Deviation Zeroing & Calibration Workflow

Every hobby robotic arm experiences physical assembly tolerance errors: a servo horn pressed onto a spline can be off by $1\text{--}3$ teeth ($3^\circ\text{--}8^\circ$). If these deviations are not zeroed out, all theoretical DH parameters and analytical inverse kinematics calculations will suffer systematic Cartesian offsets.

```text
    Theoretical Center (500 ticks / 120°):
    
         Motor 2 (Shoulder):   Arm segment points 90° straight UP
         Motor 3 (Elbow):      Forearm forms exactly 90° angle with upper arm
         Motor 4 (Wrist):      Claw points straight along forearm axis
         Motor 5 (Roll):       Claw fingers are perfectly horizontal
         Motor 6 (Claw):       Fingers mid-span (approx 25 mm gap)
```

### 3.1 Step-by-Step Zeroing Procedure (30-Minute Lab Protocol)

1. **Physical Inspection**:
   * Inspect the arm mechanically before applying power. Ensure no cables are tangled around the base or shoulder linkages.
2. **Flash Zeroing Sketch**:
   * Connect Arduino Uno R4 to laptop via USB.
   * Open `arduino/xarm_zero_calibration/xarm_zero_calibration.ino` in Arduino IDE. Select board **Arduino UNO R4 WiFi** and the assigned COM/TTY port.
   * Upload sketch.
3. **Open Serial Monitor**:
   * Set baud rate to **`115200 baud`**.
   * The arm will immediately move to its default midpoint ($500\text{ ticks}$ on all motors) over $1.5\text{ seconds}$.
4. **Mechanical Alignment Verification**:
   * **Motor 1 (Base Yaw)**: Verify the arm points directly along the chassis midline.
   * **Motor 2 (Shoulder)**: Inspect if the upper arm segment is perpendicular to the base mounting plate.
   * **Motor 3 (Elbow)**: Check if the forearm forms a clean right angle ($90^\circ$) with the upper arm.
   * **Motor 4 (Wrist Pitch)**: Confirm the tool assembly is collinearly aligned with the forearm.
   * **Motor 5 (Wrist Roll)**: Verify the claw jaws are parallel to the table.
5. **Trimming Deviations Interactively**:
   * Type `1` through `6` into the Serial Monitor to select a motor.
   * Type `+` or `-` to adjust the deviation by $\pm 1\text{ tick}$ ($0.24^\circ$).
   * Type `>` or `<` to adjust by $\pm 10\text{ ticks}$ ($2.4^\circ$).
   * Observe physical joint movement until the chassis alignment marks match perfectly.
6. **Export Calibration Offsets**:
   * Type `p` in the Serial Monitor.
   * The sketch outputs a formatted C++ / Python array representing the measured deviations.
   * Save these values to `config/servo_calibration.yaml` for use by the ROS 2 driver!

---

## 4. Tick-to-Angle Kinematic Mapping

The LewanSoul LX-15D bus servos have a total travel of **$240.0^\circ$** resolved across **$1000\text{ ticks}$** ($0\text{ to }1000$ integer counts).

### 4.1 Forward Transformation ($\text{Ticks} \to \text{Degrees}$)

$$\theta_{\text{deg}} = \text{Ticks} \times \left(\frac{240.0^\circ}{1000}\right) = \text{Ticks} \times 0.24$$

$$\text{Center Reference: } \quad 500\text{ ticks} \iff 120.0^\circ$$

$$\text{Angular Deviation from Center: } \quad \Delta \theta_{\text{deg}} = (\text{Ticks} - 500) \times 0.24$$

### 4.2 Inverse Transformation ($\text{Degrees} \to \text{Ticks}$)

$$\text{Ticks} = \text{round}\left(\theta_{\text{deg}} \times \frac{1000}{240.0^\circ}\right) = \text{round}(\theta_{\text{deg}} \times 4.16667)$$

$$\text{Offset Formula: } \quad \text{Ticks}_{\text{commanded}} = 500 + \text{round}(\Delta \theta_{\text{joint}} \times 4.16667) + \text{Deviation}_{\text{motor}}$$

---

## 5. Empirical Joint Travel Envelopes & Safe Limits

To prevent internal gear-stripping or driving linkages into mechanical hard stops, software bounds are strictly enforced in firmware and ROS 2 drivers:

| Motor ID | Joint Name | Min Safe Ticks | Max Safe Ticks | Safe Angle Span | Role / Constraint |
| :---: | :--- | :---: | :---: | :---: | :--- |
| **Motor 1** | Base Yaw ($\theta_1$) | `150` | `850` | $36.0^\circ\text{ to }204.0^\circ$ ($\Delta = 168^\circ$) | Confined within $205^\circ$ swing to avoid $240^\circ$ deadzone |
| **Motor 2** | Shoulder Pitch ($\theta_2$) | `200` | `850` | $48.0^\circ\text{ to }204.0^\circ$ | Prevents backward hyperextension against baseplate |
| **Motor 3** | Elbow Pitch ($\theta_3$) | `150` | `850` | $36.0^\circ\text{ to }204.0^\circ$ | Operates in Elbow-Up configuration |
| **Motor 4** | Wrist Pitch ($\theta_4$) | `150` | `850` | $36.0^\circ\text{ to }204.0^\circ$ | Maintains approach pitch $\phi \in [-75^\circ, 0^\circ]$ |
| **Motor 5** | Wrist Roll ($\theta_5$) | `200` | `800` | $48.0^\circ\text{ to }192.0^\circ$ | Locked to $0^\circ$ in Regime B to prevent claw tilt |
| **Motor 6** | Claw Gripper | `150` (Closed) | `750` (Open) | $36.0^\circ\text{ to }180.0^\circ$ | $150\text{ ticks} = 0\text{ mm}$, $750\text{ ticks} = 55\text{ mm}$ opening |
