# Stepping Stone 5: Hardware Bringup, Calibration & Low-Level Control Protocol

This document serves as the technical specification and laboratory handbook for **Stepping Stone 5: Hardware Bringup, Servo Calibration, and Low-Level Firmware Interfacing**. It covers the binary and ASCII serial protocols of the **LewanSoul / Hiwonder xArm 1s**, direct USB HID / Serial interfacing, the official Hiwonder xArm PC App zeroing workflow, and empirical joint limit envelopes.

---

## 1. Low-Level Communication Architecture

The xArm 1s manipulators are driven by digital bus servos (**LX-15D**) interconnected via a half-duplex daisy-chain bus. The arm base incorporates an onboard controller board that communicates directly with the PC/Jetson over USB:

```text
    ┌───────────────────────────┐
    │  PC (ROS 2 / Python Host) │
    │  (USB HID / /dev/ttyUSB*) │
    └─────────────┬─────────────┘
                  │ (Direct USB @ 115200 / 9600 Baud)
                  ▼
    ┌───────────────────────────┐
    │ xArm 1s Onboard Controller │ ◄── [ Official xArm PC App / Python Driver ]
    │ (STC / STM32 Processor)   │
    └─────────────┬─────────────┘
                  │ (1-Wire Half-Duplex Bus @ 115200 Baud)
                  ▼
    [ Motor 6 ] ──► [ Motor 5 ] ──► [ Motor 4 ] ──► [ Motor 3 ] ──► [ Motor 2 ] ──► [ Motor 1 ]
     (Base Yaw)     (Shoulder)       (Elbow)       (Wrist Pitch)   (Wrist Roll)     (Gripper)
```

> [!NOTE]
> **Kinematic Joint Indices vs. Physical Hardware Motor IDs**:
> In standard robotics literature (Denavit-Hartenberg notation), kinematic joints are indexed from base to tip ($\theta_1 = \text{Base Yaw}, \theta_2 = \text{Shoulder Pitch}, \theta_3 = \text{Elbow Pitch}, \theta_4 = \text{Wrist Pitch}, \theta_5 = \text{Wrist Roll}, \theta_6 = \text{Claw Clamp}$).
> However, on the physical **Hiwonder / LewanSoul xArm 1s** bus servo daisy chain, hardware servo IDs are factory-addressed from end-effector to base:
> - **Hardware Motor 1**: Claw Gripper
> - **Hardware Motor 2**: Wrist Roll
> - **Hardware Motor 3**: Wrist Pitch
> - **Hardware Motor 4**: Elbow Pitch
> - **Hardware Motor 5**: Shoulder Pitch
> - **Hardware Motor 6**: Base Yaw
> Both the unified configuration (`config/dual_xarm_hardware.yaml`), ROS 2 driver (`driver_node.py`), and test suite (`motion_test_suite.py`) strictly adhere to these official hardware IDs.

### 1.1 Direct Controller UART & USB HID Protocol

The secondary development UART header on the xArm base board (`RX TX GND`) interfaces with the onboard host microcontroller. Commands sent across this interface use the **Hiwonder / Lobot Bus Servo Controller Protocol**:

| Field | Size | Value / Formula | Description |
| :--- | :---: | :---: | :--- |
| **Header 1** | 1 byte | `0x55` | Packet start delimiter byte 1 |
| **Header 2** | 1 byte | `0x55` | Packet start delimiter byte 2 |
| **Data Length** | 1 byte | $\text{Params} + 2$ | Total bytes following this field ($N \text{ parameters} + 1 \text{ byte Command} + 1 \text{ byte Length}$) |
| **Command ID** | 1 byte | e.g., `0x03` (`CMD_SERVO_MOVE`) | Operation code |
| **Parameters** | $N$ bytes | Command dependent | Arguments (e.g., servo count, duration, ID, position) |

> [!IMPORTANT]
> **NO CHECKSUM ON SECONDARY CONTROLLER UART HEADER:**
> The secondary development controller board UART header does **not** take a checksum. Appending a checksum byte causes the controller's FIFO parser to encounter an extra byte when searching for the next `0x55 0x55` header, triggering a **packet frame error** and sounding the **buzzer ("beep beep")** while rejecting the command.
> (The checksum is only used internally on the 1-wire half-duplex bus between the controller and the individual LX-15D bus servos).

#### Key Command Formats:
* **Single Servo Move (`CMD_SERVO_MOVE = 0x03`)**: Length = 8 (10 bytes total)
  $$\texttt{0x55 0x55 [0x08] [0x03] [Count=0x01] [Time\_L] [Time\_H] [ID] [Pos\_L] [Pos\_H]}$$
* **Multi-Servo Move (`CMD_SERVO_MOVE = 0x03`)**: Length = $\text{Count} \times 3 + 5$ (25 bytes for 6 servos)
  $$\texttt{0x55 0x55 [0x17] [0x03] [0x06] [Time\_L] [Time\_H] [ID\_1] [P1\_L] [P1\_H] ... [ID\_6] [P6\_L] [P6\_H]}$$
* **Unload Motor Torque (`CMD_MULT_SERVO_UNLOAD = 0x14`)**: Length = 9 (11 bytes total)
  $$\texttt{0x55 0x55 [0x09] [0x14] [0x06] [0x01] [0x02] [0x03] [0x04] [0x05] [0x06]}$$
* **Read Battery Voltage (`CMD_GET_BATTERY_VOLT = 0x0F`)**: Length = 3 (5 bytes total)
  $$\texttt{0x55 0x55 [0x03] [0x0F] [0x00]}$$

### 1.2 Baud Rate & Buzzer Diagnostics

* **Secondary Development UART Baud Rate**:
  The default factory baud rate of the secondary development UART header (`RX TX GND`) is **9600 baud** on standard Hiwonder bus servo controller boards (some units may be flashed with 115200 baud). Both Arduino sketches provide live toggling between 9600 and 115200 (type `b` in the zero-calibration sketch or `BAUD` in the test bench).
* **Buzzer Error Indication ("Beep Beep")**:
  The onboard buzzer sounds an alarm and the blue signal LED (LED2) stays lit under two conditions:
  1. **Malformed Packet / Framing Error**: Extra bytes (like an unexpected checksum), wrong data length, or baud rate mismatch.
  2. **Low Voltage Alarm**: If battery or power supply voltage drops below $6.2\text{V}$ under load.

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

![xArm 1s Secondary Development Controller Board Pinout and Interfaces](pcb%20module.webp)

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
    
         Motor 6 (Base Yaw):      Points collinearly along chassis midline (East / +X)
         Motor 5 (Shoulder):      Upper arm segment points 90° straight UP (perpendicular to table)
         Motor 4 (Elbow):         Forearm forms exactly 90° right angle with upper arm
         Motor 3 (Wrist Pitch):   Claw assembly points straight along forearm axis (collinear)
         Motor 2 (Wrist Roll):    Claw fingers are perfectly horizontal (parallel to table)
         Motor 1 (Claw Gripper):  Fingers at nominal mid-span (~40 mm opening)
```

### 3.1 Step-by-Step Zeroing Procedure (Official xArm PC App Protocol)

1. **Physical Inspection**:
   * Inspect the arm mechanically before applying power. Ensure no cables are tangled around the base or shoulder linkages.
2. **Launch Official xArm PC App**:
   * Connect xArm 1s to PC via USB cable.
   * Open the official Hiwonder xArm PC App. Connect to the arm's COM port or USB HID interface.
3. **Reset to Midpoint**:
   * Click **Reset** in the app. The arm moves to default midpoint ($500\text{ ticks}$ on all motors).
4. **Mechanical Alignment Verification**:
   * **Motor 6 (Base Yaw / $\theta_1$)**: Verify the arm points directly along the chassis midline.
   * **Motor 5 (Shoulder Pitch / $\theta_2$)**: Inspect if the upper arm segment is perpendicular to the base mounting plate.
   * **Motor 4 (Elbow Pitch / $\theta_3$)**: Check if the forearm forms a clean right angle ($90^\circ$) with the upper arm.
   * **Motor 3 (Wrist Pitch / $\theta_4$)**: Confirm the tool assembly is collinearly aligned with the forearm.
   * **Motor 2 (Wrist Roll / $\theta_5$)**: Verify the claw jaws are parallel to the table.
   * **Motor 1 (Claw Gripper / $\theta_6$)**: Confirm fingers rest at nominal mid-span ($500\text{ ticks}$).
5. **Adjusting & Saving Deviations in Hardware EEPROM**:
   * Use the deviation sliders in the xArm PC App to adjust zero-point trim until linkages align perfectly.
   * Click **"Download Deviation" / "Save Deviation"** to write the offsets permanently into each bus servo's onboard EEPROM registers.
   * Note the min and max ticks for travel boundaries and the nominal center tick.
6. **Save to Unified Hardware Configuration**:
   * Record the verified min/max travel limits and nominal center ticks into `config/dual_xarm_hardware.yaml`.
   * All subsequent ROS 2 nodes, kinematics solvers, and motion test suites read directly from this unified configuration!

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

| Hardware ID | Kinematic Joint | Joint Name | Min Safe Ticks | Max Safe Ticks | Safe Angle Span | Role / Constraint |
| :---: | :---: | :--- | :---: | :---: | :---: | :--- |
| **Motor 1** | $\theta_6$ | Claw Gripper | `150` (Closed) | `800` (Open) | $36.0^\circ\text{ to }192.0^\circ$ | $150\text{ ticks} = 0\text{ mm}$, $750\text{ ticks} = 55\text{ mm}$ opening |
| **Motor 2** | $\theta_5$ | Wrist Roll | `200` | `800` | $48.0^\circ\text{ to }192.0^\circ$ | Locked to $0^\circ$ in Regime B to prevent claw tilt |
| **Motor 3** | $\theta_4$ | Wrist Pitch | `150` | `850` | $36.0^\circ\text{ to }204.0^\circ$ | Maintains approach pitch $\phi \in [-75^\circ, 0^\circ]$ |
| **Motor 4** | $\theta_3$ | Elbow Pitch | `150` | `850` | $36.0^\circ\text{ to }204.0^\circ$ | Operates in Elbow-Up configuration |
| **Motor 5** | $\theta_2$ | Shoulder Pitch | `200` | `850` | $48.0^\circ\text{ to }204.0^\circ$ | Prevents backward hyperextension against baseplate |
| **Motor 6** | $\theta_1$ | Base Yaw | `150` | `850` | $36.0^\circ\text{ to }204.0^\circ$ ($\Delta = 168^\circ$) | Confined within $205^\circ$ swing to avoid $240^\circ$ deadzone |
