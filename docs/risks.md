# Risk Assessment & Mitigation Matrix — Dual xArm 1s Color-Sorting Robot

This document details the critical physical, optical, electronic, and algorithmic failure modes identified during the ideation and architecture phases, along with concrete mitigation strategies.

---

## 1. Risk Summary Matrix

| ID | Category | Risk Description | Severity | Probability | Primary Mitigation |
|---|---|---|:---:|:---:|---|
| **R-01** | Mechanical | Cube sliding / displacement during horizontal grasp | **High** | **High** | High-friction silicone arena mat + textured claw pads |
| **R-02** | Kinematics | Arm sag & joint flex at max extension ($R \approx 26\text{ cm}$) | **High** | **High** | Radius-dependent $Z$-offset compensation in Pitch LUT |
| **R-03** | Vision | Optical parallax error at outer arena perimeter | **Medium** | **High** | Height-ratio radial parallax correction in `pipeline.py` |
| **R-04** | Vision | HSV color threshold drift under fluctuating lab lighting | **High** | **Medium** | Fixed camera exposure + dedicated 5V diffuse LED ring |
| **R-05** | Hardware | Power brownouts / servo reset during peak dual-arm torque | **Critical**| **Medium** | Independent 7.4V/5A power supply per arm (shared ground) |
| **R-06** | Comms | WiFi latency jitter / packet drops causing arm stutter | **High** | **Medium** | Direct USB-to-UART serial connections to compute unit |
| **R-07** | Mechanical | Cubes bouncing off collection box dividers on drop | **Low** | **Medium** | Chamfered / funnel-top bin dividers |
| **R-08** | Mechanical | Table gouging from claw rotation at low angles | **High** | **Low** | Fixed wrist orientation (locked $\text{Roll} = 0^\circ$) |
| **R-09** | Multi-Arm | Deadlock or race condition on shared arena access | **High** | **Low** | Strict FIFO atomic queue in `arena_mutex` scheduler |
| **R-10** | Vision | ArUco corner marker occlusion by operator or arm swing | **Medium** | **Low** | Cache homography matrix; only update on valid 4-corner lock |

---

## 2. In-Depth Risk Analysis & Engineering Mitigations

### 2.1 Mechanical & Kinematics Risks

#### R-01: The "Pushed Cube" Contact Problem
* **Failure Mechanism**: At distances $R > 20\text{ cm}$, the arm approaches with a low tilt angle ($10^\circ\text{--}25^\circ$). The claw fingers close along the horizontal plane. On smooth wood, acrylic, or laminated surfaces, the finger tips will push or kick the 25 mm wooden cube across the table instead of clamping it.
* **Impact**: Failed grasps, cubes pushed into dead zones or colliding with neighboring cubes.
* **Engineering Solution**:
  1. Cover the entire 80×60 cm baseboard with a **matte textured silicone or neoprene mat**. The high static friction prevents cubes from sliding.
  2. Apply thin anti-slip silicone tape to the inner pads of the 2-finger claw.
  3. Implement a rapid-close servo profile: approach slowly, then close the claw swiftly so inertial force does not shift the object.

#### R-02: Structural Flex & Servo Droop at Maximum Reach
* **Failure Mechanism**: The xArm 1s uses serial bus servos with gear trains. When the arm is fully extended horizontally to $26\text{ cm}$, link bending and gear backlash cause the end-effector to sag by $4\text{--}8\text{ mm}$ below the commanded position.
* **Impact**: Commanding a nominal table-level $Z$ at $R = 26\text{ cm}$ will slam the claw into the table, causing servo stall cutoffs, gear skipping, or stripped teeth.
* **Engineering Solution**:
  1. Incorporate an empirical $Z$-compensation curve into the Pitch Lookup Table:
     * Near ($R = 12\text{ cm}$): $Z_{\text{cmd}} = Z_0$
     * Mid ($R = 18\text{ cm}$): $Z_{\text{cmd}} = Z_0 + 3\text{ mm}$
     * Far ($R = 26\text{ cm}$): $Z_{\text{cmd}} = Z_0 + 6\text{ mm}$
  2. Calibrate table height empirically with the physical arm before running vision-driven tasks.

#### R-07: Collection Box Drop Accuracy & Bounce
* **Failure Mechanism**: Releasing a cube into a $45 \times 45\text{ mm}$ cell from an angled wrist orientation can cause the cube to strike the top edge of a partition wall and bounce into the wrong cell.
* **Impact**: Sorting errors on demo day despite 100% correct vision detection.
* **Engineering Solution**:
  1. Add $45^\circ$ chamfers or 3D-print a drop funnel onto the top rim of the collection box partition walls.
  2. Ensure the drop sequence includes an explicit descent step ($Z_{\text{drop}}$ lowers inside the rim before opening the claw).

---

### 2.2 Computer Vision & Optical Risks

#### R-03: Perspective Parallax Error at Perimeter
* **Failure Mechanism**: The overhead camera is mounted at the center $(X=400, Y=280)$. For cubes at the perimeter, the optical line of sight strikes the table at an oblique angle ($\sim 25^\circ\text{--}30^\circ$). Because cubes are 25 mm tall, the camera captures both the top face and the side face. The contour centroid will be offset radially outward from the true ground contact point by $4\text{--}8\text{ mm}$.
* **Impact**: The arm reaches to the visual centroid and misses the cube or clips its edge.
* **Engineering Solution**:
  1. Add a radial parallax correction formula in `pipeline.py`:
     $$\Delta r_{\text{corr}} = -\frac{h_{\text{cube}}}{2 \cdot H_{\text{cam}}} \cdot r_{\text{camera}}$$
  2. Calibrate ArUco marker height to match the center of mass of the cubes ($Z = 12.5\text{ mm}$) by mounting markers on thin wooden risers.

#### R-04: Ambient Light & HSV Threshold Instability
* **Failure Mechanism**: Sunlight shifting through college windows or alternating AC fluorescent lighting creates hue and saturation shifts. Yellow and Red are particularly prone to washing out or misclassification.
* **Impact**: Undetected cubes, wrong color classifications, or noisy false-positive bounding boxes.
* **Engineering Solution**:
  1. Mount a dedicated 5V USB diffuse LED ring light directly around the camera lens.
  2. Disable all camera auto-exposure and auto-white-balance in OpenCV:
     ```python
     cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
     cap.set(cv2.CAP_PROP_EXPOSURE, -5)
     ```
  3. Store color ranges in `config/colors.yaml` with explicit tolerance bands tuned under the dedicated LED light.

#### R-10: ArUco Marker Occlusion
* **Failure Mechanism**: While an arm is reaching or rotating, its shoulder or elbow link might pass directly over one of the 4 corner ArUco markers.
* **Impact**: If homography is recalculated every single frame naively, losing 1 marker will crash the perspective warp.
* **Engineering Solution**:
  1. Implement a **Homography Cache**: Compute and lock the $3 \times 3$ perspective transform matrix only when all 4 markers are detected with high confidence. If fewer than 4 are visible, reuse the last valid matrix.

---

### 2.3 Electronics, Power & Communications Risks

#### R-05: Voltage Sag & Brownouts under Dual-Arm Load
* **Failure Mechanism**: When both arms move simultaneously or close their grippers under load, the 10+ bus servos can draw peak transient currents exceeding $6\text{--}8\text{ A}$. Running both arms off a single weak power supply causes voltage dips below $6.0\text{ V}$, resetting the servo microcontrollers mid-motion.
* **Impact**: Arms suddenly lose torque, go limp, or drop cubes unexpectedly.
* **Engineering Solution**:
  1. Provide **two separate dedicated 7.4V / 5A power supplies** (one for each arm).
  2. Connect the ground lines (GND) of both power supplies and the compute unit together to maintain a common signal reference.

#### R-06: WiFi Network Congestion & Jitter
* **Failure Mechanism**: College campus WiFi networks have dynamic channel hopping, firewall restrictions, and severe 2.4 GHz interference from hundreds of personal devices.
* **Impact**: TCP socket timeouts, command lag of $200\text{--}600\text{ ms}$, or broken ROS 2 topic handshakes.
* **Engineering Solution**:
  1. **Primary Protocol**: Use direct USB-to-UART serial communication (`/dev/ttyUSB0` and `/dev/ttyUSB1`) between the compute unit (Jetson/RPi) and each arm.
  2. **Backup Protocol**: If WiFi is required, use a dedicated standalone offline 5 GHz router placed right next to the workspace with no internet uplink.

---

### 2.4 Multi-Arm Coordination & Scheduling Risks

#### R-09: Multi-Arm Deadlock / Starvation
* **Failure Mechanism**: If an arm experiences a grip failure or enters an error state while holding `arena_mutex`, the other arm could be blocked indefinitely waiting for the mutex to release.
* **Impact**: Total system freeze during a live demo.
* **Engineering Solution**:
  1. **Mutex Timeout & Auto-Release**: Implement a maximum lease time on `arena_mutex` (e.g., 8 seconds). If an arm does not complete its pick within the timeout, it releases the mutex, enters safe home pose, and flags an alert.
  2. **Atomic State Tracking**: Ensure mutex release is executed inside a `finally` block in Python, guaranteeing release even on exceptions.

---

## 3. Pre-Flight Verification Checklist

Before running full-speed autonomous sorting, complete this verification routine:

- [ ] **Baseboard Level**: Verify baseboard is flat with no bowing across the 80 cm span.
- [ ] **Friction Check**: Test dragging a cube across the mat—verify it does not slip when touched lightly.
- [ ] **Serial Latency**: Ping both arms over USB serial; confirm round-trip command time $< 15\text{ ms}$.
- [ ] **Light Lock**: Confirm camera auto-exposure is OFF and histogram is stable.
- [ ] **Sag Profile**: Jog Arm 1 and Arm 2 to $R=26\text{ cm}$ manually; verify claw clears table by $3\text{--}5\text{ mm}$.
- [ ] **Mutex Timeout**: Intentionally simulate a delayed pick; confirm the secondary arm does not deadlock.