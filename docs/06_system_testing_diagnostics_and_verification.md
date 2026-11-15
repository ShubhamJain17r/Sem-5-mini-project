# Stepping Stone 6: System Testing, Fault Diagnostics & Verification Matrix

This document provides the definitive engineering specification for **Stepping Stone 6: Verification, Diagnostics, and Algorithmic Failure Mitigation**. It addresses the physical realities and failure modes identified during the ideation phase, specifically:
1. **Nearest-First Corridor Collision Avoidance** (preventing forearm plowing during horizontal picks).
2. **Software Grip Confirmation via Motor 6 Position Readback** (detecting dropped or missed cubes without extra sensors).
3. **Empirical Z-Sag Compensation** (offsetting linkage flex and gear backlash at extreme reach).
4. **Camera Lens Undistortion Pipeline** (eliminating metric warping errors at arena boundaries).
5. **System-wide Fault Diagnostics and Safety Watchdogs**.

---

## 1. The Nearest-First Corridor Collision Avoidance Algorithm

### 1.1 The Radial Plowing Failure Mode

In Stepping Stone 2 ([`02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md)), we established that pure vertical top-down grasps are kinematically impossible beyond $r = 158.7\text{ mm}$, necessitating an inclined or near-horizontal approach vector ($\phi \in [-25^\circ, -5^\circ]$).

However, in an arena populated by 16–20 scattered cubes, if the arm attempts to pick **Cube B** ($r = 280\text{ mm}$) while **Cube A** ($r = 160\text{ mm}$) lies along the same azimuthal line of sight ($\theta_B \approx \theta_A$):

```text
  Top-Down View of Radial Plowing:
  
  xArm Base (0,0) ───────► [ Cube A (r = 160mm) ] ───────► [ Cube B (r = 280mm) ]
                                  ▲
                                  │ Forearm & Claw Strike Cube A During Descent!
```

As the arm sweeps forward at low altitude to grasp Cube B, the elbow bracket, forearm, or outer jaw will plow through Cube A, scattering it uncontrollably.

### 1.2 Mathematical Formulation of Corridor Clearance

Let an arm be positioned at $(X_{\text{base}}, Y_{\text{base}})$. For any target cube $i$ with polar coordinates $(r_i, \theta_i)$, the radial approach corridor is defined as the rectangular swept volume:

$$\mathcal{C}_i = \left\{ (r, \theta) \;\middle|\; 0 \le r < r_i - \Delta r_{\text{margin}}, \quad |\theta - \theta_i| < \arcsin\left(\frac{W_{\text{arm}} / 2}{r}\right) \right\}$$

Where:
* $W_{\text{arm}} \approx 65\text{ mm}$ (maximum physical width of the forearm and claw assembly).
* $\Delta r_{\text{margin}} = 30\text{ mm}$ (pre-grasp standoff distance).

### 1.3 The Nearest-First Queue Policy

To guarantee that the corridor $\mathcal{C}_i$ is strictly empty of physical obstacles, the `scheduler_node` enforces a **Nearest-First Sorting Policy**:

$$\text{Queue}_{\text{arm}} = \text{sort}\Big(\mathcal{S}_{\text{assigned\_cubes}}, \;\; \text{key} = r_i, \;\; \text{ascending} = \text{True}\Big)$$

```python
# scheduler_node snippet: Deterministic Nearest-First sorting
def prioritize_queue(detected_objects, arm_base_xy):
    """
    Sorts target objects by ascending radial distance r from the arm's base.
    Guarantees no foreground cube obstructs the radial approach path.
    """
    for obj in detected_objects:
        dx = obj.x_mm - arm_base_xy[0]
        dy = obj.y_mm - arm_base_xy[1]
        obj.r_mm = (dx**2 + dy**2)**0.5
        obj.theta_rad = math.atan2(dy, dx)
        
    # Sort strictly ascending by distance: closest cubes picked first
    return sorted(detected_objects, key=lambda obj: obj.r_mm)
```

**Result**: Inner cubes are always picked and sorted first. When the arm later reaches for outer cubes ($r > 250\text{ mm}$), the inner radial corridor is guaranteed to be completely clear.

---

## 2. Software Grip Confirmation via Motor 6 Readback

### 2.1 The Open-Loop Problem

Low-cost hobby robotic arms do not feature tactile load cells, optical break-beams, or pressure sensors on their fingertips. If an open-loop arm closes its claw and misses the cube, it will blindly execute a full $205^\circ$ swing to the collection box, open an empty claw, and report task success. The cube remains on the table, causing state desynchronization in the master scheduler.

### 2.2 Position Readback Grasp Discrimination

The **LewanSoul / Hiwonder LX-15D** bus servo inside the claw (Motor 6) includes an internal magnetic potentiometer. By issuing command `0x15 (CMD_SERVO_POS_READ)`, the controller reads the true shaft position after the clamp command has executed:

```text
    Motor 6 Tick Envelopes (Claw Clamp State):
    
    [0 ticks] ──── [150 ticks] ──────── [420 - 480 ticks] ──────── [750 ticks] ──── [1000 ticks]
                     ▲                        ▲                          ▲
                     │                        │                          │
                 FULLY CLOSED            GRIPPED 25mm CUBE            FULLY OPEN
                 (Missed Cube)          (Grasp Confirmed)           (Pre-Grasp)
```

| Motor 6 Position | Physical Meaning | Software Action |
| :---: | :--- | :--- |
| **$P \le 220\text{ ticks}$** | Jaws closed completely together ($w \approx 0\text{ mm}$). **Object was not captured**. | Set `has_cube = False`. Abort drop swing. Retract arm, increment retry counter, and re-scan arena. |
| **$380 \le P \le 520\text{ ticks}$** | Jaws stalled against flat faces of $25\text{ mm}$ cube ($w \approx 25\text{ mm}$). **Grasp Successful**. | Set `has_cube = True`. Proceed with transit and $205^\circ$ drop swing. |
| **$P > 600\text{ ticks}$** | Jaws jammed or obstructed before closing. | Set `error_code = "CLAW_JAMMED"`. Move arm to Home and alert operator. |

### 2.3 Implementation in `arm_node.py`

```python
def verify_grasp(self, serial_port, expected_min=380, expected_max=520, timeout_sec=1.0):
    """
    Queries Motor 6 position register to verify physical cube capture.
    Returns: bool (True if cube successfully grasped)
    """
    # Send CMD_SERVO_POS_READ for Motor 6
    self.send_packet(servo_id=6, cmd=0x15)
    
    pos_ticks = self.read_position_feedback(servo_id=6, timeout=timeout_sec)
    
    if expected_min <= pos_ticks <= expected_max:
        self.get_logger().info(f"Grasp Confirmed: Motor 6 clamped at {pos_ticks} ticks.")
        return True
    elif pos_ticks < 220:
        self.get_logger().warn(f"Grasp Failed: Claw fully closed ({pos_ticks} ticks). Cube missed!")
        return False
    else:
        self.get_logger().error(f"Grasp Abnormality: Claw obstructed at {pos_ticks} ticks.")
        return False
```

---

## 3. Empirical Z-Sag & Mechanical Backlash Compensation

### 3.1 Physics of Cantilever Deflection

The xArm 1s utilizes stamped sheet-metal aluminum brackets and nylon/metal gear-train servos. When the arm reaches horizontally to $R = 300\text{--}320\text{ mm}$, the static gravity moment on Motor 2 and Motor 3 reaches maximum:

$$\tau_{\text{shoulder}} = m_{\text{arm}} \cdot g \cdot r_{\text{CoM}} + m_{\text{payload}} \cdot g \cdot R$$

Under this load, gear backlash ($\approx 1^\circ\text{--}2^\circ$ per joint) and torsional bracket deflection compound, causing the physical claw to droop **$8\text{--}14\text{ mm}$ below the theoretically computed Cartesian height $Z$**.

Without compensation, a commanded grasp at $Z = 12.5\text{ mm}$ will result in the lower claw finger scraping the flat baseboard surface at $Z \approx 0.0\text{ mm}$, jamming the servos.

### 3.2 Piecewise Linear Compensation Model

We introduce a radial distance compensation offset $Z_{\text{offset}}(r)$:

$$Z_{\text{commanded}} = Z_{\text{target}} + \Delta Z_{\text{sag}}(r)$$

$$\Delta Z_{\text{sag}}(r) = \begin{cases} 
0, & r \le R_{\text{threshold}} \\
k_{\text{sag}} \cdot (r - R_{\text{threshold}}), & r > R_{\text{threshold}}
\end{cases}$$

Where:
* $R_{\text{threshold}} = 180\text{ mm}$ (deflection is negligible within near reach).
* $k_{\text{sag}} \approx 0.07\text{ mm/mm}$ (empirically tuned for xArm 1s link stiffness).

### 3.3 Empirical Sag Lookup Table

| Radial Distance $R$ | Distance from Edge $d_{\text{edge}}$ | Measured Uncompensated Droop | Commanded Height Offset $\Delta Z_{\text{sag}}$ | Effective Grasp Height |
| :---: | :---: | :---: | :---: | :---: |
| **$140\text{ mm}$** | $7\text{ cm}$ | $-1.5\text{ mm}$ | $+0.0\text{ mm}$ | $12.5\text{ mm}$ |
| **$180\text{ mm}$** | $11\text{ cm}$ | $-3.0\text{ mm}$ | $+0.0\text{ mm}$ | $12.5\text{ mm}$ |
| **$220\text{ mm}$** | $15\text{ cm}$ | $-6.0\text{ mm}$ | $+2.8\text{ mm}$ | $15.3\text{ mm}$ |
| **$260\text{ mm}$** | $19\text{ cm}$ | $-9.5\text{ mm}$ | $+5.6\text{ mm}$ | $18.1\text{ mm}$ |
| **$300\text{ mm}$** | $23\text{ cm}$ | $-12.0\text{ mm}$ | $+8.4\text{ mm}$ | $20.9\text{ mm}$ |
| **$320\text{ mm}$** | $25\text{ cm}$ | $-14.0\text{ mm}$ | $+9.8\text{ mm}$ | $22.3\text{ mm}$ |

---

## 4. Lens Undistortion Pipeline

Standard commercial webcams (such as the Logitech C270 / C920) feature wide-angle lenses exhibiting significant barrel distortion. Planar homography matrices assume a strict collinear projective transformation. Applying homography directly to barrel-distorted raw pixels introduces non-linear curvature errors:

```text
    Barrel Distortion Effect on Arena Geometry:
    
    True Rectilinear Plane:                 Uncorrected Webcam View:
    ┌─────────────────────────┐             ┌───────-~~~~~~~~~-───────┐
    │                         │             )                         (
    │    Elevated Arena       │    ───►    (      Curved Boundaries    )
    │                         │             )                         (
    └─────────────────────────┘             └───────-~~~~~~~~~-───────┘
```

At the outer boundary corners ($X = 330.0\text{ mm}, Y = 40.0\text{ mm}$), uncorrected radial distortion can displace the perceived cube centroid by up to **$8.5\text{ mm}$**, exceeding the allowable claw capture tolerance.

### 4.1 Calibration & Undistortion Pipeline

Prior to running ArUco detection or planar homography, the camera intrinsic matrix $K$ and lens distortion coefficients $D$ are computed using a standard $9\times 6$ checkerboard:

$$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}, \quad D = \begin{bmatrix} k_1 & k_2 & p_1 & p_2 & k_3 \end{bmatrix}$$

```python
# vision_node processing pipeline
def process_frame(self, raw_frame):
    # Step 1: Remove lens barrel and tangential distortion
    undistorted_frame = cv2.undistort(raw_frame, self.camera_matrix, self.dist_coeffs)
    
    # Step 2: Detect 4 ArUco markers on the rectified image plane
    corners, ids, _ = cv2.aruco.detectMarkers(undistorted_frame, self.aruco_dict)
    
    # Step 3: Compute or apply cached planar homography
    warped_arena = cv2.warpPerspective(undistorted_frame, self.H, (self.arena_w_px, self.arena_h_px))
    
    # Step 4: Run HSV thresholding and contour moments
    return self.extract_cubes(warped_arena)
```

### 4.2 Homography Freezing on Startup

To prevent table and boom vibration (caused by rapid servo accelerations) from corrupting the coordinate mapping during live sorting:
* The 4 ArUco markers are detected and averaged over **30 consecutive frames** during startup calibration.
* The resulting homography matrix $H$ is **locked in memory**.
* Continuous ArUco detection is disabled during active arm execution, saving CPU cycles and guaranteeing zero coordinate jitter.

---

## 5. System Safety Watchdogs & Fault Annunciation

```text
    ┌──────────────────────────────────────────────────────────┐
    │                 SCHEDULER WATCHDOG SUITE                 │
    ├──────────────────────────────────────────────────────────┤
    │ 1. Trajectory Execution Watchdog (5.0s max per segment)  │
    │    - Automatically aborts trajectory if an arm stalls.   │
    ├──────────────────────────────────────────────────────────┤
    │ 2. Serial Heartbeat Watchdog (500 ms loss threshold)     │
    │    - Halts computation graph if UART line drops.         │
    ├──────────────────────────────────────────────────────────┤
    │ 3. Servo Thermal Duty-Cycle Limiter                      │
    │    - Commands compact tucked Home pose between picks     │
    │    - Inserts 500 ms cooling pause between cycles.        │
    └──────────────────────────────────────────────────────────┘
```

| Watchdog / Fault | Trigger Condition | System Reaction | Recovery Action |
| :--- | :--- | :--- | :--- |
| **Trajectory Stall** | Arm trajectory execution exceeds $> 5.0\text{ s}$ timeout. | Aborts current motion; transitions manipulator to `ERROR_STALL`. | Unloads motor torque, returns to safe Home pose, and re-scans arena. |
| **Serial Disconnect** | Controller stops responding to status queries for $> 500\text{ ms}$. | Sets `ArmStatus.is_busy = False`, halts task dispatch. | Node attempts automatic serial reconnection (`/dev/ttyUSB0`) up to 3 times. |
| **Voltage Brownout** | Bus voltage drops below $6.2\text{ V}$ during joint acceleration. | Limits maximum trajectory acceleration profile by 40%. | Logs power warning to dashboard; prompts inspection of 7.4V PSU. |
| **Thermal Protection** | Servo internal thermistor exceeds $65^\circ\text{C}$. | Arm returns immediately to compact tucked Home pose. | Arm remains idle in tucked configuration until temperature drops below $50^\circ\text{C}$. |
