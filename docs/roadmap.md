# Project Roadmap — Dual xArm 1s Color-Sorting Robot

---

## 🏫 College Week 1 — Environment Setup + Physical Assembly

**Constraint**: Hardware available in college lab from day one.  
**Goal**: Baseboard assembled, arms mounted, elevated arena built, NW/SW drop boxes installed, Central Electronics Bay wired.  
**Reference Document**: [`docs/01_arena_and_layout.md`](01_arena_and_layout.md) (Stepping Stone 1)

| Task | Detail |
|---|---|
| Set up Python venv & ROS 2 Humble | On Jetson Nano / RPi 5 (Ubuntu 22.04), install OpenCV and dependencies |
| Clone `xArm-Developer/xarm_ros2` | Verify SDK and serial driver communication |
| Mount 80×60 cm Baseboard | Secure flat baseboard on workbench |
| Mount Arms Front-to-Back | Mount Arm 2 (Front @ 180 mm) and Arm 1 (Back @ 420 mm) flat on baseboard ($Z=0$), collinear along $X = 360\text{ mm}$ ($S = 240\text{ mm}$) |
| Build Elevated Arena Platform | Cut 15–20 mm board for RIGHT side; cover with high-friction silicone mat |
| Mount 4 ArUco Markers on Arena | Mount Markers 0 & 2 collinear with arms; Markers 1 & 3 on outer boundary (all at $Z=15\text{--}20\text{ mm}$) |
| Install Sector Barriers | Place curved physical barriers at $R_{\min}=100\text{ mm}$ and $R_{\max}=240\text{ mm}$ on the right |
| Mount NW / SW Collection Boxes | Secure Box 1 in North-West ($\approx +125^\circ$) and Box 2 in South-West ($\approx -125^\circ$) at table level ($0\text{ mm}$) |
| Assemble Central Electronics Bay | Mount Jetson/RPi 5 and dual 7.4V/10A power hub in central bay ($X \in [55, 175], Y \in [215, 385]$) |
| Mount Overhead Camera | Boom stand at 60–75 cm height centered over right-side arena; confirm all 4 markers visible |
| Spray-Paint Cubes | 4 colors (Red, Green, Blue, Yellow) — 4–5 cubes each = ~16–20 cubes |

**Exit criteria**: Both arms home reliably. Base servo swing verified inside $240^\circ$ ROM. Camera sees the 4 arena ArUco markers clearly. Elevated arena, electronics bay, and drop boxes fixed.

---

## 🏫 College Week 2 — Vision Pipeline (Dedicated Arena Framing)

**Goal**: OpenCV pipeline running live, rectifying solely the elevated picking arena.

| Task | Detail |
|---|---|
| Dedicated Arena Homography | Detect the 4 arena markers → compute homography → rectify elevated arena to mm |
| Confirm Zero Box Intrusion | Verify left-side collection boxes and electronics bay are completely excluded from warped image |
| HSV Color Thresholding | Tune HSV ranges in `config/colors.yaml` under fixed camera exposure |
| Polar Frame Transformation | Convert arena $(X, Y)$ to polar $(r, \theta)$ relative to Arm 1 and Arm 2 bases |
| Boundary Sanity Filtering | Reject any detection outside $100\text{ mm} \le r \le 240\text{ mm}$ or out-of-sector |
| Coordinate Accuracy Test | Verify detected $(r, \theta)$ error is $< 3\text{ mm}$ against ruler ground truth |

**Deliverable**: Scatter 8 cubes on the raised arena → detected with high pixel density, correct color, and $(r, \theta)$ in < 1.5 seconds.

---

## 🏫 College Week 3 — ROS 2 Nodes + Pipelined Scheduler

**Goal**: Full ROS 2 graph operational; scheduler tested with single `arena_mutex`.

| Task | Detail |
|---|---|
| Write `vision_node` | Publishes `/detected_objects` with $(r, \theta, x, y)$ on scan trigger |
| Write `scheduler_node` | Assigns cubes to Arm 1 (Back) or Arm 2 (Front) based on midline ($Y = 300\text{ mm}$); manages `arena_mutex` |
| Write `arm_node` stub | Receives `ArmTask.msg`, simulates radial pick on right and 205° drop into NW/SW boxes on left |
| Write `simulate_detection.py` | Injects synthetic cube coordinates to stress-test task queue and mutex locking |
| Verify Zero-Drop Contention | Confirm Arm 1 and Arm 2 can drop into their NW/SW boxes simultaneously |

**Exit criteria**: Scheduler assigns tasks cleanly → zero mutex deadlocks in simulation.

---

## 🏫 College Week 4 — Single Arm, Radial Motion & Kinematics

**Goal**: One arm executes radial approach pick on elevated arena and drops into its dedicated NW/SW box.  
**Reference Document**: [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) (Stepping Stone 2)

| Task | Detail |
|---|---|
| Implement Analytical IK & Pitch LUT | Implement `XArmKinematics` solver from Stepping Stone 2 with dual-regime wrist roll |
| Calibrate NW/SW Bin Coordinates | Jog Arm 1 to Box 1 (NW) and Arm 2 to Box 2 (SW), record $(x, y, z)$ in `bin_coordinates.yaml` |
| Calibrate Pitch LUT | Empirically tune pitch angle $\alpha(r)$ from $R=10\text{ cm}$ ($45^\circ$) to $R=24\text{ cm}$ ($15^\circ$) |
| Implement Radial Standoff | Move to $r - 30\text{ mm}$, open claw, advance radially onto $15\text{ mm}$ raised surface |
| Verify Friction Grasp | Test clamping cubes on silicone mat—verify zero forward sliding |
| Run Full Pick-and-Drop Loop | Pick on right $\rightarrow$ ascend $\rightarrow$ swing 205° to NW/SW box $\rightarrow$ drop in cell $\rightarrow$ home |
| Repeat for xArm 2 | Calibrate Arm 2's Pitch LUT and SW box coordinates |

**Exit criteria**: Both arms individually pick from raised arena and sort into their dedicated NW/SW boxes with > 95% success.

---

## 🏫 College Week 5 — Single Arm + Camera Integration

**Goal**: Vision-guided sorting with one arm.

| Task | Detail |
|---|---|
| Connect Vision to Arm | `scheduler_node` feeds live vision coordinates to `arm_node` |
| Single Cube Sorting | Place one cube on right arena → detect → pick → sort into NW/SW box |
| Multi-Color Sequential Sort | Scatter 4 cubes (1 per color) → arm sorts all into correct cells |
| Grip Retry Routine | If grip feedback fails, arm returns home and re-attempts |

**Exit criteria**: 4 cubes scattered on right arena sorted cleanly into dedicated box by one arm.

---

## 🏫 College Week 6 — Full Single-Arm Capacity Test

**Goal**: Robust single-arm sorting of 16–20 cubes.

| Task | Detail |
|---|---|
| Full Queue Run | Scatter 16+ cubes across arm's arc sector → complete sorting queue |
| Boundary Cube Tests | Test cubes at inner boundary ($10\text{ cm}$) and outer boundary ($24\text{ cm}$) |
| Cycle Time Tuning | Optimize joint speed during 205° swing while maintaining smooth acceleration |

**Exit criteria**: 16 cubes sorted consecutively without grip drops or motor stalling.

---

## 🏫 College Weeks 7–8 — Dual-Arm Pipelined Integration

**Goal**: Both arms sorting simultaneously across the continuous two-center arena.

| Task | Detail |
|---|---|
| Launch Both Arm Nodes | Run `arm_1` and `arm_2` concurrently with independent hardware serial ports |
| Pipelined Mutex Verification | Verify Arm 2 picks on right while Arm 1 drops in Box 1 (NW), and vice-versa |
| Midline Object Handover | Test cubes placed directly on the dividing line; confirm deterministic assignment |
| Full Dual-Arm Run | Scatter 16–20 cubes across the entire right arena → dual arms sort in parallel |
| Throughput Benchmark | Compare sort time: single arm vs. dual arm (expect ~1.8× speedup) |

**Exit criteria**: Complete arena sorted by dual arms with zero collisions and zero idle waiting.

---

## 🏫 College Weeks 9–10 — Dashboard, Polish + Robustness

**Goal**: Live dashboard connected, system demo-ready.

| Task | Detail |
|---|---|
| Build Web Dashboard | Two-panel UI: live annotated camera feed (left), real-time arm states & queue (right) |
| Connect `rosbridge_server` | Stream topics over WebSocket to browser |
| 5× Back-to-Back Stress Test | Run 5 complete sorting cycles back-to-back without failure |
| Lock Illumination & Wire Looming | Secure USB cables, power lines in Central Bay, and LED ring light |

**Exit criteria**: 3 consecutive flawless sorting runs with live dashboard operating.

---

## 🏫 College Week 11 — Demo Prep + Documentation

| Task |
|---|
| Record high-quality demo video (multi-angle footage) |
| Prepare presentation slide deck |
| Finalize engineering project report |
| Review setup instructions and code documentation |
