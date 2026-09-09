# Project Roadmap — Dual xArm 1s Color-Sorting Robot

---

## 🏫 College Week 1 — Environment Setup + Physical Assembly

**Constraint**: First week at college — hardware is available from day one.
**Goal**: Development environment ready, arms connected, arena physically built.
**Goal**: Development environment ready, arms connected, 80×60 cm arena physically built.

| Task | Detail |
|---|---|
| Install Python, OpenCV, Git on compute unit | `pip install opencv-python`, verify `import cv2` works |
| Install ROS 2 Humble (Ubuntu 22.04) | On Jetson Nano or RPi 5 — follow official ROS 2 install guide |
| Set up Python venv & ROS 2 Humble | On Jetson Nano / RPi 5 (Ubuntu 22.04), install OpenCV and dependencies |
| Clone `xArm-Developer/xarm_ros2` | Read SDK and driver structure |
| Create GitHub repo | Full folder structure from the implementation plan |
| Set up config files | `config/colors.yaml` and `config/bin_coordinates.yaml` (placeholders) |
| Measure xArm 1s reach | Use xArm Studio: jog arm to max reach in all directions, record radius in mm |
| Determine arena dimensions | Arena half = one arm's reachable area. Total = 2×, with margin |
| Build arena boundary | Wood strips or tape defining arena edges on the table |
| Place and fix arm mounts | Position both arms on the left edge, spaced so rotation arcs don't overlap |
| Mount camera overhead | Boom tripod, camera centered above arena, confirm full arena visible in frame |
| Place ArUco markers | Tape printed markers to arena corners, verify OpenCV detects all 4 |
| Connect arms over WiFi | Assign static IPs, verify `ping` and SDK `move_gohome()` on both arms |
| Confirm compute unit | Ubuntu + ROS 2 Humble running, both arm IPs reachable |
| Build collection box | 2×2 grid with physical dividers, fixed to table left of the arms |
| Measure xArm 1s reach | Verify $R_{min} \approx 10\text{ cm}$ and $R_{max} \approx 26\text{ cm}$ |
| Prepare 80×60 cm Baseboard | Mount flat board on table; place 4 ArUco markers at outer corners |
| Mount arms side-by-side | Space arms 25–30 cm apart along mid-baseline |
| Mount camera overhead | Boom tripod centered at 75–85 cm height; confirm full 80×60 cm board is visible |
| Install curved sector barriers | Form physical boundaries along $R_{min}=10\text{ cm}$ and $R_{max}=26\text{ cm}$ arcs |
| Build 2× separate 2×2 boxes | One dedicated box placed behind each arm (~180°–220° swing) |
| Connect arms over WiFi/Serial | Verify static IPs, `ping`, and SDK homing on both arms |
| Spray-paint cubes | 4 colors (Red, Green, Blue, Yellow) — 4–5 cubes per color = ~16–20 cubes total |

**Exit criteria**: Both arms home on command. Camera sees full arena. ArUco markers detected. Cubes painted and ready.
**Exit criteria**: Both arms home on command. Camera sees full 80×60 cm board with all 4 ArUco markers. Cubes and physical borders ready.

---

## 🏫 College Week 2 — Vision Pipeline
## 🏫 College Week 2 — Vision Pipeline (Global Frame & Polar Transform)

**Goal**: Vision pipeline running live with real camera, detecting real cubes.
**Goal**: Vision pipeline running live with real camera, detecting real cubes in polar coordinates.

| Task | Detail |
|---|---|
| Write `aruco_calibration.py` | Detect 4 ArUco markers → compute homography → warp frame to top-down view |
| Write `color_detect.py` | HSV thresholding → contours → centroids → mm coordinates |
| Write `pipeline.py` | Combine calibration + detection into a single callable function |
| Plug camera into compute unit | Verify OpenCV captures from USB: `cv2.VideoCapture(0)` |
| Run ArUco homography live | Confirm all 4 markers detected, warp looks correct |
| Test HSV for each color | Place one cube at a time, tune ranges in `colors.yaml` until detection is clean |
| Test all 4 colors together | Scatter 4+ cubes, verify all detected with correct labels |
| Launch `vision_node` in ROS 2 | Confirm `/detected_objects` publishes with correct mm coordinates |
| Verify coordinate accuracy | Place cube at known position (measured by ruler), compare against detected mm — error < 5 mm |
| Write `aruco_calibration.py` | Detect 4 corner markers → compute homography → rectify 80×60 cm board to global mm |
| Write `color_detect.py` | HSV thresholding for 4 colors → contours → centroids in global mm |
| Convert to Arm Polar Frame | Transform $(X_g, Y_g)$ to $(r, \theta)$ relative to Arm 1 and Arm 2 bases |
| Software Boundary Filter | Discard/flag cubes outside $100\text{ mm} \le r \le 260\text{ mm}$ or out-of-sector |
| Write `pipeline.py` | Combine calibration + detection + polar transform into single callable module |
| Calibrate HSV ranges | Tune `config/colors.yaml` under lab lighting until detection is 100% clean |
| Verify coordinate accuracy | Place cube at ruler-measured position, confirm error < 5 mm in $(r, \theta)$ |

**Deliverable**: Scatter 8 cubes → all detected with correct color and position in < 2 seconds.
**Deliverable**: Scatter 8 cubes → all detected with correct color, $(X, Y)$ global mm, and $(r, \theta)$ in < 2 seconds.

**Exit criteria**: Consistent color detection for all 4 colors live on the real camera feed.

---

## 🏫 College Week 3 — ROS 2 Nodes + Scheduler Logic
## 🏫 College Week 3 — ROS 2 Nodes + Pipelined Scheduler

**Goal**: Full ROS 2 node graph running; scheduler logic tested end-to-end.
**Goal**: Full ROS 2 node graph running; scheduler tested with single `arena_mutex`.

| Task | Detail |
|---|---|
| Write `vision_node` | Wraps `pipeline.py`, subscribes to `/camera/image_raw`, publishes `/detected_objects` on trigger |
| Write `scheduler_node` | Receives object list, splits by y-midpoint, builds two task queues, implements mutex logic |
| Write `arm_node` stub | Receives tasks, prints them — no real arm movement yet |
| Write `simulate_detection.py` | Inject fake detected-object messages to test the scheduler without camera or arms |
| Test scheduler in simulation | Verify task assignment, mutex behavior, pipeline sequencing with fake data |
| Write `vision_node` | Wraps `pipeline.py`, publishes `/detected_objects` with $(r, \theta, x, y)$ |
| Write `scheduler_node` | Assigns cubes to arms by y-midline, manages `arena_mutex` (no box mutex needed) |
| Write `arm_node` stub | Receives `ArmTask.msg` with $(r, \theta, z)$, simulates pick and drop timing |
| Write `simulate_detection.py` | Injects synthetic cube coordinates to stress-test queueing and mutex locking |
| Test pipelining logic | Confirm Arm 1 picks while Arm 2 drops, without race conditions |
| Define custom ROS 2 messages | `DetectedObject.msg`, `ArmTask.msg`, `ArmStatus.msg` in `custom_msgs/` |

**Exit criteria**: Simulated object list → scheduler assigns tasks correctly to both arm stubs → no mutex conflicts.
**Exit criteria**: Simulated object list → scheduler assigns tasks correctly → zero mutex deadlock.

---

## 🏫 College Week 4 — Single Arm, Fixed Positions
## 🏫 College Week 4 — Single Arm, Radial Motion & Pitch LUT

**Goal**: One arm picks a cube from a fixed point and drops it into each bin cell. No camera integration yet.
**Goal**: One arm executes radial approach pick and places into its dedicated rear 2×2 box.

| Task | Detail |
|---|---|
| Jog arm to each bin cell manually | Use xArm Studio to jog to each of the 4 cells, record (x, y, z) for `bin_coordinates.yaml` |
| Test fixed pick → fixed drop | Hardcode one pick position and one bin position, run the full motion sequence |
| Tune pick height and grip force | Adjust until grip is consistent — aim for 9/10 success on repeated picks |
| Test all 4 bin cells | Drop a cube into each cell in sequence, verify positions are correct |
| Add grip feedback | Read gripper servo position post-grip → log success/failure |
| Repeat for xArm 2 | Measure and record its 4 bin coordinates separately |
| Measure rear bin coordinates | Jog arm to its 4 rear bin cells (~180°–220° swing), record in `bin_coordinates.yaml` |
| Calibrate Pitch LUT | Empirically determine pitch angle $\alpha(r)$ for $r = 10\text{ cm}$ (steep) to $26\text{ cm}$ (shallow) |
| Lock wrist roll level | Ensure `roll = 0°` throughout approach to prevent claw tips touching table |
| Implement radial standoff | Move to $(r - 30\text{ mm})$, open 55mm claw, advance radially into cube |
| Verify mechanical self-centering | Test gripping rotated 25mm cubes without wrist roll rotation |
| Test fixed pick → rear drop | Run full sequence: radial pick → ascend → rotate rear → drop into cell |
| Repeat for xArm 2 | Calibrate its Pitch LUT and rear bin coordinates |

**Exit criteria**: Each arm individually picks from a fixed point and places into any of 4 cells reliably.
**Exit criteria**: Both arms individually execute reliable radial pick and rear drop for all 4 cells.

---

## 🏫 College Week 5 — Single Arm + Camera Integration

**Goal**: Camera detects cube → arm picks and sorts it.

| Task | Detail |
|---|---|
| Connect vision → arm | `scheduler_node` receives `/detected_objects`, dispatches task to one arm node |
| Single cube test | Place one cube → detect → pick → sort into correct bin |
| Multiple cubes, one arm | Scatter 4 cubes (one per color) → arm sorts all sequentially |
| Verify end-to-end accuracy | Check that arm lands on the cube (not offset). Tune coordinate transform if needed |
| Add retry on grip failure | Arm returns home → re-scan → retry failed object |

**Exit criteria**: 4 cubes (one per color) placed randomly → single arm sorts all into correct bins.

---

## 🏫 College Week 6 — Full Single-Arm Sort

**Goal**: Robust single-arm sort of all 16–20 cubes (multiple cubes per color).

| Task | Detail |
|---|---|
| Full queue test | Scatter all 16–20 cubes → arm builds and executes full task list |
| Edge cases | Test cubes near walls, near arena center, cubes close together |
| Speed tuning | Increase arm speed to find fastest reliable operation without grip failures |

**Exit criteria**: 16+ cubes sorted correctly by one arm, consistently, 3 runs in a row.

---

## 🏫 College Weeks 7–8 — Dual Arm Integration

**Goal**: Both arms sort simultaneously using the pipeline scheduler.
**Goal**: Both arms sort simultaneously using the pipelined scheduler.

| Task | Detail |
|---|---|
| Bring up second arm node | Launch both `arm_node` instances with separate `arm_id` parameters |
| Test scheduler split | Verify objects in top half go to xArm 1, bottom half to xArm 2 |
| Enable mutex pipeline | Both arms run in pipeline: one picks while the other drops |
| Enable pipelined execution | While Arm 1 picks in arena, Arm 2 drops in rear Box 2, and vice-versa |
| Test center-line objects | Place cubes on y-midline → verify deterministic assignment to xArm 1 |
| Test simultaneous box access | Verify box mutex prevents both arms dropping at once |
| Full dual-arm run | Scatter 16–20 cubes → both arms sort in parallel |
| Measure throughput | Compare sort time: single arm vs. dual arm |
| Verify zero drop contention | Confirm both arms can access their private rear boxes without any waiting or locking |
| Full dual-arm run | Scatter 16–20 cubes → both arms sort in parallel across overlapping sectors |
| Measure throughput | Compare sort time: single arm vs. dual arm (expect ~1.8x speedup) |

**Exit criteria**: Full arena of cubes sorted correctly by both arms together, no collisions, all bins correct.

---

## 🏫 College Weeks 9–10 — Dashboard, Polish + Robustness

**Goal**: System is demo-ready and reliable.

| Task | Detail |
|---|---|
| Write `dashboard/index.html`, `style.css`, `app.js` | Two-panel layout: annotated video (left), stats panel (right) |
| Launch `rosbridge_server` | Connect dashboard WebSocket to live ROS 2 topics |
| Live annotated feed | Annotated MJPEG stream in left panel |
| Live stats panel | Arm states, queue count, grip log updating in real time |
| Robustness testing | Run 5× back-to-back full sorts without failure |
| Lighting finalization | Lock LED setup, re-verify HSV ranges under final lighting conditions |
| Motion path optimization | Reduce unnecessary arm travel; minimize cycle time per cube |
| Wide range demo | Place cubes at far corners of arena — verify arm reaches without struggle |

**Exit criteria**: 3 clean consecutive full-sort runs with live dashboard visible.

---

## 🏫 College Week 11 — Demo Prep + Documentation

| Task |
|---|
| Record clean demo video (multiple runs, different camera angles) |
| Prepare presentation slides: motivation → system design → pipeline → results |
| Write project report |
| Update README: setup steps, wiring, how to run |
| Package final code — tag release on GitHub |

---

## Timeline Summary

| Period | Focus | Key Deliverable |
|---|---|---|
| College Week 1 | Environment setup + physical assembly | Arms connected, arena built, cubes painted |
| College Week 2 | Vision pipeline on real hardware | All 4 colors detected live with mm coordinates |
| College Week 3 | ROS 2 nodes + scheduler logic | Full node graph tested end-to-end in simulation |
| College Week 4 | Single arm fixed positions | Reliable pick-and-place, bin coordinates logged |
| College Week 5 | Single arm + camera | Vision-guided sorting, one arm |
| College Week 1 | Environment setup + physical assembly | Arms connected, 80×60 cm arena built, cubes painted |
| College Week 2 | Vision pipeline (Global mm & Polar) | All 4 colors detected live with $(r, \theta)$ coordinates |
| College Week 3 | ROS 2 nodes + pipelined scheduler | Node graph tested end-to-end with `arena_mutex` |
| College Week 4 | Radial motion & Pitch LUT | Reliable radial pick-and-place into rear boxes |
| College Week 5 | Single arm + camera integration | Vision-guided radial sorting, one arm |
| College Week 6 | Full single-arm sort | 16+ cubes sorted, robust |
| College Weeks 7–8 | Dual arm integration | Both arms pipeline-sorting simultaneously |
| College Weeks 7–8 | Dual arm pipelined integration | Both arms sorting simultaneously with private rear boxes |
| College Weeks 9–10 | Dashboard + polish | Demo-ready, 3 clean runs |
| College Week 11 | Demo prep | Video, slides, report, GitHub release |

