# Stepping Stone 3: Computer Vision & Camera Calibration Pipeline

This document details the complete **optical, algorithmic, and mathematical specifications** for the overhead vision system. It covers camera mounting optics, planar homography rectification using ArUco fiducials, illumination compensation, the **Color Discrimination Methodology** (Red target picking, Green distractor filtering, Blue chassis exclusion, and spatial masking), geometric centroid extraction, cube orientation analysis, and coordinate transformations into robot polar space.

---

## 1. Vision Architecture Overview

The perception pipeline is built on **pure classical computer vision (OpenCV 4.x)** running under **ROS 2 Humble**:

```text
       Overhead USB 1080p Webcam (V4L2 @ 60–65 cm Height)
                           │
                           ▼
               [ Raw Camera Frame (1920×1080) ]
                           │
             ┌─────────────┴─────────────┐
             │                           │
             ▼ (Continuous @ 30 FPS)     ▼ (On-Demand / Streaming Scan)
     [ Web Dashboard Stream ]     [ ArUco Marker Detection ]
     (Annotated Telemetry)        (DICT_4X4_50, IDs: 0, 1, 2, 3 Clockwise)
                                         │
                                         ▼
                               [ Planar Homography H ]
                               (Rectify to 2.0 px/mm Grid)
                                         │
                                         ▼
                             [ Metric Warped Arena ]
                             (600 px × 800 px View)
                                         │
                                         ▼
                             [ Chassis Exclusion Mask ]
                             (Rows 3..5, Cols 2..4 Zeroed Out)
                                         │
                                         ▼
                             [ HSV Color Segmentation ]
                             (Red Dual-Band Target | Green Distractor)
                                         │
                                         ▼
                               [ Morphological Clean-Up ]
                               (Kernel: 5×5 Ellipse Open/Close)
                                         │
                                         ▼
                               [ Contour & Moment Analysis ]
                               (Filter Area 625 mm² ± 40%)
                                         │
                                         ▼
                               [ Centroid (X_w, Y_w) & Yaw ψ ]
                                         │
                                         ▼
                           [ Robot Polar Frame Transform ]
                           (ΔX, ΔY → r, θ, d_edge for xArm 1S)
                                         │
                                         ▼
                            [ /detected_objects (ROS 2) ]
                            (Target: Red Only | Distractor: Green)
```

### Key Engineering Objectives

1. **100% Dedicated Metric Resolution**: The active arena framed by the 4 ArUco markers ($300\text{ mm} \times 400\text{ mm}$) is warped to an exact $600 \times 800\text{ px}$ image at $2.0\text{ px/mm}$ ($1\text{ pixel} = 0.5\text{ mm}$).
2. **Deterministic Sub-25 ms Latency**: OpenCV contour moments guarantee scan cycle completion in $< 25\text{ ms}$, ensuring zero motion bottleneck.
3. **Sub-2 mm Spatial Accuracy**: Sub-pixel ArUco corner refinement combined with coplanar homography guarantees cube centroid error $< 2.0\text{ mm}$.
4. **Strict Color Discrimination**: Red target cubes are prioritized; green distractor cubes are tracked but excluded from the pick queue; blue metallic reflections and chassis structures are completely eliminated.

---

## 2. Camera Hardware & Mounting Optics

### 2.1 Hardware Specification

| Parameter              | Value / Specification                                                                                     |
| ---------------------- | --------------------------------------------------------------------------------------------------------- |
| **Camera Model**       | Logitech C920 / C270 or generic 1080p UVC USB Webcam                                                      |
| **Interface**          | Direct USB 2.0 / 3.0 (`/dev/video0`)                                                                      |
| **Backend Driver**     | Linux Video4Linux2 (`V4L2`) via `cv2.CAP_V4L2`                                                            |
| **Native Resolution**  | $1920 \times 1080\text{ pixels}$ @ 30 FPS                                                                 |
| **Lens Focal Length**  | $f \approx 3.67\text{ mm}$ (Diagonal FOV: $\approx 78^\circ$, Horizontal FOV: $\approx 70.4^\circ$)       |
| **Mount Position**     | Freestanding overhead stand centered at $(X = 175.0\text{ mm}, Y = 250.0\text{ mm})$                      |
| **Mount Height ($H$)** | **$600\text{--}650\text{ mm}$** above baseboard ($>150\text{ mm}$ clear margin above highest robot pose)  |

### 2.2 Optical Geometry & Coverage Verification

The active region framed by the 4 ArUco markers measures $300.0\text{ mm} \text{ (Width)} \times 400.0\text{ mm} \text{ (Depth)}$:
$$\text{Width } (X) = 325 - 25 = 300\text{ mm}, \quad \text{Height } (Y) = 425 - 25 = 400\text{ mm}$$

At a camera height of $H = 620\text{ mm}$ with a horizontal FOV of $\theta_H = 70.4^\circ$:
$$\text{Coverage}_X = 2 \cdot H \cdot \tan\left(\frac{70.4^\circ}{2}\right) = 2 \cdot 620 \cdot \tan(35.2^\circ) \approx 2 \cdot 620 \cdot 0.705 = \mathbf{874\text{ mm}}$$
$$\text{Coverage}_Y = \text{Coverage}_X \cdot \left(\frac{1080}{1920}\right) = 874 \cdot 0.5625 = \mathbf{491\text{ mm}}$$

Because $\text{Coverage}_X (874\text{ mm}) > 350\text{ mm}$ and $\text{Coverage}_Y (491\text{ mm}) > 400\text{ mm}$, the entire arena, all 4 ArUco fiducials, and the pick zone fit comfortably within the camera frame with zero edge clipping.

### 2.3 Lighting Lock & Exposure Control

Ambient room lighting causes fatal HSV color drift. Auto-exposure and auto-white-balance **must be disabled in hardware**:

```bash
# Disable auto-exposure and set fixed exposure time
v4l2-ctl -d /dev/video0 -c exposure_auto=1
v4l2-ctl -d /dev/video0 -c exposure_absolute=156

# Disable auto white balance and lock color temperature to 4500K
v4l2-ctl -d /dev/video0 -c white_balance_temperature_auto=0
v4l2-ctl -d /dev/video0 -c white_balance_temperature=4500

# Fix gain and brightness
v4l2-ctl -d /dev/video0 -c gain=32
v4l2-ctl -d /dev/video0 -c brightness=128
```

---

## 3. Planar Homography & ArUco Metric Rectification

### 3.1 Coplanar ArUco Fiducial Setup

Four ArUco markers (`DICT_4X4_50`, size $50\text{ mm} \times 50\text{ mm}$) are mounted flat to the platform board at $Z = 0.0\text{ mm}$ in clockwise order:

```text
  Y (mm)
   425 ┌─────────────────────────────────────────────────────────────┐
       │ [ArUco #0] (Row 3, Col 0)         [ArUco #1] (Row 3, Col 6) │
       │ (25, 425)                          (325, 425)               │
       │                                                             │
       │                     ACTIVE PICKING ARENA                    │
       │                      (Z = 0.0 mm Plane)                     │
       │                                                             │
       │ [ArUco #3] (Row 11, Col 0)        [ArUco #2] (Row 11, Col 6)│
    25 │ (25, 25)                           (325, 25)                │
       └─────────────────────────────────────────────────────────────┘
       25                                                          325 ───> X (mm)
```

| Marker ID | Placement Role | World Coordinate $(X_w, Y_w)$ | Physical Elevation | Rectified Pixel $(u, v)$ |
| :---: | :--- | :---: | :---: | :---: |
| **ID 0** | Row 3, Col 0 (Rear-Left) | $(25.0\text{ mm}, 425.0\text{ mm})$ | $Z = 0.0\text{ mm}$ | $(0, 0)$ |
| **ID 1** | Row 3, Col 6 (Rear-Right) | $(325.0\text{ mm}, 425.0\text{ mm})$ | $Z = 0.0\text{ mm}$ | $(600, 0)$ |
| **ID 2** | Row 11, Col 6 (Front-Right) | $(325.0\text{ mm}, 25.0\text{ mm})$ | $Z = 0.0\text{ mm}$ | $(600, 800)$ |
| **ID 3** | Row 11, Col 0 (Front-Left) | $(25.0\text{ mm}, 25.0\text{ mm})$ | $Z = 0.0\text{ mm}$ | $(0, 800)$ |

> [!CRITICAL]
> **Strict Table Coplanarity**: All 4 markers share the exact elevation of the arena floor ($Z = 0.0\text{ mm}$). Planar homography produces a mathematically exact perspective warp ($H$) into a rectified $600 \times 800\text{ px}$ grid at $2.0\text{ px/mm}$ ($1\text{ px} = 0.5\text{ mm}$) with zero elevation-induced parallax error!

### 3.2 Homography Derivation & Resolution

The homography matrix $\mathbf{H} \in \mathbb{R}^{3 \times 3}$ maps raw camera pixels $[u, v, 1]^T$ to metric pixels $[u', v', 1]^T$:
- **Rectified Width**: $W_{\text{rect}} = (325.0 - 25.0) \times 2.0 = \mathbf{600\text{ pixels}}$
- **Rectified Height**: $H_{\text{rect}} = (425.0 - 25.0) \times 2.0 = \mathbf{800\text{ pixels}}$

Clockwise destination points:
- $\mathbf{p}'_0 = (0, 0)$
- $\mathbf{p}'_1 = (600, 0)$
- $\mathbf{p}'_2 = (600, 800)$
- $\mathbf{p}'_3 = (0, 800)$

---

## 4. Color Discrimination Methodology

### 4.1 Target vs. Distractor Discrimination Strategy

In practical robotic workcell operations, non-target objects and structural components frequently confuse single-color computer vision detectors. The single xArm 1S system implements a rigorous three-tier discrimination methodology:

1. **Target Object (Red Craft Cubes)**:
   - Primary pick target.
   - Segmented using dual HSV hue bands to accommodate the cyclic wrap of red around $180^\circ$:
     - Band 1: $H \in [0, 10], S \in [120, 255], V \in [70, 255]$
     - Band 2: $H \in [170, 180], S \in [120, 255], V \in [70, 255]$
   - Merged via bitwise OR: $\text{Mask}_{\text{red}} = \text{Mask}_1 \lor \text{Mask}_2$.
   - Validated contours are assigned `color = "red"` and flagged `targetable: true` for the autonomous pick queue.

2. **Distractor Object (Green Craft Cubes)**:
   - Present on the active arena to stress-test discrimination robustness.
   - Segmented using calibrated green band: $H \in [38, 82], S \in [100, 255], V \in [60, 255]$.
   - Detected and tracked for telemetry, HUD visualization, and collision modeling, but explicitly flagged **`targetable: false`**.
   - The autonomous sorter node (`autonomous_sorter_node.py`) strictly rejects green cubes from the execution queue, ensuring they remain undisturbed on the arena floor.

3. **Chassis Elimination (Blue Bypass & Spatial Masking)**:
   - **Why Blue is Excluded**: The Hiwonder / LewanSoul xArm 1S manipulator chassis is constructed from anodized metallic **blue** aluminum brackets. If blue color segmentation were active, the robot's own links, wrist brackets, and specular LED glints would trigger massive, false-positive object contours, causing erratic tracking and trajectory aborts. Therefore, blue color detection is completely disabled (`enabled: false`).
   - **Chassis Spatial Exclusion Mask**: In addition to color filtering, a geometric exclusion mask is applied in image space over the robot mounting zone (Rows 3..5, Cols 2..4):
     $$X_w \in [100.0, 250.0\text{ mm}], \quad Y_w \in [300.0, 450.0\text{ mm}]$$
     In rectified image coordinates ($u, v$):
     $$u \in [150, 450\text{ px}], \quad v \in [0, 250\text{ px}]$$
     All pixels inside this rectangular window are explicitly zeroed out (`mask[v1:v2, u1:u2] = 0`) before contour finding, ensuring the physical baseplate and cables can never produce false cube detections.

---

## 5. Contour Extraction, Centroiding & Orientation Analysis

### 5.1 Area & Geometry Filtering

A $25.4\text{ mm} \times 25.4\text{ mm}$ wooden cube has a nominal top-down area of:
$$A_{\text{nominal}} = 25.4\text{ mm} \times 25.4\text{ mm} \approx 645\text{ mm}^2$$
At $\text{PPM} = 2.0\text{ px/mm}$:
$$A_{\text{pixels}} = 645 \times (2.0)^2 \approx \mathbf{2{,}580\text{ pixels}^2}$$

- **Valid Contour Area Filter**: $A \in [1{,}400, 3{,}800\text{ pixels}^2]$ ($\approx 350\text{--}950\text{ mm}^2$).
- **Bounding Box Aspect Ratio**: $\text{Aspect} = \frac{\max(w, h)}{\min(w, h)} \le 1.45$ (rejects elongated non-square artifacts).

### 5.2 Centroid Extraction via Spatial Moments

For a binary contour $C$, the centroid $(\bar{u}, \bar{v})$ is:
$$\bar{u} = \frac{M_{10}}{M_{00}}, \quad \bar{v} = \frac{M_{01}}{M_{00}}$$

### 5.3 Cube Yaw Orientation ($\psi$)

Using `cv2.minAreaRect(contour)`:
- Rotated box returns angle $\psi$ normalized to $[-45^\circ, +45^\circ]$ relative to the radial approach axis.
- Gripper wrist roll (Motor 2) aligns directly with $\psi$ during near/mid picks.

---

## 6. Metric Coordinate Transformation (Vision to Robot Frames)

Once a cube centroid $(\bar{u}, \bar{v})$ is extracted from the rectified $600 \times 800\text{ px}$ image:

### 6.1 World Coordinate Mapping $(X_w, Y_w)$

Since the rectified origin $(0, 0)$ corresponds to $(X = 25.0\text{ mm}, Y = 425.0\text{ mm})$:
$$X_w = 25.0 + \frac{\bar{u}}{\text{PPM}} = 25.0 + \frac{\bar{u}}{2.0}$$
$$Y_w = 425.0 - \frac{\bar{v}}{\text{PPM}} = 425.0 - \frac{\bar{v}}{2.0}$$
$$Z_w = 12.5\text{ mm} \quad (\text{table deck } 0.0\text{ mm} + \text{half cube height } 12.5\text{ mm})$$

### 6.2 Robot Polar Coordinate Mapping $(r, \theta)$

The single xArm 1S is mounted at $(X_{\text{base}} = 175.0\text{ mm}, Y_{\text{base}} = 375.0\text{ mm})$ with nominal heading $\theta_{\text{nom}} = -35.0^\circ$:

$$\Delta X = X_w - 175.0\text{ mm}$$
$$\Delta Y = Y_w - 375.0\text{ mm}$$
$$r = \sqrt{(\Delta X)^2 + (\Delta Y)^2}$$
$$\theta_{\text{world}} = \operatorname{atan2}(\Delta Y, \Delta X) \quad [\text{degrees}]$$
$$\theta_{\text{arm}} = \theta_{\text{world}} - (-35.0^\circ) = \theta_{\text{world}} + 35.0^\circ$$
Normalize $\theta_{\text{arm}}$ to $[-180^\circ, +180^\circ]$.

### 6.3 Reach Validation & Safety Clamping

A detected red cube is accepted into the pick queue if and only if:
$$160.0\text{ mm} \le r \le 300.0\text{ mm}$$
$$X_w \in [25.0, 325.0\text{ mm}], \quad Y_w \in [25.0, 300.0\text{ mm}]$$

---

## 7. Calibration Tooling & Verification Scripts

### 7.1 `vision/calibrate_camera.py` (Intrinsic Lens Calibration)
- $9 \times 6$ checkerboard ($25\text{ mm}$ square size).
- Solves camera intrinsic matrix $\mathbf{K}$ and distortion coefficients $[k_1, k_2, p_1, p_2, k_3]$.
- Exports atomically to `config/camera_calibration.yaml`. Supports `--synthetic` for CI/CD.

### 7.2 `vision/tune_hsv.py` (Interactive HSV Tuner)
- 6 trackbars (`H_min`, `H_max`, `S_min`, `S_max`, `V_min`, `V_max`).
- Keys `1` (Red), `2` (Green) to switch tuning profile. Press `s` to save to `config/colors.yaml`.

### 7.3 `vision/test_aruco_warp.py` (Metric Verification & Homography Lock)
- Detects the 4 ArUco markers (`DICT_4X4_50`, IDs 0..3) in clockwise order.
- Computes $600 \times 800\text{ px}$ homography warp at $2.0\text{ px/mm}$.
- Press `l` to lock homography in memory, `g` to display metric grid lines, `s` to save matrix to config.

### 7.4 Automated Vision Tests
- Run tests anytime via:
  ```bash
  pytest ros2_ws/src/sorting_arm/test/test_vision_pipeline.py -v
  ```
- All 6 unit tests verify ArUco detection, $600\times 800$ warp, Red/Green discrimination, chassis masking, and Nearest-First ordering.
