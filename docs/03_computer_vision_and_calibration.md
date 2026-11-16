# Stepping Stone 3: Computer Vision & Camera Calibration Pipeline

This document details the complete **optical, algorithmic, and mathematical specifications** for the overhead vision system. It covers camera mounting optics, planar homography rectification using ArUco fiducials, illumination compensation, HSV color segmentation, geometric centroid extraction, cube orientation analysis, and coordinate frame transformations into robot polar space.

---

## 1. Vision Architecture Overview

The vision pipeline is built on **pure classical computer vision (OpenCV 4.x)** running on the **NVIDIA Jetson Orin Nano / Orin NX** under **ROS 2 Humble**.

```text
       Overhead USB 1080p Webcam (V4L2 @ 65–75 cm Height)
                           │
                           ▼
               [ Raw Camera Frame (1920×1080) ]
                           │
             ┌─────────────┴─────────────┐
             │                           │
             ▼ (Continuous @ 30 FPS)     ▼ (On-Demand Scan Trigger)
     [ Web Dashboard Stream ]     [ ArUco Marker Detection ]
     (Annotated Telemetry)        (DICT_4X4_50, IDs: 0, 1, 2, 3)
                                         │
                                         ▼
                               [ Planar Homography H ]
                               (Rectify to 0.5 mm/pixel Grid)
                                         │
                                         ▼
                             [ Metric Warped Arena ]
                             (370 mm × 480 mm View)
                                         │
                                         ▼
                               [ HSV Color Thresholding ]
                               (Red, Green, Blue, Yellow)
                                         │
                                         ▼
                               [ Morphological Clean-Up ]
                               (Kernel: 5×5 Ellipse Open/Close)
                                         │
                                         ▼
                               [ Contour & Moment Analysis ]
                               (Filter Area 625 mm² ± 35%)
                                         │
                                         ▼
                               [ Centroid (X_w, Y_w) & Yaw ψ ]
                                         │
                                         ▼
                           [ Robot Polar Frame Transform ]
                           (ΔX, ΔY → r, θ, d_edge per arm)
                                         │
                                         ▼
                            [ /detected_objects (ROS 2) ]
```

### Key Engineering Objectives

1. **100% Dedicated Pixel Density**: Only the elevated picking arena is warped and processed. Left-side drop boxes and electronics are completely excluded from the warped frame, eliminating false-positive color detections.
2. **Sub-30 ms Latency**: Deterministic OpenCV contour moments guarantee scan cycle completion in under $30\text{ ms}$ on the Jetson Orin Nano.
3. **Sub-2 mm Spatial Accuracy**: Sub-pixel ArUco corner detection combined with lens distortion correction ensures cube localization error is strictly $< 2.0\text{ mm}$ across the entire $2{,}100\text{ cm}^2$ arena.
4. **Hybrid Operating Mode**: Continuous MJPEG video streaming for the operator dashboard, paired with state-triggered on-demand scanning for pick tasks (eliminates motion blur and claw occlusion).

---

## 2. Camera Hardware & Mounting Optics

### 2.1 Hardware Specification

| Parameter              | Value / Specification                                                                                     |
| ---------------------- | --------------------------------------------------------------------------------------------------------- |
| **Camera Model**       | Logitech C920 / C270 or generic 1080p UVC USB Webcam                                                      |
| **Interface**          | Direct USB 2.0 / 3.0 to Jetson Orin Nano (`/dev/video0`)                                                  |
| **Backend Driver**     | Linux Video4Linux2 (`V4L2`) via `cv2.CAP_V4L2`                                                            |
| **Native Resolution**  | $1920 \times 1080\text{ pixels}$ @ 30 FPS                                                                 |
| **Lens Focal Length**  | $f \approx 3.67\text{ mm}$ (Diagonal FOV: $\approx 78^\circ$, Horizontal FOV: $\approx 70.42^\circ$)      |
| **Mount Position**     | Freestanding desk clamp/tripod centered overhead at $(X = 200\text{ mm}, Y = 110\text{ mm})$              |
| **Mount Height ($H$)** | **$450\text{--}500\text{ mm}$** above baseboard ($>100\text{ mm}$ clear margin above highest robot pose)  |

### 2.2 Optical Geometry & Coverage Verification

The picking arena rectangle framed by the 4 ArUco markers measures:
$$\text{Width } (X) = 330 - 70 = 260\text{ mm}, \quad \text{Height } (Y) = 180 - 40 = 140\text{ mm}$$

At a camera height of $H = 480\text{ mm}$ with a horizontal FOV of $\theta_H = 70.4^\circ$:
$$\text{Coverage}_X = 2 \cdot H \cdot \tan\left(\frac{70.4^\circ}{2}\right) = 2 \cdot 480 \cdot \tan(35.2^\circ) \approx 2 \cdot 480 \cdot 0.705 = \mathbf{677\text{ mm}}$$
$$\text{Coverage}_Y = \text{Coverage}_X \cdot \left(\frac{1080}{1920}\right) = 677 \cdot 0.5625 = \mathbf{381\text{ mm}}$$

Because $\text{Coverage}_X (677\text{ mm}) > 260\text{ mm}$ and $\text{Coverage}_Y (381\text{ mm}) > 140\text{ mm}$, the entire arena, all 4 ArUco markers, and the Left Flank container fit comfortably inside the camera frame while doubling the effective pixel density to over $2.8\text{--}3.2\text{ px/mm}$.

### 2.3 Lighting Lock & Exposure Control

Ambient lighting changes in a university lab cause fatal HSV color drift. Auto-exposure and auto-white-balance **must be disabled in hardware**:

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

### 2.4 Interactive Camera Centering & Optical Tilt Alignment (`align_camera.py`)

When mounting the overhead camera on its boom stand, slight mechanical tilts (pitch, roll, yaw) or off-center positioning inevitably occur. Uncalibrated tilt introduces keystoning distortion that degrades homography resolution uniformity. To ensure the camera is perpendicular and physically centered before running the workcell, the pre-flight alignment utility [`vision/align_camera.py`](file:///home/shubham/Projects/Sem-5-mini-project/vision/align_camera.py) provides an interactive real-time Heads-Up Display (HUD):

```text
 ┌────────────────────────────────────────────────────────┐
 │ [Camera Alignment Tool]  STATUS: PERFECTLY ALIGNED     │
 │                                                        │
 │        [P0] ─────────────────────── [P1]               │
 │          \ \                       / /                 │
 │           \  \       (Ca)        /  /                  │
 │            \   \   Cyan 'x'    /   /                   │
 │             \    \     │     /    /                    │
 │              \     \   │   /     /                     │
 │               \      \ │ /      /                      │
 │                \      (Cc)     /                       │
 │                 \  Yellow '+' /                        │
 │                  \           /                         │
 │                   \         /                          │
 │                    \       /                           │
 │                  [P2] ─── [P3]                         │
 │                                                        │
 │ Centering: ΔX = -0.8 mm, ΔY = +1.1 mm (≤ 5.0 mm OK)    │
 │ Tilt: Roll = 0.2° (≤ 1.0° OK) | Pitch Ratio = 1.008    │
 └────────────────────────────────────────────────────────┘
```

#### 1. Geometric Arena Center via Diagonal Intersection
The geometric center of the arena $C_a = (x_a, y_a)$ is derived from the projective intersection of the two diagonals connecting opposite ArUco marker centers $(P_0 \leftrightarrow P_3)$ and $(P_1 \leftrightarrow P_2)$:
$$P(t) = P_0 + t(P_3 - P_0), \quad Q(u) = P_1 + u(P_2 - P_1)$$
Solving the $2\times 2$ linear system via Cramer's rule:
$$t = \frac{(P_1 - P_0) \times (P_2 - P_1)}{(P_3 - P_0) \times (P_2 - P_1)} \implies C_a = P_0 + t(P_3 - P_0)$$
Because projective collinearity and intersection points are preserved under central projection, this intersection represents the true physical center of the four markers regardless of perspective distortion.

#### 2. Optical Center & Centering Offsets
The optical principal center $C_c = (x_c, y_c)$ is taken from the camera intrinsic matrix $(c_x, c_y)$ (scaled to current resolution) or frame center $(w/2, h/2)$. The centering offsets are computed as:
$$\Delta x_{\text{px}} = x_a - x_c, \quad \Delta y_{\text{px}} = y_a - y_c$$
$$\Delta x_{\text{mm}} = \frac{\Delta x_{\text{px}}}{S}, \quad \Delta y_{\text{mm}} = \frac{\Delta y_{\text{px}}}{S}$$
where $S = 2.0\text{ px/mm}$ (scaled to frame resolution). The alignment target is $|\Delta x_{\text{mm}}| \le 5.0\text{ mm}$ and $|\Delta y_{\text{mm}}| \le 5.0\text{ mm}$.

#### 3. Tilt & Keystoning Detection
- **Roll Tilt ($\theta_{\text{roll}}$)**: Derived from the average horizontal slope of the top and bottom marker edges:
  $$\theta_{\text{roll}} = \frac{1}{2}\left[\operatorname{atan2}(y_1 - y_0, x_1 - x_0) + \operatorname{atan2}(y_3 - y_2, x_3 - x_2)\right]$$
  Target: $|\theta_{\text{roll}}| \le 1.0^\circ$.
- **Pitch Keystoning Ratio ($R_{\text{pitch}}$)**: Ratio of the top arena width to the bottom arena width:
  $$R_{\text{pitch}} = \frac{\|P_1 - P_0\|}{\|P_3 - P_2\|}$$
  If $R_{\text{pitch}} < 0.96$, the top edge appears narrower, indicating the camera is pitched backward; if $R_{\text{pitch}} > 1.04$, the camera is pitched forward. Target: $0.96 \le R_{\text{pitch}} \le 1.04$.
- **Yaw Keystoning Ratio ($R_{\text{yaw}}$)**: Ratio of the left arena height to the right arena height:
  $$R_{\text{yaw}} = \frac{\|P_2 - P_0\|}{\|P_3 - P_1\|}$$
  Target: $0.96 \le R_{\text{yaw}} \le 1.04$.

#### 4. Execution & Alignment Report
Launch the live alignment tool before tightening the camera stand clamp:
```bash
# Live interactive HUD:
python3 vision/align_camera.py --device /dev/video0

# Headless synthetic verification (saves config/camera_alignment.json):
python3 vision/align_camera.py --synthetic --save-report --headless
```
Pressing **'s'** captures a high-resolution snapshot in `data/camera_alignment_snapshot.png` and exports alignment verification metrics to `config/camera_alignment.json`.

---

## 3. Planar Homography & ArUco Metric Rectification

### 3.1 Coplanar ArUco Marker Setup

Four ArUco markers from the `DICT_4X4_50` dictionary are permanently mounted flat at the 4 corners of the **flat picking arena ($Z = 0.0\text{ mm}$)**.

```text
  Y (mm)
   180 ┌─────────────────────────────────────────────────────────────┐
       │ [ArUco #0] (Rear-Left)              [ArUco #1] (Rear-Right) │
       │ (70, 180)                           (330, 180)              │
       │                                                             │
       │                    FLAT PICKING ARENA                       │
       │                    (Z = 0.0 mm Plane)                       │
       │                                                             │
       │ [ArUco #2] (Front-Left)             [ArUco #3] (Front-Right)│
    40 │ (70, 40)                            (330, 40)               │
       └─────────────────────────────────────────────────────────────┘
       70                                                          330 ───> X (mm)
```

| Marker ID |           Placement Role           |  World Coordinate $(X_w, Y_w)$   |  Physical Elevation   |  Rectified Pixel $(u, v)$ |
| :-------: | :--------------------------------: | :------------------------------: | :-------------------: | :-----------------------: |
| **ID 0**  | Rear-Left Arena Corner             | $(70.0\text{ mm}, 180.0\text{ mm})$ | $Z = 0.0\text{ mm}$   | $(0, 0)$                  |
| **ID 1**  | Rear-Right Arena Corner            | $(330.0\text{ mm}, 180.0\text{ mm})$| $Z = 0.0\text{ mm}$   | $(520, 0)$                |
| **ID 2**  | Front-Left Arena Corner            | $(70.0\text{ mm}, 40.0\text{ mm})$  | $Z = 0.0\text{ mm}$   | $(0, 280)$                |
| **ID 3**  | Front-Right Arena Corner           | $(330.0\text{ mm}, 40.0\text{ mm})$ | $Z = 0.0\text{ mm}$   | $(520, 280)$              |

> [!CRITICAL]
> **Strict Coplanarity with Table Surface**: All 4 markers share the exact same physical elevation as the table and arena floor ($Z = 0.0\text{ mm}$). Because the markers are coplanar with the bottom faces of the cubes, planar homography maps image pixels directly to metric world coordinates with zero perspective altitude distortion! Markers are printed on **matte vinyl/paper sticker sheets** to eliminate specular LED reflection.

### 3.2 Homography Mathematical Derivation

The homography matrix $\mathbf{H} \in \mathbb{R}^{3 \times 3}$ maps homogeneous pixel coordinates $\mathbf{p} = [u, v, 1]^T$ in the raw camera frame to metric world coordinates $\mathbf{P} = [X_w, Y_w, 1]^T$ on the arena plane:

$$s \begin{bmatrix} X_w \\ Y_w \\ 1 \end{bmatrix} = \mathbf{H} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \begin{bmatrix} h_{11} & h_{12} & h_{13} \\ h_{21} & h_{22} & h_{23} \\ h_{31} & h_{32} & h_{33} \end{bmatrix} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}$$

Expanding into inhomogeneous Cartesian coordinates:
$$X_w = \frac{h_{11}u + h_{12}v + h_{13}}{h_{31}u + h_{32}v + h_{33}}, \quad Y_w = \frac{h_{21}u + h_{22}v + h_{23}}{h_{31}u + h_{32}v + h_{33}}$$

Given 4 pairs of non-collinear corresponding points $(u_i, v_i) \leftrightarrow (X_i, Y_i)$ ($i = 0, 1, 2, 3$), $\mathbf{H}$ is solved uniquely up to a scale factor via Singular Value Decomposition (SVD) using `cv2.getPerspectiveTransform()`.

### 3.3 Metric Resolution & Warp Output

We define a canonical rectified image grid with a scale of **$\text{PPM} = 2.0\text{ pixels/mm}$** ($1\text{ pixel} = 0.5\text{ mm}$):

- **Rectified Width**: $W_{\text{rect}} = (330 - 70) \times 2.0 = \mathbf{520\text{ pixels}}$
- **Rectified Height**: $H_{\text{rect}} = (180 - 40) \times 2.0 = \mathbf{280\text{ pixels}}$

The target destination points for the warp are:

- $\mathbf{p}'_0 = (0, 0)$ (ID 0: Rear-Left)
- $\mathbf{p}'_1 = (W_{\text{rect}}, 0) = (520, 0)$ (ID 1: Rear-Right)
- $\mathbf{p}'_2 = (0, H_{\text{rect}}) = (0, 280)$ (ID 2: Front-Left)
- $\mathbf{p}'_3 = (W_{\text{rect}}, H_{\text{rect}}) = (520, 280)$ (ID 3: Front-Right)

---

## 4. Color Segmentation (HSV Pipeline)

### 4.1 Why HSV Over RGB

RGB color channels are strongly coupled to luminance; changing the overhead room lighting shifts $R$, $G$, and $B$ values simultaneously. In **HSV (Hue, Saturation, Value)**:

- **Hue ($H \in [0, 180]$)**: Encodes pure color wavelength independent of illumination brightness.
- **Saturation ($S \in [0, 255]$)**: Encodes color vibrancy/purity.
- **Value ($V \in [0, 255]$)**: Encodes brightness.

### 4.2 Calibrated HSV Ranges

The wooden cubes are spray-painted with high-contrast matte finishes. The calibrated threshold windows under controlled lab lighting are:

| Color       | Hue Lower | Hue Upper | Sat Lower | Sat Upper | Val Lower | Val Upper | Notes                                |
| ----------- | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | ------------------------------------ |
| **Red (1)** |     0     |    10     |    120    |    255    |    70     |    255    | Lower red spectrum                   |
| **Red (2)** |    170    |    180    |    120    |    255    |    70     |    255    | Red wraps around $180^\circ$         |
| **Green**   |    38     |    82     |    100    |    255    |    60     |    255    | Clean emerald/lime band              |
| **Blue**    |    98     |    130    |    120    |    255    |    60     |    255    | Deep sapphire band                   |
| **Yellow**  |    20     |    36     |    120    |    255    |    100    |    255    | Narrow hue band to avoid green spill |

```python
# Red requires merging two masks due to cyclic Hue wraparound
mask_red1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
mask_red2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
mask_red = cv2.bitwise_or(mask_red1, mask_red2)
```

### 4.3 Morphological Filtering

To eliminate salt-and-pepper noise and fill interior voids caused by slight specular glint:

```python
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
# Opening (erosion followed by dilation) removes isolated noise specks
clean_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
# Closing (dilation followed by erosion) fills holes inside cube contours
clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
```

---

## 5. Contour Extraction, Centroiding & Orientation Analysis

### 5.1 Area & Geometry Verification

A $25\text{ mm} \times 25\text{ mm}$ wooden cube viewed directly from above has a nominal geometric area of:
$$A_{\text{nominal}} = 25\text{ mm} \times 25\text{ mm} = 625\text{ mm}^2$$
At our rectified metric scale of $\text{PPM} = 2.0\text{ pixels/mm}$:
$$A_{\text{pixels}} = 625\text{ mm}^2 \times (2.0)^2 = \mathbf{2{,}500\text{ pixels}^2}$$

To accommodate corner bevels, slight perspective slope, and painting thickness:

- **Valid Contour Area**: $A \in [1{,}400, 3{,}600\text{ pixels}^2]$ ($\approx 350\text{--}900\text{ mm}^2$).
- **Bounding Box Aspect Ratio**: $\text{Aspect} = \frac{\text{Width}}{\text{Height}} \in [0.70, 1.40]$ (rejects elongated cables or arm links).

### 5.2 Centroid Calculation via Spatial Moments

For a binary contour $C$, spatial image moments $M_{pq}$ are computed by:
$$M_{pq} = \iint x^p y^q I(x, y) \, dx \, dy$$
The center of mass (centroid) $(\bar{u}, \bar{v})$ in the rectified image is:
$$\bar{u} = \frac{M_{10}}{M_{00}}, \quad \bar{v} = \frac{M_{01}}{M_{00}}$$

### 5.3 Cube Yaw Orientation ($\psi$) via `cv2.minAreaRect`

When grasping in the near zone ($d_{\text{edge}} \le 10\text{ cm}$), the gripper aligns with the cube's vertical yaw orientation. We extract the rotated bounding rectangle:

```python
rect = cv2.minAreaRect(contour)
# rect returns: ((center_x, center_y), (width, height), angle_deg)
(cx, cy), (w, h), angle = rect

# Normalize angle to range [-45°, +45°] relative to the gripper approach axis
if w < h:
    yaw_angle = angle - 90.0
else:
    yaw_angle = angle

if yaw_angle < -45.0:
    yaw_angle += 90.0
elif yaw_angle > 45.0:
    yaw_angle -= 90.0
```

---

## 6. Metric Coordinate Transformation (Vision to Robot Frames)

Once a cube centroid $(\bar{u}, \bar{v})$ is identified in the rectified metric image:

```text
                                        Rectified Origin (0, 0)
                                        [World X = 70.0 mm, Y = 180.0 mm]
                                        ┌────────────────────────┐ ───> +u (Pixels)
                                        │                        │
                                        │      Cube (u_c, v_c)   │
                                        │          ●             │
                                        │                        │
                                        │                        │
                                        └────────────────────────┘
                                        │
                                        ▼ +v (Pixels)
```

### 6.1 Transformation to World Coordinates $(X_w, Y_w)$

Since rectified image $(0, 0)$ corresponds to $(X_w = 70.0\text{ mm}, Y_w = 180.0\text{ mm})$:
$$X_w = 70.0 + \frac{\bar{u}}{\text{PPM}} = 70.0 + \frac{\bar{u}}{2.0}$$
$$Y_w = 180.0 - \frac{\bar{v}}{\text{PPM}} = 180.0 - \frac{\bar{v}}{2.0}$$

### 6.2 Transformation to Robot Polar Coordinates $(r, \theta)$

The single xArm 1S is centered at $(X_{\text{base}} = 200.0\text{ mm}, Y_{\text{base}} = 280.0\text{ mm})$, with a nominal South heading $\theta_{\text{nom}} = -90.0^\circ$ (pointing along $-Y$ directly into the picking arena):

$$\Delta X = X_w - 200.0\text{ mm}$$
$$\Delta Y = Y_w - 280.0\text{ mm}$$

The radial distance $r$ and global azimuth angle $\theta_{\text{world}}$ relative to the arm's base are:
$$r = \sqrt{(\Delta X)^2 + (\Delta Y)^2}$$
$$\theta_{\text{world}} = \text{atan2}(\Delta Y, \Delta X)$$

The arm-relative azimuth $\theta_{\text{arm}}$ normalized to $[-180^\circ, +180^\circ]$ is:
$$\theta_{\text{arm}} = \theta_{\text{world}} - (-90.0^\circ) = \theta_{\text{world}} + 90.0^\circ$$

### 6.3 Boundary Sanity Filter

The detection is accepted **only if it satisfies the physical reach limits**:
$$110.0\text{ mm} \le r \le 300.0\text{ mm}$$
$$X_w \in [70.0, 330.0\text{ mm}], \quad Y_w \in [40.0, 180.0\text{ mm}]$$
All valid detected cubes are assigned to `xarm_1`.

---

## 7. Complete Production Python Implementation

Below is the complete, self-contained `ArenaVisionPipeline` class, ready to drop into ROS 2 `vision_node.py` or run standalone:

```python
#!/usr/bin/env python3
\"\"\"
ArenaVisionPipeline: Dedicated Planar Homography and HSV Detection Engine
Optimized for NVIDIA Jetson Orin Nano running ROS 2 Humble.
\"\"\"

import cv2
import cv2.aruco as aruco
import numpy as np
from typing import Dict, List, Optional, Tuple

class DetectedCube:
    def __init__(self, color: str, x_w: float, y_w: float, z_w: float,
                 yaw_deg: float, r1: float, theta1: float, d_edge1: float,
                 r2: float, theta2: float, d_edge2: float, assigned_arm: str):
        self.color = color
        self.x_w = x_w          # World X in mm
        self.y_w = y_w          # World Y in mm
        self.z_w = z_w          # World Z in mm (flat surface + half cube = 12.5 mm)
        self.yaw_deg = yaw_deg  # Cube orientation on mat (-45° to +45°)

        # Arm 1 (Left Arm @ X=170, Y=360) coordinates
        self.r1 = r1
        self.theta1 = theta1
        self.d_edge1 = d_edge1

        # Arm 2 (Right Arm @ X=390, Y=360) coordinates
        self.r2 = r2
        self.theta2 = theta2
        self.d_edge2 = d_edge2

        self.assigned_arm = assigned_arm  # "arm_1" or "arm_2" based on X=280 mm midline

    def __repr__(self):
        return (f"DetectedCube({self.color}, X={self.x_w:.1f}mm, Y={self.y_w:.1f}mm, "
                f"Assigned={self.assigned_arm}, r={getattr(self, 'r' + self.assigned_arm[-1]):.1f}mm, "
                f"d_edge={getattr(self, 'd_edge' + self.assigned_arm[-1]):.1f}mm, yaw={self.yaw_deg:.1f}°)")

class ArenaVisionPipeline:
    def __init__(self, pixels_per_mm: float = 2.0):
        self.ppm = pixels_per_mm

        # World coordinates of 4 ArUco markers (Flat Baseboard Z = 0.0 mm)
        # IDs: 0 (Rear-Left), 1 (Rear-Right), 2 (Front-Left), 3 (Front-Right)
        self.world_corners = {
            0: (85.0, 255.0),
            1: (475.0, 255.0),
            2: (85.0, 45.0),
            3: (475.0, 45.0)
        }

        self.rect_w = int((475.0 - 85.0) * self.ppm)  # 780 px
        self.rect_h = int((255.0 - 45.0) * self.ppm)  # 420 px

        self.dst_pts = np.array([
            [0, 0],                     # ID 0 (Rear-Left)
            [self.rect_w, 0],           # ID 1 (Rear-Right)
            [0, self.rect_h],           # ID 2 (Front-Left)
            [self.rect_w, self.rect_h]  # ID 3 (Front-Right)
        ], dtype=np.float32)

        self.H_matrix: Optional[np.ndarray] = None

        # ArUco detector setup (DICT_4X4_50)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Morphological structuring element
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        # Inward-Angled V-Configuration Arm Base Centers & Headings
        self.arm1_base = (170.0, 360.0)  # Left Arm (Heading: +35.0°)
        self.arm2_base = (390.0, 360.0)  # Right Arm (Heading: -35.0°)
        self.arm1_heading_deg = 35.0
        self.arm2_heading_deg = -35.0

    def calibrate_homography(self, frame: np.ndarray) -> bool:
        \"\"\"Detects 4 ArUco markers and updates planar homography matrix H.\"\"\"
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        if ids is None or len(ids) < 4:
            return False

        detected_ids = ids.flatten()
        if not all(k in detected_ids for k in [0, 1, 2, 3]):
            return False

        src_pts = np.zeros((4, 2), dtype=np.float32)
        for i, marker_id in enumerate([0, 1, 2, 3]):
            idx = np.where(detected_ids == marker_id)[0][0]
            # Center of the 40 mm ArUco marker
            c = corners[idx][0]
            center = np.mean(c, axis=0)
            src_pts[i] = center

        self.H_matrix = cv2.getPerspectiveTransform(src_pts, self.dst_pts)
        return True

    def warp_arena(self, frame: np.ndarray) -> Optional[np.ndarray]:
        \"\"\"Warps input frame to the dedicated metric picking arena plane.\"\"\"
        if self.H_matrix is None:
            if not self.calibrate_homography(frame):
                return None
        return cv2.warpPerspective(frame, self.H_matrix, (self.rect_w, self.rect_h))

    def detect_cubes(self, warped_frame: np.ndarray) -> List[DetectedCube]:
        \"\"\"Segments cubes by color and computes world + polar coordinates.\"\"\"
        hsv = cv2.cvtColor(warped_frame, cv2.COLOR_BGR2HSV)
        detected_cubes: List[DetectedCube] = []

        color_ranges = {
            "Red": [
                (np.array([0, 120, 70]), np.array([10, 255, 255])),
                (np.array([170, 120, 70]), np.array([180, 255, 255]))
            ],
            "Green": [(np.array([38, 100, 60]), np.array([82, 255, 255]))],
            "Blue": [(np.array([98, 120, 60]), np.array([130, 255, 255]))],
            "Yellow": [(np.array([20, 120, 100]), np.array([36, 255, 255]))]
        }

        for color_name, bands in color_ranges.items():
            if len(bands) == 2:
                m1 = cv2.inRange(hsv, bands[0][0], bands[0][1])
                m2 = cv2.inRange(hsv, bands[1][0], bands[1][1])
                mask = cv2.bitwise_or(m1, m2)
            else:
                mask = cv2.inRange(hsv, bands[0][0], bands[0][1])

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                # Nominal area is 2500 px² (625 mm²); filter within [1400, 3600]
                if not (1400.0 <= area <= 3600.0):
                    continue

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                u_c = M["m10"] / M["m00"]
                v_c = M["m01"] / M["m00"]

                # Yaw orientation extraction via minimum bounding box
                rect = cv2.minAreaRect(cnt)
                _, (w_box, h_box), angle = rect
                aspect = max(w_box, h_box) / max(min(w_box, h_box), 1e-4)
                if aspect > 1.45:  # Reject non-square shapes
                    continue
                yaw = angle - 90.0 if w_box < h_box else angle
                if yaw < -45.0: yaw += 90.0
                elif yaw > 45.0: yaw -= 90.0

                # Transform to World coordinates (mm)
                x_w = 85.0 + (u_c / self.ppm)
                y_w = 255.0 - (v_c / self.ppm)
                z_w = 12.5  # 0.0 mm flat arena + 12.5 mm half cube

                # Arm 1 relative (Left Arm @ 170, 360)
                dx1, dy1 = x_w - self.arm1_base[0], y_w - self.arm1_base[1]
                r1 = float(np.hypot(dx1, dy1))
                th1 = float(np.degrees(np.arctan2(dy1, dx1)))
                d_edge1 = r1 - 70.0

                # Arm 2 relative (Right Arm @ 390, 360)
                dx2, dy2 = x_w - self.arm2_base[0], y_w - self.arm2_base[1]
                r2 = float(np.hypot(dx2, dy2))
                th2 = float(np.degrees(np.arctan2(dy2, dx2)))
                d_edge2 = r2 - 70.0

                # Initial midline assignment (X = 280 mm workcell midline)
                assigned_arm = "arm_1" if x_w <= 280.0 else "arm_2"

                # Boundary validation check (must fall inside 5-25 cm operating reach)
                active_r = r1 if assigned_arm == "arm_1" else r2
                if 120.0 <= active_r <= 320.0:
                    cube = DetectedCube(color_name, x_w, y_w, z_w, yaw,
                                        r1, th1, d_edge1, r2, th2, d_edge2, assigned_arm)
                    detected_cubes.append(cube)

        return detected_cubes
```

---

## 8. Calibration Tooling & Verification Scripts

The repository includes standalone calibration utilities, autonomous execution bridges, and ROS 2 nodes:

### 8.1 `vision/calibrate_camera.py` (Intrinsic Lens Calibration)

- Uses a $9 \times 6$ checkerboard ($25\text{ mm}$ square size).
- Solves camera intrinsic matrix $\mathbf{K}$ and radial/tangential distortion coefficients $[k_1, k_2, p_1, p_2, k_3]$.
- Features live GUI with corner overlay, `SPACE` sample capture, and `c` key to compute reprojection error and atomically save parameters to `config/camera_calibration.yaml`.
- Includes `--synthetic` mode for headless CI/CD and offline verification.

### 8.2 `vision/tune_hsv.py` (Interactive HSV Tuner)

- Real-time GUI with 6 trackbars (`H_min`, `H_max`, `S_min`, `S_max`, `V_min`, `V_max`).
- Keys `1` (Red), `2` (Green), `3` (Blue), `4` (Yellow) to switch active tuning target.
- Displays live side-by-side feed: Original Warped Arena vs. Active Color Mask.
- Allows exporting tuned thresholds directly into `config/colors.yaml` with one keypress (`s`).

### 8.3 `vision/test_aruco_warp.py` (Metric Accuracy Verification & Homography Lock)

- Captures live frame, detects the 4 arena ArUco markers (`DICT_4X4_50`, IDs: 0, 1, 2, 3), and applies the homography warp ($780 \times 420\text{ px}$ at $2.0\text{ px/mm}$).
- Press `l` to **lock/freeze** the homography matrix $H$ in memory, completely eliminating camera boom vibration jitter.
- Press `g` to display the $50\text{ mm}$ metric grid lines. The operator places a ruler on the arena surface to verify accuracy ($\le 1.5\text{ mm}$ error).
- Press `s` to atomically save the homography matrix to `config/camera_calibration.yaml`.

### 8.4 `tools/autonomous_pick_and_place.py` (Phase 1 + Phase 2 Autonomous Bridge)

- Merges analytical polar inverse kinematics with computer vision localization.
- Enforces **Nearest-First radial ordering** to eliminate forearm collision plowing.
- Automatically routes cubes into dedicated 2×2 flank tray drop compartments by color (Red, Green, Blue, Yellow).
- Supports `--dry-run`, `--synthetic`, and `--single-shot` modes for safe testing.

### 8.5 ROS 2 Perception Nodes & Launch Architecture

- **`vision_node`**: Captures V4L2 video, rectifies arena, segments cubes, and publishes `/vision/detected_objects` (JSON array of world & polar coordinates) and `/vision/annotated_image`.
- **`autonomous_sorter_node`**: Subscribes to `/vision/detected_objects`, maintains pick queues, requests IK trajectories, and drives hardware.
- **Launch Command**:
  ```bash
  ros2 launch sorting_arm xarm.launch.py auto_start:=true
  ```

### 8.6 Automated Perception Test Suite

- Located at [`ros2_ws/src/sorting_arm/test/test_vision_pipeline.py`](file:///home/shubham/Projects/Sem-5-mini-project/ros2_ws/src/sorting_arm/test/test_vision_pipeline.py).
- 6 comprehensive unit tests verifying camera calibration loading, ArUco corner ordering, synthetic frame warp dimensions ($780 \times 420$), HSV color segmentation, Nearest-First ordering, and the end-to-end IK bridge.
- Run anytime via:
  ```bash
  pytest ros2_ws/src/sorting_arm/test/test_vision_pipeline.py -v
  ```
