# Stepping Stone 7: The Ultra-Simplified Workcell Fabrication Handbook (Zero-Carpentry & Minimal 3D Printing)

This document is the official, practical build guide for the **Single xArm 1s Color-Sorting Robotic Workcell**.

Following an end-to-end design review, all complex carpentry (saws, routers, multi-layer wood laminating) and risky 70-hour 3D printing jobs have been eliminated. This simplified design can be built in **under 2 hours** using **ONE flat store-cut wooden board ($400\times 400\text{ mm}$)**, **one small 3D-printed sorting tray** (under 45 minutes print time, $< 42\text{ g}$ filament), and simple off-the-shelf components.

All coordinates, reach envelopes, and optical configurations remain 100% mathematically aligned with:
- [`docs/01_arena_and_layout.md`](01_arena_and_layout.md) (Workcell topology, coordinates, reach zones)
- [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) (Analytical IK, grasp angles, reach envelopes)
- [`docs/03_computer_vision_and_calibration.md`](03_computer_vision_and_calibration.md) (Planar homography, exposure lock, HSV color segmentation)

---

## 1. Visual Architecture Overview

### 1.1 Complete Workcell 3D CAD Prototype
![Realistic 3D Perspective Render of Single xArm Workcell](assets/images/single_arm_setup.png)

### 1.2 Layout Schematic

```text
  Y (Depth: 400 mm)
   ^
   │  ┌────────────────────────────────────────────────────────┐
   │  │ REAR OF WORKSPACE (Y = 400 mm)                         │
   │  │          [Clamp-On Overhead Webcam Stand Base]         │
   │  │                  (X = 200, Y = 390)                    │
   │  │                                                        │
   │  │          ┌──────────────────────────────────┐          │
   │  │          │    REAR ELECTRONICS & PSU BAY    │          │
   │  │          │ (RPi 5 / Jetson & 7.4V Regulator)│          │
   │  │          └────────────────┬─────────────────┘          │
   │  │                           │                            │
   │  │   ┌─────────────┐       xArm 1                         │
   │  │   │  2x2 Tray   │     (Base Z = 0)                     │
   │  │   │(R|G|B|Y Bins│    Heading: -90°                     │
   │  │   │  (50, 280)  │      (200, 280)                      │
   │  │   └─────────────┘          │                           │
   │  │                            ▼                           │
   │  │     [ArUco #0] ────────────┼──────────── [ArUco #1]    │
   │  │     (70, 180)              │             (330, 180)    │
   │  │                            │                           │
   │  │                   ACTIVE PICKING ARENA                 │
   │  │                         (Z = 0)                        │
   │  │                       260 x 140 mm                     │
   │  │                            │                           │
   │  │     [ArUco #2] ────────────┴──────────── [ArUco #3]    │
   │  │     (70, 40)   (Foam Tape Border Barrier)(330, 40)     │
   │  │                                                        │
   │  │ FRONT OF WORKSPACE (Operator Position @ Y = 0 mm)      │
   │  └────────────────────────────────────────────────────────┘
   └───────────────────────────────────────────────────────────────> X (Width: 400 mm)
```

### Elevation Levels (Z-Axis)
```text
 Elevation (Z)
   │
 650 mm ─── [Logitech Webcam on Overhead Stand / 10" Ring Light]
   │
   │        (Rigid welded scrap aluminum stand anchored to rear board edge)
   │
  65 mm ─── [xArm 1s Shoulder Axis (Motor 5) @ d1 = 65 mm]
  35 mm ─── [Top Rim of 2x2 Color Sorting Tray]
  25 mm ─── [Top of 25 mm Target Sorting Cubes]
  12.5 mm ── [Grasp Target Center of Cube]
  10 mm ─── [Top of Adhesive Foam Tape Retaining Curb]
   0 mm ─── [Flat Arena Surface & Wooden Baseboard Top Face (Z = 0)]
 -18 mm ─── [Table Surface / Baseboard Underside] (18 mm MDF Board)
```

---

## 2. Key Design Decisions & Wooden Platform Blueprint

Below is the technical dimensional blueprint for the single flat wooden baseboard ($400.0\text{ mm} \times 400.0\text{ mm} \times 18.0\text{ mm}$), showing pencil layout lines, arm position, arena boundaries, and drop tray location:

![Single xArm 1S Workcell Engineering Blueprint & Arena Layout](assets/images/arena_layout.png)

| Component | Design Implementation | Engineering Rationale |
| :--- | :--- | :--- |
| **Main Baseboard** | Single store-cut MDF board ($400\times 400\text{ mm}$, $18\text{ mm}$) | Zero cutting at home. Substantial mass (~2.3 kg) eliminates table slipping and absorbs servo vibrations. |
| **Arena Surface** | Completely flat on the board ($Z = 0$) | Eliminates 3D-printing massive puzzle tiles or jigsawing wood platforms. |
| **Arena Retaining Curb** | Self-adhesive foam weatherstripping tape ($8\text{--}10\text{ mm}$ high) | Peels and sticks in 2 minutes along $X \in [70, 330], Y \in [40, 180]$. Soft barrier stops cubes from rolling away and won't jam the claw. |
| **Arm Mounting** | Bolted flat to board ($Z = 0$) at $(200, 280\text{ mm})$, heading $-90.0^\circ$ (South) | Eliminates 3D-printed wedges. Flat mounting maximizes horizontal reach ($280\text{ mm}$) and rigidity. |
| **Color Sorting Tray**| Single 3D-printed $2\times 2$ quadrant bin ($80\times 80\times 35\text{ mm}$) on left flank | Dedicated 4-slot sorting (Red, Green, Blue, Yellow). Sits at $(50, 280\text{ mm})$, $r = 150\text{ mm}$ ($90^\circ$ clockwise swing from arena heading). |
| **Overhead Camera Stand**| Welded scrap aluminum door stand ($700\text{ mm} \times 400\text{ mm}$) | Rigid, zero wobble. 1/4"-20 tip bolt mounts Invicto 10" LED ring light + camera nadir over $(200, 110\text{ mm})$. |
| **ArUco Calibration** | 4 matte vinyl stickers applied flat on the board corners ($Z = 0$) | At $(70, 180), (330, 180), (70, 40), (330, 40)$. Coplanar with arena floor and cube bottoms—mathematically ideal for OpenCV planar homography. |
| **Electronics & Wiring**| SBC & single 7.4V buck regulator mounted on rear bay with adhesive standoffs | Accessible USB/HDMI ports, short flat wire routing with stick-on wire clips. Zero under-deck routing. |
| **Sorting Cubes** | 1-inch ($25.4\text{ mm}$) unfinished wooden craft blocks | Natural $15\text{ g}$ mass prevents claw skidding. Spray-painted with ultra-matte Red, Green, Blue, Yellow. |

---

## 3. Minimal 3D Printing: Only the 1× Color Tray

In this ultra-simplified single-arm design, your 3D printer only needs to print **ONE small $2\times 2$ sorting tray** (`cad/models/sorting_tray_2x2.stl`):

![3D CAD Render of 2x2 Quadrant Color Sorting Tray](assets/images/tray_3d_print_cad.png)

```text
             TOP VIEW: 2x2 QUADRANT COLOR SORTING TRAY
 
           ◄───────────────── 80 mm ─────────────────►
         ┌─────────────────────────┬─────────────────────────┐ ▲
         │                         │                         │ │
         │       SLOT 1: RED       │      SLOT 2: GREEN      │ │
         │       (36 x 36 mm)      │       (36 x 36 mm)      │ │
         │                         │                         │ 80 mm
         ├─────────────────────────┼─────────────────────────┤ │
         │                         │                         │ │
         │      SLOT 3: BLUE       │      SLOT 4: YELLOW     │ │
         │       (36 x 36 mm)      │       (36 x 36 mm)      │ │
         │                         │                         │ │
         └─────────────────────────┴─────────────────────────┘ ▼
                                   Height: 35 mm, Wall: 2.0 mm
```

### 3D Printing Specifications
* **Tray Dimensions**: $80.0\text{ mm (Width)} \times 80.0\text{ mm (Depth)} \times 35.0\text{ mm (Height)}$.
* **Compartment Size**: 4 equal quadrant pockets of $36.0\text{ mm} \times 36.0\text{ mm} \times 33.0\text{ mm}$ deep (holds 2 cubes stacked per slot).
* **Wall Thickness**: $2.0\text{ mm}$ with $3.0\text{ mm}$ bottom floor.
* **Internal Corner Fillets**: $R = 3.0\text{ mm}$ for easy cube retrieval.
* **Slicer Profile**: $0.20\text{ mm}$ layer height, 3 walls, 15% infill.
* **Print Time**: $\approx \mathbf{45\text{ minutes}}$ (only 1 tray needed!).
* **Filament Used**: **$< 42\text{ g}$ total** (minimal PLA+ consumption).

---

## 4. Step-by-Step Build & Setup Guide (Under 2 Hours)

```mermaid
graph TD
    A["1. Place Pre-Cut Board on Desk (400x400 mm)"] --> B["2. Mark Coordinates with Pencil & Ruler"]
    B --> C["3. Drill & Fasten xArm Baseplate (200, 280)"]
    C --> D["4. Stick Foam Tape Perimeter Border (260x140 mm)"]
    D --> E["5. Press-Fit 4x ArUco Stickers at Arena Corners"]
    E --> F["6. Place 2x2 Color Sorting Tray on Left Flank (50, 280)"]
    F --> G["7. Mount Welded Scrap Aluminum Stand & 10-Inch Ring Light"]
    G --> H["8. Mount SBC & Electronics in Rear Bay (Y = 330..390 mm)"]
```

### Step 1: Baseboard Preparation (Zero Tools)
1. Acquire a single flat rectangular panel of **$18\text{ mm}$ (or $12\text{ mm}$) MDF or shelving board** pre-cut at the hardware store to **$400\text{ mm} \times 400\text{ mm}$**.
2. Place it on your desk or workstation. You do zero sawing or cutting.

### Step 2: Pencil Coordinate Marking
1. With a pencil and ruler, draw a centerline along $Y$ at $X = 200\text{ mm}$.
2. Mark the single arm base center at **$(X = 200\text{ mm}, Y = 280\text{ mm})$**.
3. Outline the rectangular picking arena:
   - Left edge: $X = 70\text{ mm}$, Right edge: $X = 330\text{ mm}$ (Width: $260\text{ mm}$, centered at $X = 200\text{ mm}$)
   - Front edge: $Y = 40\text{ mm}$, Rear edge: $Y = 180\text{ mm}$ (Depth: $140\text{ mm}$)
   - *Clearance*: The arena stops at $Y = 180\text{ mm}$, providing a safe $23.5\text{ mm}$ buffer before reaching the robot metal baseplate edge ($Y = 203.5\text{ mm}$).

### Step 3: Drill & Fasten the Robotic Arm
1. Align the metal baseplate of xArm 1 **square with the board edges** ($0^\circ$ physical rotation, base center at $(200, 280\text{ mm})$):
   - Mounting holes ($127\times 143\text{ mm}$ pattern): $X \in \{136.5, 263.5\}$, $Y \in \{208.5, 351.5\}$.
2. **Drilling the Mounting Holes**:
   - **Wood Screw Method (Recommended)**: Use a **3.0 mm (1/8") drill bit** to drill 4 pilot holes to a depth of **12–14 mm** (wrap painter's tape around the bit at 13 mm as a depth stop so you don't punch through the 18 mm board). Fasten using four #8 (4.0 mm) $\times$ 16–20 mm wood screws with M4 flat washers.
   - **Through-Bolt Method**: Use a **4.0 mm or 4.5 mm drill bit** to drill 4 holes completely through the 18 mm board. Fasten using M4 $\times$ 25–30 mm machine bolts with M4 washers and hex nuts on the underside.
3. Motor 6 (turntable) nominal arena center heading is $-90.0^\circ$ (South, straight forward into the picking arena).
4. The arm is now rigidly grounded at $Z = 0$ with zero flex, zero wobble, and clear lateral clearance to the flank tray ($+10.0\text{ mm}$).

### Step 4: Apply the Foam Tape Arena Border
1. Take a roll of standard **self-adhesive foam weatherstripping tape ($8\text{--}10\text{ mm}$ tall $\times 10\text{--}12\text{ mm}$ wide)**.
2. Peel the backing and stick it down along the four penciled arena boundary lines ($X \in [70, 330], Y \in [40, 180]$).
3. This creates a soft, clean retaining bumper (outer footprint $X \in [60, 340], Y \in [30, 190]$) that stops wooden cubes from rolling outside the camera's view.
4. *(Optional)* Lay a $1.0\text{ mm}$ high-friction matte silicone mat ($\mu \approx 0.8$) inside the rectangular arena boundary to ensure cubes don't slide during gripper contact.

### Step 5: Apply the 4 ArUco Calibration Stickers
1. Print the 4 ArUco markers (`DICT_4X4_50`, IDs 0, 1, 2, 3) at $30\text{ mm} \times 30\text{ mm}$ on matte vinyl sticker paper.
2. Peel and stick them directly flat onto the wooden board at the four corners of the arena:
   - **ArUco #0 (Rear-Left)**: $(X = 70.0\text{ mm}, Y = 180.0\text{ mm}, Z = 0.0\text{ mm})$
   - **ArUco #1 (Rear-Right)**: $(X = 330.0\text{ mm}, Y = 180.0\text{ mm}, Z = 0.0\text{ mm})$
   - **ArUco #2 (Front-Left)**: $(X = 70.0\text{ mm}, Y = 40.0\text{ mm}, Z = 0.0\text{ mm})$
   - **ArUco #3 (Front-Right)**: $(X = 330.0\text{ mm}, Y = 40.0\text{ mm}, Z = 0.0\text{ mm})$
3. Because the stickers are coplanar with the table surface ($Z = 0$), OpenCV calculates a mathematically perfect planar homography warp ($H$) with zero perspective skew ($520\times 280\text{ px}$ grid at $2.0\text{ px/mm}$)!

### Step 6: Position the 2×2 Color Sorting Tray
1. Place the single 3D-printed Left Tray at $(X = 50.0\text{ mm}, Y = 280.0\text{ mm})$ on the left flank of the arm ($r = 150.0\text{ mm}$, $90^\circ$ clockwise swing from arena heading).
2. Fasten with double-sided foam mounting tape (zero drilling required).
3. Color Slot Mapping:
   - Compartment 0 (Rear-Outer): **Red**
   - Compartment 1 (Rear-Inner): **Green**
   - Compartment 2 (Front-Outer): **Blue**
   - Compartment 3 (Front-Inner): **Yellow**

### Step 7: Fabricate & Mount Scrap Aluminum Stand & 10" Ring Light
1. **Cut the Aluminum Profiles**: From scrap aluminum door tube/pipe, cut:
   - **Vertical Upright**: **$700\text{ mm}$** (70 cm)
   - **Horizontal Cantilever**: **$400\text{ mm}$** (40 cm)
2. **Weld Joint**: Weld the two sections together at a rigid **$90^\circ$ angle**.
3. **Drill Camera Tip Hole**: Drill a **$6.5\text{ mm} (1/4")$ hole** vertically through the horizontal cantilever pipe $15\text{ mm}$ from the front tip. Pass a standard **1/4"-20 UNC bolt** ($1/2"$ or $3/4"$ length) pointing downward and secure with a hex nut and washer.
4. **Anchor Upright to Baseboard**: Position the vertical post upright at the rear center of the board $(X = 200\text{ mm}, Y = 390\text{--}400\text{ mm})$. Fasten with 2× heavy-duty corner brackets:
   - Into board: Drill 3.0 mm pilot holes for wood screws (or 5.0 mm through-holes for M5 bolts).
   - Into aluminum: Drill 4.2 mm holes for pop-rivets or 5.0 mm holes for M5 screws.
5. **Mount Ring Light & Camera**:
   - Screw the **Invicto 10" Ring Light's mini ball head** onto the 1/4"-20 tip bolt.
   - Adjust the ball head so the ring light sits horizontally, illuminating directly downwards onto the arena.
   - Clamp the camera / smartphone into the ring light's central flexible holder, centered over $(X = 200\text{ mm}, Y = 110\text{ mm})$ at an elevation of **$H \approx 650\text{ mm}$** pointing straight down (nadir).

### Step 8: Mount Electronics & Route Cables
1. Stick 4× adhesive nylon PCB standoffs on the rear section of the board ($Y \approx 330\text{--}380\text{ mm}$).
2. Mount the Raspberry Pi 5 / NVIDIA Jetson Orin Nano onto the standoffs.
3. Mount the single 7.4V / 5A buck converter module beside the SBC.
4. Use adhesive wire clips to route the short USB serial cable ($< 18\text{ cm}$) and 7.4V servo power leads neatly to the arm base.

---

## 5. Sorting Cubes Preparation

* **Sourcing**: Buy a 20-pack of **1-inch ($25.4\text{ mm}$) unfinished hardwood craft cubes** from Amazon or a craft store (~₹400 / $4).
* **Mass**: They naturally weigh **$14\text{--}16\text{ g}$**, perfectly matching the physics model in [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md).
* **Finishing**:
  1. Sand faces lightly with 220-grit sandpaper.
  2. Spray with **Matte Gray Primer**.
  3. Spray 4 high-contrast matte topcoats: **Red, Green, Blue, Yellow** (e.g., Rust-Oleum Chalked or Montana Gold Matte).
  4. Never use gloss or semi-gloss paint.

---

## 6. Complete Hardware & Cost Breakdown

See [**`docs/arena_bill_of_materials.md`**](arena_bill_of_materials.md) for the master 1-page procurement spreadsheet.

| Item Description | Source | Estimated Cost |
| :--- | :---: | :---: |
| 1× $18\text{ mm}$ (or $12\text{ mm}$) MDF Board ($400\times 400\text{ mm}$, store cut) | Hardware Store / Timber Merchant | ₹600 (~$7.20) |
| 1× Scrap Aluminum Door Stand (cut $700+400\text{ mm}$, welded 90°) + 2 Brackets | Scrap Aluminum Fabricator | ₹300 (~$3.50) |
| 1× Invicto 10" LED Ring Light with Ball Head & Phone Holder | Amazon | ₹249 (~$3.00) |
| 1× Roll of Self-Adhesive Foam Weatherstripping Tape ($8\text{--}10\text{ mm}$) | Hardware Store / Amazon | ₹150 (~$1.80) |
| 1× Pack of 1-inch Wooden Craft Cubes (20 pcs) + 4 Spray Paint Cans | Craft Store / Amazon | ₹400 (~$4.80) |
| 1× Sheet of Matte Vinyl Sticker Paper (for ArUco) | Office Supplies / Amazon | ₹100 (~$1.20) |
| 1× Small spool of PLA+ filament (need only ~42 g for 1 tray) | 3D Printer Stock | ₹75 (~$0.90) |
| 1× Set of Fasteners (wood screws, M4 washers, 1/4"-20 bolt/nut, PCB clips) | Hardware Store | ₹80 (~$1.00) |
| **TOTAL COST OF WORKCELL** | — | **$\approx$ ₹1,954 (~$23.40)** |

---

## 7. Pre-Flight Verification Checklist

| Subsystem | Milestone / Verification Check | Status |
| :--- | :--- | :---: |
| **Baseboard** | Single flat board ($400\times 400\text{ mm}$) sits dead-flat on desk with zero rocking | [ ] |
| **Arm** | Single xArm screwed flat at $(200, 280\text{ mm})$, nominal heading $-90.0^\circ$ (South) | [ ] |
| **Arena** | Foam tape border pressed in a clean $260\times 140\text{ mm}$ rectangle ($X \in [70, 330], Y \in [40, 180]$) | [ ] |
| **ArUco** | 4 matte stickers applied flat at $(70, 180), (330, 180), (70, 40), (330, 40)$ ($Z = 0$ coplanar with floor) | [ ] |
| **Camera** | Overhead boom arm clamped to rear edge; camera centered at $(200, 110)$ @ $H = 650\text{ mm}$ | [ ] |
| **Tray** | 1× 2x2 grid sorting tray placed on left flank at $(50, 280\text{ mm})$ | [ ] |
| **Electronics** | SBC and 7.4V buck converter secured with adhesive standoffs; USB run $< 18\text{ cm}$ | [ ] |
| **Cubes** | 1-inch craft cubes painted in ultra-matte Red, Green, Blue, Yellow (~15 g mass) | [ ] |
| **Kinematics** | Test swing executed: Arm swings $90^\circ$ clockwise to flank tray at $r = 150\text{ mm}$ with zero stall | [ ] |
