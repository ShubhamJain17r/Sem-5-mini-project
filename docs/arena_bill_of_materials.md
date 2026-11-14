# 📋 Dual xArm 1s Workcell: Simplified Bill of Materials (BOM) & Workshop Guide

This document provides the definitive, practical hardware checklist, drill bit sizes, cut lengths, fasteners, and procurement guide for fabricating the **Dual xArm 1s Color-Sorting Workcell**.

---

## 🏗️ 1. Master Hardware Checklist

### Category A: Baseboard & Active Arena
| Item | Component Description | Sizing & Specifications | Qty | Placement / Role | Notes / Purchasing |
| :---: | :--- | :--- | :---: | :--- | :--- |
| **A1** | **Main Baseboard** | **$560\text{ mm} \times 520\text{ mm} \times 18\text{ mm}$** MDF or Plywood | 1 pc | Workcell foundation ($Z = 0.0\text{ mm}$) | Pre-cut at local timber store. Zero sawing needed. Mass (~4 kg) absorbs vibrations. |
| **A2** | **Foam Arena Border** | **$8\text{--}10\text{ mm}$ Height $\times 10\text{--}12\text{ mm}$ Width** Self-Adhesive Foam Tape | 1 roll ($2\text{ m}$) | Surrounds $400\times 220\text{ mm}$ picking arena ($X \in [80, 480], Y \in [40, 260]$) | Soft bumper prevents blocks from escaping and protects gripper claw fingers. |
| **A3** | **Anti-Skid Mat** *(Optional)*| **$400\text{ mm} \times 220\text{ mm} \times 1\text{ mm}$** Matte Silicone Sheet | 1 pc | Inside arena boundary | High friction ($\mu \approx 0.8$) prevents cubes from skidding during grasping. |
| **A4** | **Baseboard Rubber Feet** | $\varnothing 20\text{ mm} \times 5\text{ mm}$ Self-Adhesive Rubber Bumpers | 4 pcs | Underside 4 corners of baseboard | Stops board slipping on desk and prevents table scratches. |

---

### Category B: Scrap Aluminum Camera Stand & Lighting
| Item | Component Description | Sizing & Specifications | Qty | Placement / Role | Notes / Workshop Tip |
| :---: | :--- | :--- | :---: | :--- | :--- |
| **B1** | **Scrap Aluminum Stand** | **Scrap Aluminum Door Profile** (Square/Rectangular tube):<br>- **Vertical Piece: $700\text{ mm}$ (70 cm)**<br>- **Horizontal Arm: $400\text{ mm}$ (40 cm)** | 1.1 m total | Welded at $90^\circ$. Upright anchored at rear center $(X = 280, Y = 520\text{ mm})$ | Cantilever reaches directly over arena center $(X = 280, Y = 150\text{ mm})$ at $H \approx 650\text{ mm}$. |
| **B2** | **Heavy-Duty Corner Brackets** | 2-hole or 4-hole 90° angle bracket / gusset | 2 pcs | Anchors vertical aluminum post to rear MDF board edge | Standard hardware store L-brackets or aluminum angle offcuts. |
| **B3** | **10" Dimmable LED Ring Light** | **Invicto 10" USB LED Ring Light** with integrated ball head & central phone clamp | 1 pc | Screws directly onto 1/4"-20 tip bolt on horizontal arm | Ball head tilts ring flat facing downward; phone clamp holds camera nadir over arena. |

---

### Category C: Fasteners & Drill Bits Guide

| Item | Fastener / Hardware | Exact Dimensions | Qty | Used For | Required Drill Bit Size |
| :---: | :--- | :--- | :---: | :--- | :---: |
| **C1** | **xArm Baseplate Wood Screws** | #8 (4.0 mm) $\times$ 16–20 mm Pan Head Wood Screws + M4 Flat Washers | 8 pcs | Bolting 2× xArm 1s metal baseplates to MDF board | **3.0 mm (1/8")** *(Depth: 12–14 mm pilot)* |
| *Alt* | *xArm Base Through-Bolts* | M4 $\times$ 25–30 mm Machine Bolts + M4 Washers + M4 Hex Nuts | 8 pcs | Heavy-duty through-bolting through 18 mm board | **4.0 mm or 4.5 mm** *(Through-hole)* |
| **C2** | **Stand Base Bracket Screws** | #8 (4.0 mm) $\times$ 16 mm Wood Screws OR M5 bolts | 4 pcs | Anchoring bottom corner brackets into MDF board | **3.0 mm** (pilot) / **5.0 mm** (M5 bolt) |
| **C3** | **Stand Bracket-to-Aluminum** | M5 $\times$ 10 mm Machine Screws OR 4.0 mm Blind Pop-Rivets | 4 pcs | Fastening bracket into the aluminum vertical post | **4.2 mm** (rivet) / **5.0 mm** (M5 screw) |
| **C4** | **Camera Tip Mounting Bolt** | **1/4"-20 UNC Hex Bolt (1/2" or 3/4" length)** + 1/4" Nut + Washer | 1 set | Bolted through horizontal tube tip for ring light ball head | **6.5 mm (1/4")** *(Through-hole in tube)* |
| **C5** | **Sorting Tray Attachment** | Double-sided foam mounting tape (or 1 small 3.5mm wood screw) | 2 pcs | Securing 2× 2x2 sorting trays to flanks | **No drilling needed** (or 2.0 mm pilot) |

> [!TIP]
> **Tool Summary**: You only need **three drill bits** for the entire build:
> 1. **3.0 mm (1/8")**: Pilot holes into MDF for wood screws.
> 2. **4.5 mm / 5.0 mm**: Through-holes for M4/M5 machine bolts or corner brackets.
> 3. **6.5 mm (1/4")**: Through-hole at the aluminum cantilever tip for the 1/4"-20 ring light bolt.

---

### Category D: 3D-Printed Parts & Vision Targets

| Item | Component | File Path | Specs / Slicing | Quantity |
| :---: | :--- | :--- | :--- | :---: |
| **D1** | **2×2 Color Sorting Trays** | [`cad/models/sorting_tray_2x2.step`](../cad/models/sorting_tray_2x2.step)<br>[`cad/models/sorting_tray_2x2.stl`](../cad/models/sorting_tray_2x2.stl) | PLA+ or PETG, 0.20 mm layer height, 3 walls, 15% infill. **100% support-free**. | 2 units (~84 g total) |
| **D2** | **Planar ArUco Markers** | [`docs/assets/aruco_markers/`](assets/aruco_markers/) | 4× `DICT_4X4_50` printed at $30\times 30\text{ mm}$ on matte vinyl sticker paper | 4 pcs (IDs 0, 1, 2, 3) |
| **D3** | **Sorting Craft Cubes** | 1-inch ($25.4\text{ mm}$) Unfinished Hardwood Blocks | Sprayed with ultra-matte chalked paint (4 each of Red, Green, Blue, Yellow) | 16–20 pcs (~15 g each) |
| **D4** | **Electronics Mounting** | Adhesive Nylon PCB Standoffs (M3) & stick-on cable clips | Mounts Jetson / RPi 5 and dual buck regulators on rear bay ($Y \in [440, 500]$) | 8 standoffs, 6 clips |

---

## 💰 2. Estimated Workshop Procurement Summary

| Subsystem | Components Included | Estimated Cost (INR / USD) |
| :--- | :--- | :---: |
| **Baseboard & Arena** | MDF board ($560\times 520\times 18\text{ mm}$), foam tape, rubber feet | ₹1,000 (~$12.00) |
| **Scrap Aluminum Stand** | Scrap door profile (1.1 m) + welding + 2 L-brackets | ₹300 (~$3.50) |
| **Ring Light & Phone Mount** | Invicto 10" USB LED Ring Light with Ball Head & Phone Clamp | ₹249 (~$3.00) |
| **Fasteners & Hardware** | Wood screws, M4/M5 bolts, washers, 1/4"-20 bolt & nut | ₹150 (~$1.80) |
| **3D Printing Filament** | ~85 g PLA+ (2× 2x2 sorting trays) | ₹150 (~$1.80) |
| **Sorting Cubes & Paint** | 20 wooden cubes + 4 matte spray cans | ₹400 (~$4.80) |
| **ArUco Stickers & Clips** | Matte sticker sheet, PCB standoffs, wire clips | ₹200 (~$2.40) |
| **TOTAL WORKCELL FABRICATION COST** | *(Excluding robot arms & SBC)* | **$\approx$ ₹2,449 (~$29.30)** |

