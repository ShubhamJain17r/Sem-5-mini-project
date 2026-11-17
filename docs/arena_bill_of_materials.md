# 📋 Single xArm 1S Red-Object Picking Workcell: Bill of Materials (BOM) & Workshop Guide

This document provides the practical hardware checklist, drill bit sizes, cut lengths, fasteners, and procurement guide for fabricating the **Single xArm 1S Red-Object Picking Workcell**.

---

## 🏗️ 1. Master Hardware Checklist

### Category A: Baseboard & Active Arena
| Item | Component Description | Sizing & Specifications | Qty | Placement / Role | Notes / Purchasing |
| :---: | :--- | :--- | :---: | :--- | :--- |
| **A1** | **Main Platform Board** | **$350\text{ mm} \times 600\text{ mm} \times 18\text{ mm}$** MDF or Plywood | 1 pc | Workcell foundation ($Z = 0.0\text{ mm}$) | Hand-drawn $7\times 12$ grid ($5\text{ cm} \times 5\text{ cm}$ cells) drawn directly on platform. |
| **A2** | **Foam Arena Border** | **$8\text{--}10\text{ mm}$ Height $\times 10\text{--}12\text{ mm}$ Width** Self-Adhesive Foam Tape | 1 roll ($2.0\text{ m}$) | Surrounds active picking arena boundary | Soft bumper prevents blocks from rolling away and cushions claw fingers. |
| **A3** | **Rear Flank Container Box** | **$100\text{ mm} \times 100\text{ mm} \times 35\text{ mm}$** Physical Container | 1 pc | Rows 0..1, Cols 5..6 at $(X=300, Y=550\text{ mm})$ | Physical box for red objects; zero 3D printing needed. |
| **A4** | **Baseboard Rubber Feet** | $\varnothing 20\text{ mm} \times 5\text{ mm}$ Self-Adhesive Rubber Bumpers | 4 pcs | Underside 4 corners of baseboard | Stops board slipping on desk and prevents table scratches. |

---

### Category B: Scrap Aluminum Camera Stand & Lighting
| Item | Component Description | Sizing & Specifications | Qty | Placement / Role | Notes / Workshop Tip |
| :---: | :--- | :--- | :---: | :--- | :--- |
| **B1** | **Scrap Aluminum Stand** | **Scrap Aluminum Door Profile** (Square/Rectangular tube):<br>- **Vertical Piece: $700\text{ mm}$ (70 cm)**<br>- **Horizontal Arm: $400\text{ mm}$ (40 cm)** | 1.1 m total | Welded at $90^\circ$. Upright anchored at rear edge | Cantilever reaches overhead nadir over $(X = 175, Y = 250\text{ mm})$ at $H \approx 620\text{ mm}$. |
| **B2** | **Heavy-Duty Corner Brackets** | 2-hole or 4-hole 90° angle bracket / gusset | 2 pcs | Anchors vertical aluminum post to rear MDF board edge | Standard hardware store L-brackets or aluminum angle offcuts. |
| **B3** | **10" Dimmable LED Ring Light** | **Invicto 10" USB LED Ring Light** with integrated ball head & central phone clamp | 1 pc | Screws directly onto 1/4"-20 tip bolt on horizontal arm | Ball head tilts ring flat facing downward; holds camera nadir over arena. |

---

### Category C: Fasteners & Drill Bits Guide

| Item | Fastener / Hardware | Exact Dimensions | Qty | Used For | Required Drill Bit Size |
| :---: | :--- | :--- | :---: | :--- | :---: |
| **C1** | **xArm Baseplate Wood Screws** | #8 (4.0 mm) $\times$ 16–20 mm Pan Head Wood Screws + M4 Flat Washers | 4 pcs | Bolting 1× xArm 1S metal baseplate to MDF board | **3.0 mm (1/8")** *(Depth: 12–14 mm pilot)* |
| *Alt* | *xArm Base Through-Bolts* | M4 $\times$ 25–30 mm Machine Bolts + M4 Washers + M4 Hex Nuts | 4 pcs | Heavy-duty through-bolting through 18 mm board | **4.0 mm or 4.5 mm** *(Through-hole)* |
| **C2** | **Stand Base Bracket Screws** | #8 (4.0 mm) $\times$ 16 mm Wood Screws OR M5 bolts | 4 pcs | Anchoring bottom corner brackets into MDF board | **3.0 mm** (pilot) / **5.0 mm** (M5 bolt) |
| **C3** | **Stand Bracket-to-Aluminum** | M5 $\times$ 10 mm Machine Screws OR 4.0 mm Blind Pop-Rivets | 4 pcs | Fastening bracket into the aluminum vertical post | **4.2 mm** (rivet) / **5.0 mm** (M5 screw) |
| **C4** | **Camera Tip Mounting Bolt** | **1/4"-20 UNC Hex Bolt (1/2" or 3/4" length)** + 1/4" Nut + Washer | 1 set | Bolted through horizontal tube tip for ring light ball head | **6.5 mm (1/4")** *(Through-hole in tube)* |
| **C5** | **Container Box Attachment** | Double-sided foam mounting tape | 1 strip | Securing physical container box to Rear Flank | **No drilling needed** |

> [!TIP]
> **Tool Summary**: You only need **three drill bits** for the entire build:
> 1. **3.0 mm (1/8")**: Pilot holes into MDF for wood screws.
> 2. **4.5 mm / 5.0 mm**: Through-holes for M4/M5 machine bolts or corner brackets.
> 3. **6.5 mm (1/4")**: Through-hole at the aluminum cantilever tip for the 1/4"-20 ring light bolt.

---

### Category D: Target Cubes & Vision Targets

| Item | Component | Placement | Specs | Quantity |
| :---: | :--- | :--- | :--- | :---: |
| **D1** | **Planar ArUco Markers** | Cells (3,0), (3,6), (11,6), (11,0) | 4× `DICT_4X4_50` printed at $50\times 50\text{ mm}$ on matte vinyl sticker paper | 4 pcs (IDs 0, 1, 2, 3) |
| **D2** | **Target Craft Cubes (Red)** | Active Picking Arena | 1-inch ($25.4\text{ mm}$) blocks painted ultra-matte Red | 4–8 pcs (~15 g each) |
| **D3** | **Distractor Cubes (Green)** | Active Picking Arena | 1-inch ($25.4\text{ mm}$) blocks painted ultra-matte Green | 4–8 pcs (~15 g each) |
| **D4** | **Electronics Mounting** | Rear Bay ($Y \in [500, 600\text{ mm}]$) | Adhesive Nylon PCB Standoffs (M3) & stick-on cable clips | 6 standoffs, 4 clips |

---

## 💰 2. Estimated Workshop Procurement Summary

| Subsystem | Components Included | Estimated Cost (INR / USD) |
| :--- | :--- | :---: |
| **Baseboard & Arena** | Platform board ($350\times 600\times 18\text{ mm}$), foam tape, rubber feet | ₹650 (~$7.80) |
| **Scrap Aluminum Stand** | Scrap door profile (1.1 m) + welding + 2 L-brackets | ₹300 (~$3.50) |
| **Ring Light & Phone Mount** | Invicto 10" USB LED Ring Light with Ball Head & Phone Clamp | ₹249 (~$3.00) |
| **Fasteners & Hardware** | Wood screws, M4/M5 bolts, washers, 1/4"-20 bolt & nut | ₹80 (~$1.00) |
| **Container Box** | Physical plastic/cardboard collection container box | ₹50 (~$0.60) |
| **Target & Distractor Cubes**| 12–16 wooden blocks + Red & Green matte spray paint | ₹250 (~$3.00) |
| **ArUco Stickers & Clips** | Matte sticker sheet, PCB standoffs, wire clips | ₹150 (~$1.80) |
| **TOTAL WORKCELL FABRICATION COST** | *(Excluding robot arm & SBC)* | **$\approx$ ₹1,729 (~$20.70)** |
