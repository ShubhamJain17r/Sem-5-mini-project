# 3D CAD & Fabrication Models — Single xArm 1S

This directory is organized into dedicated subdirectories so scripts and models are strictly separated:

```text
cad/
├── README.md               # Directory overview & slicing specs (this document)
├── models/                 # Production-ready 3D CAD & mesh files (.step & .stl)
│   ├── sorting_tray_2x2.step   # STEP file for CAD modeling & modification
│   └── sorting_tray_2x2.stl    # STL mesh ready for 3D printer slicers
└── scripts/                # Parametric CAD generator scripts (Python / build123d)
    └── generate_tray.py    # Parametric generator for the 2x2 sorting tray
```

---

## 📦 Production Models (`cad/models/`)

| Component | STEP File | STL File | Description |
| :--- | :--- | :--- | :--- |
| **2×2 Color Sorting Tray** | [`models/sorting_tray_2x2.step`](models/sorting_tray_2x2.step) | [`models/sorting_tray_2x2.stl`](models/sorting_tray_2x2.stl) | 4-compartment collection bin for Red, Green, Blue, Yellow blocks (1 unit required on Left Flank) |

> [!NOTE]
> **Camera Stand Fabrication**: The overhead camera and 10" ring light mount uses a workshop-welded **scrap aluminum door profile** ($700\text{ mm}$ vertical $\times 400\text{ mm}$ horizontal) with a standard **1/4"-20 UNC** tip bolt, eliminating the need for bulky 3D-printed camera brackets.

---

## 🖨️ Slicing & 3D Printing Recommendations (`sorting_tray_2x2.stl`)

- **Outer Dimensions**: $80.0\text{ mm} \times 80.0\text{ mm} \times 35.0\text{ mm}$ Height.
- **Compartments**: 4 quadrant pockets ($36\times 36\text{ mm} \times 32\text{ mm}$ deep), holds 2 stacked 25 mm craft cubes per pocket.
- **Material**: **PLA+** or **PETG** (White / Light Gray recommended for high contrast).
- **Layer Height**: $0.20\text{ mm}$.
- **Walls / Perimeters**: 3 walls.
- **Infill**: 15% Grid or Gyroid.
- **Supports**: **Zero supports needed (100% support-free)**.
- **Print Time**: $\approx 45\text{ minutes}$ (only 1 unit needed).
- **Filament Mass**: $\approx 42\text{ g}$ total (**$< 45\text{ g}$**).

---

## 🛠️ Re-generating Models

To modify or regenerate the 3D models using `build123d`:

```bash
python3 cad/scripts/generate_tray.py
```
Outputs are automatically placed into [`cad/models/`](models/).
