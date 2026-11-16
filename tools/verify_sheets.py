#!/usr/bin/env python3
import cv2
import numpy as np

def verify_sheet(image_path: str, is_a3: bool = True):
    print(f"\n==========================================")
    print(f"VERIFYING: {image_path}")
    print(f"==========================================")
    img = cv2.imread(image_path)
    assert img is not None, f"Failed to load {image_path}"
    H, W, _ = img.shape
    DPI = 300.0
    PPM = DPI / 25.4

    # 1. Detect markers with OpenCV
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(aruco_dict)
        corners, ids, _ = detector.detectMarkers(img)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(img, aruco_dict)

    assert ids is not None and len(ids) == 4, f"Expected 4 markers, found {ids}"
    detected_ids = ids.flatten()
    print(f"[PASS] Successfully detected all 4 ArUco markers: {detected_ids}")

    # 2. Check each marker for zero text/non-binary contamination
    specs = [
        (0, 70.0, 180.0),
        (1, 330.0, 180.0),
        (2, 70.0, 40.0),
        (3, 330.0, 40.0),
    ]

    for mid, xw, yw in specs:
        xp = xw - 51.5
        page_h_mm = 420.0 if is_a3 else 210.0
        yp = page_h_mm - yw
        cx = int(round(xp * PPM))
        cy = int(round(yp * PPM))
        half_m = int(round(13.0 * PPM))

        # Crop exactly inside the black border of marker (excluding red crosshair ticks)
        # Inset by 1 px to avoid sub-pixel anti-aliasing on outer edge
        marker_crop = img[cy - half_m + 1 : cy + half_m - 1, cx - half_m + 1 : cx + half_m - 1]
        gray = cv2.cvtColor(marker_crop, cv2.COLOR_BGR2GRAY)

        corrupted_mask = (gray > 25) & (gray < 230)
        corrupted = np.sum(corrupted_mask)
        if corrupted > 0:
            coords = np.argwhere(corrupted_mask)
            print(f"  Marker {mid} corrupted coords inside crop:\n{coords}")
            for y, x in coords[:5]:
                print(f"    at ({x}, {y}): gray={gray[y, x]}, bgr={marker_crop[y, x]}")
        print(f"  Marker {mid} at ({cx}, {cy}): {corrupted} contaminated/text pixels")
        assert corrupted == 0, f"Marker {mid} has text or non-binary pixels!"

        # Check quiet zone (1 mm outside the marker edge, excluding the 4 red ticks)
        # Check a 1 mm strip above the marker
        top_strip = img[cy - half_m - int(round(PPM)) : cy - half_m, cx - half_m : cx + half_m]
        # Ignore center column where red tick might be
        center_x = top_strip.shape[1] // 2
        left_sub = top_strip[:, : center_x - int(round(2.0 * PPM))]
        right_sub = top_strip[:, center_x + int(round(2.0 * PPM)) :]
        assert np.all(left_sub == 255) and np.all(right_sub == 255), f"Quiet zone above marker {mid} has non-white pixels!"

    print("[PASS] All 4 ArUco markers are 100% pristine with zero text contamination!")

    # 3. Check active arena interior (must be 100% solid spotless white)
    # Arena: X in [70, 330], Y in [40, 180]
    # Safe active field between markers: X in [33, 264] mm, Y_world in [54, 166] mm
    page_h_mm = 420.0 if is_a3 else 210.0
    x1 = int(round(33.0 * PPM))
    x2 = int(round(264.0 * PPM))
    y1 = int(round((page_h_mm - 166.0) * PPM))
    y2 = int(round((page_h_mm - 54.0) * PPM))

    arena_sub = img[y1:y2, x1:x2]
    non_white = np.sum(np.any(arena_sub < 255, axis=-1))
    print(f"  Active picking field: {arena_sub.shape[0] * arena_sub.shape[1]} px, Non-white: {non_white}")
    assert non_white == 0, "Active arena interior must be 100% spotless white!"
    print("[PASS] Active arena interior is 100% spotless solid white!")

if __name__ == "__main__":
    verify_sheet("docs/assets/aruco_markers/master_workcell_sheet_A3_1to1.png", is_a3=True)
    verify_sheet("docs/assets/aruco_markers/arena_aruco_sheet_A4_1to1.png", is_a3=False)
    print("\nALL VERIFICATIONS PASSED SUCCESSFULLY!")
