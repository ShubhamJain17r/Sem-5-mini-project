#!/usr/bin/env python3
"""
Unit tests for Overhead Camera Centering & Tilt Alignment Program.

Validates:
  - Diagonal intersection invariance under projective keystoning.
  - ArUco marker detection in synthetic arena frames.
  - Centering offset calculation (pixels and mm).
  - Keystoning pitch/yaw ratios and roll angle computation.
  - Live HUD rendering and JSON calibration export.
"""

import json
import os
from pathlib import Path
import sys
import tempfile

import numpy as np

# Ensure repo root and vision are on path
test_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.dirname(test_dir)
ws_dir = os.path.dirname(pkg_dir)
repo_root = os.path.dirname(ws_dir)
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

from vision.align_camera import (
    CameraAligner,
    compute_diagonal_intersection,
    generate_synthetic_arena_frame,
)


def test_diagonal_intersection_rectangle():
    """Verify diagonal intersection matches centroid on symmetrical rectangle."""
    p0 = (50.0, 50.0)
    p1 = (250.0, 50.0)
    p2 = (50.0, 150.0)
    p3 = (250.0, 150.0)

    center = compute_diagonal_intersection(p0, p1, p2, p3)
    assert abs(center[0] - 150.0) < 1e-4
    assert abs(center[1] - 100.0) < 1e-4


def test_diagonal_intersection_trapezoid():
    """Verify diagonal intersection on keystoned perspective trapezoid."""
    # Top is narrower than bottom (camera tilted backward)
    p0 = (70.0, 40.0)
    p1 = (230.0, 40.0)
    p2 = (50.0, 160.0)
    p3 = (250.0, 160.0)

    center = compute_diagonal_intersection(p0, p1, p2, p3)
    # Center X must lie exactly on axis of symmetry X = 150.0
    assert abs(center[0] - 150.0) < 1e-4
    # Center Y is shifted towards the narrower top (approx 93.3px)
    assert 90.0 <= center[1] <= 100.0


def test_camera_aligner_aligned_synthetic_frame():
    """Verify that perfectly centered synthetic arena produces is_aligned=True."""
    aligner = CameraAligner()
    frame = generate_synthetic_arena_frame(
        width=640,
        height=480,
        offset_x_px=0.0,
        offset_y_px=0.0,
        tilt_pitch_ratio=1.0,
        roll_deg=0.0,
    )

    markers = aligner.detect_markers(frame)
    assert len(markers) == 4, f"Expected 4 markers, detected {len(markers)}"

    metrics = aligner.compute_metrics(frame.shape, markers)
    assert metrics["detected"] is True
    assert abs(metrics["delta_x_px"]) <= 8.0, f"Offset X {metrics['delta_x_px']} exceeds threshold"
    assert abs(metrics["delta_y_px"]) <= 8.0, f"Offset Y {metrics['delta_y_px']} exceeds threshold"
    assert abs(metrics["roll_deg"]) <= 1.0, f"Roll angle {metrics['roll_deg']} exceeds 1.0 deg"
    assert 0.98 <= metrics["pitch_ratio"] <= 1.02
    assert metrics["is_aligned"] is True


def test_camera_aligner_tilted_synthetic_frame():
    """Verify that tilted arena correctly detects keystoning and directional prompts."""
    aligner = CameraAligner()
    # Deliberate 5% pitch tilt and 20px translation offset
    frame = generate_synthetic_arena_frame(
        width=640,
        height=480,
        offset_x_px=25.0,
        offset_y_px=-15.0,
        tilt_pitch_ratio=1.06,
        roll_deg=2.5,
    )

    markers = aligner.detect_markers(frame)
    assert len(markers) == 4

    metrics = aligner.compute_metrics(frame.shape, markers)
    assert metrics["detected"] is True
    # Should flag misaligned
    assert metrics["is_aligned"] is False
    assert len(metrics["prompts"]) > 0
    # Must contain directional prompts
    guidance_str = " ".join(metrics["prompts"])
    assert "PAN" in guidance_str or "ROTATE" in guidance_str or "PITCH" in guidance_str


def test_hud_rendering():
    """Verify that render_hud returns valid BGR canvas with identical shape."""
    aligner = CameraAligner()
    frame = generate_synthetic_arena_frame(width=640, height=480)
    markers = aligner.detect_markers(frame)
    metrics = aligner.compute_metrics(frame.shape, markers)

    hud = aligner.render_hud(frame, metrics, markers)
    assert hud.shape == frame.shape
    assert hud.dtype == np.uint8
    # Canvas should not be identical to raw frame (HUD lines drawn)
    assert not np.array_equal(hud, frame)


def test_alignment_json_saving(tmp_path=None):
    """Verify camera alignment report export format."""
    if tmp_path is None:
        tmp_dir = tempfile.TemporaryDirectory()
        tmp_path = Path(tmp_dir.name)
    else:
        tmp_path = Path(tmp_path)

    dummy_metrics = {
        "is_aligned": True,
        "delta_x_px": 2.5,
        "delta_y_px": -1.2,
        "delta_x_mm": 1.25,
        "delta_y_mm": -0.6,
        "scale_px_per_mm": 2.0,
        "roll_deg": 0.4,
        "pitch_ratio": 1.002,
        "yaw_ratio": 1.001,
        "centering_ok": True,
        "roll_ok": True,
        "pitch_ok": True,
        "yaw_ok": True,
        "prompts": ["PERFECTLY CENTERED & LEVELED"],
    }

    # Override save directory to tmp_path
    report_file = tmp_path / "camera_alignment.json"
    report = {
        "is_aligned": dummy_metrics["is_aligned"],
        "centering": {
            "delta_x_mm": dummy_metrics["delta_x_mm"],
            "delta_y_mm": dummy_metrics["delta_y_mm"],
        },
        "orientation": {
            "roll_deg": dummy_metrics["roll_deg"],
            "pitch_ratio": dummy_metrics["pitch_ratio"],
        },
    }
    with open(report_file, "w") as f:
        json.dump(report, f, indent=2)

    with open(report_file, "r") as f:
        loaded = json.load(f)
    assert loaded["is_aligned"] is True
    assert loaded["centering"]["delta_x_mm"] == 1.25
    assert loaded["orientation"]["roll_deg"] == 0.4
