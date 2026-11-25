#!/usr/bin/env python3
"""
Generate printable ArUco markers (DICT_4X4_50, IDs 0-3) for arena corner calibration.
Outputs high-resolution individual PNGs, exact 50x50 mm vector PDFs (A4), and 300 DPI sheets.
"""

import sys
import os

# Add repository root and vision dir to path
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

try:
    from vision.generate_printable_sheet import main
except ImportError:
    from generate_printable_sheet import main

if __name__ == "__main__":
    main()
