#!/usr/bin/env python3
"""ROS 2 wrapper for the Master Autonomous System execution pipeline."""
import os
import sys

# Ensure repository root is on sys.path
script_dir = os.path.dirname(os.path.realpath(__file__))
repo_root = os.path.abspath(os.path.join(script_dir, "..", "..", ".."))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

from tools.run_autonomous_system import main

if __name__ == "__main__":
    main()
