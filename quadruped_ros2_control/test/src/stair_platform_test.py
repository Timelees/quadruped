#!/usr/bin/env python3
"""Compatibility entry point: run one explicit, independently spawned scenario.

Use --scenario stairs or --scenario platform, --output PATH, --speed .4.
The platform scenario starts at the origin and turns using rx (yaw).
"""
import runpy
from pathlib import Path

if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).with_name('terrainSafetyCorridor_test')/'run_trial.py'), run_name='__main__')
