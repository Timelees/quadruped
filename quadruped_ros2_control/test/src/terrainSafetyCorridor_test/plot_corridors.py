#!/usr/bin/env python3
"""Compatibility entry point for per-scene corridor figures (schema 2 exports)."""
import sys
from plot_corridor_sections import main

if __name__ == '__main__':
    # Preserve `plot_corridors.py DATA`: write the figures beside the CSVs.
    if len(sys.argv) > 1 and '--output' not in sys.argv:
        sys.argv.extend(['--output', sys.argv[1]])
    main()
