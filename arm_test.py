#!/usr/bin/env python3
"""CLI to drive the arm over HTTP (see docs/USAGE.md).

Thin shim kept at the repo root so `python3 arm_test.py ...` still works; the
logic lives in the package.
"""
from moneysort.interface.cli import main

if __name__ == "__main__":
    main()
