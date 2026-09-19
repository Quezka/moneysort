#!/usr/bin/env python3
"""Entry point for the Money Sorter arm daemon (systemd `moneysort-arm`).

Thin shim: the daemon lives in the package. Kept at the repo root so the
systemd unit's `ExecStart=/usr/bin/python3 .../armd.py` needs no change.
"""
from moneysort.interface.server import main

if __name__ == "__main__":
    main()
