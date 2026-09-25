#!/usr/bin/env python3
"""Collect an FPM-built static core into a minimal CMake install prefix."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--prefix", type=Path, required=True)
    args = parser.parse_args()

    libraries = sorted(args.build.rglob("libfabrik_core.a"))
    if not libraries:
        raise SystemExit(f"libfabrik_core.a not found below {args.build}")
    header = args.build.parent / "include" / "fabrik_core.h"
    if not header.is_file():
        raise SystemExit(f"fabrik_core.h not found at {header}")

    include_dir = args.prefix / "include"
    lib_dir = args.prefix / "lib"
    include_dir.mkdir(parents=True, exist_ok=True)
    lib_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(header, include_dir / header.name)
    shutil.copy2(libraries[0], lib_dir / "libfabrik_core.a")
    print(args.prefix)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
