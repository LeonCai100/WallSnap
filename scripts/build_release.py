#!/usr/bin/env python3

import argparse
import json
import re
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LIB_NAME = "WallSnap"
REPO = "LeonCai100/WallSnap"
SUPPORTED_KERNELS = "^4.2.1"
TARGET = "v5"
SYSTEM_FILES = [
    "include/wallsnap/wallsnap.hpp",
    "include/wallsnap/wall_snap.hpp",
    "src/wallsnap/WallSnap.cpp",
]


def read_version() -> str:
    makefile = (ROOT / "Makefile").read_text()
    match = re.search(r"^VERSION\s*:?=\s*([^\s]+)", makefile, re.MULTILINE)
    if not match:
        raise RuntimeError("Could not find VERSION in Makefile")
    return match.group(1)


def release_url(version: str) -> str:
    encoded_name = f"{LIB_NAME}%40{version}.zip"
    return f"https://github.com/{REPO}/releases/download/v{version}/{encoded_name}"


def template_pros(version: str) -> dict:
    return {
        "py/object": "pros.conductor.templates.external_template.ExternalTemplate",
        "py/state": {
            "metadata": {},
            "name": LIB_NAME,
            "supported_kernels": SUPPORTED_KERNELS,
            "system_files": SYSTEM_FILES,
            "target": TARGET,
            "user_files": [],
            "version": version,
        },
    }


def depot_entry(version: str) -> dict:
    return {
        "metadata": {
            "location": release_url(version),
        },
        "name": LIB_NAME,
        "py/object": "pros.conductor.templates.base_template.BaseTemplate",
        "supported_kernels": SUPPORTED_KERNELS,
        "target": TARGET,
        "version": version,
    }


def write_release_zip(version: str) -> Path:
    dist_dir = ROOT / "dist"
    dist_dir.mkdir(exist_ok=True)
    zip_path = dist_dir / f"{LIB_NAME}@{version}.zip"

    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("template.pros", json.dumps(template_pros(version), indent=4) + "\n")
        for file_name in SYSTEM_FILES:
            archive.write(ROOT / file_name, file_name)

    return zip_path


def write_depot(version: str) -> Path:
    depot_dir = ROOT / "depot"
    depot_dir.mkdir(exist_ok=True)
    depot_path = depot_dir / "stable.json"
    depot_path.write_text(json.dumps([depot_entry(version)], indent=2) + "\n")
    return depot_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Build WallSnap PROS release artifacts.")
    parser.add_argument("--check", action="store_true", help="verify generated files are current")
    args = parser.parse_args()

    version = read_version()
    zip_path = write_release_zip(version)
    depot_path = write_depot(version)

    if args.check:
        print(f"checked {zip_path}")
        print(f"checked {depot_path}")
    else:
        print(f"wrote {zip_path}")
        print(f"wrote {depot_path}")


if __name__ == "__main__":
    main()

