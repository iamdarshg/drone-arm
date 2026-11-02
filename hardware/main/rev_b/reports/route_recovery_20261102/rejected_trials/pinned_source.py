"""Materialize the immutable 16-open board used by rejected-route replays."""
import hashlib
import pathlib
import shutil
import subprocess
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[5]
SOURCE_REV = "7f9a9eff504ad5ebc7f33bd1af25ee743d9381c1"
SOURCE_REL = "hardware/main/rev_b/main_rev_b.kicad_pcb"
SOURCE_SHA256 = "60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191"
PROJECT = ROOT / "hardware/main/rev_b/main_rev_b.kicad_pro"
PROJECT_SHA256 = "f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"


def materialize_candidate16(output_directory: pathlib.Path) -> pathlib.Path:
    """Write and hash-check the pinned historical 16-open board and project."""
    output_directory.mkdir(parents=True, exist_ok=True)
    data = subprocess.run(
        ["git", "-C", str(ROOT), "show", f"{SOURCE_REV}:{SOURCE_REL}"],
        check=True,
        stdout=subprocess.PIPE,
    ).stdout
    source = output_directory / "source_16_open.kicad_pcb"
    source.write_bytes(data)
    if hashlib.sha256(data).hexdigest() != SOURCE_SHA256:
        raise RuntimeError("pinned 16-open board hash mismatch")
    if hashlib.sha256(PROJECT.read_bytes()).hexdigest() != PROJECT_SHA256:
        raise RuntimeError("pinned KiCad project hash mismatch")
    shutil.copyfile(PROJECT, output_directory / "main_rev_b.kicad_pro")
    return source


def candidate16_board() -> pathlib.Path:
    """Return a cached copy of the immutable 16-open board Git object."""
    cache = pathlib.Path(tempfile.gettempdir()) / "drone-arm-pinned-route-replay"
    return materialize_candidate16(cache)
