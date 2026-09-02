"""Regression checks for adding USB footprints without moving the hand placement."""

from pathlib import Path
import subprocess


REPO = Path(__file__).resolve().parents[1]
SOURCE = REPO / "hardware/esc/rev_b/reports/esc_165mm_hybrid_newtop_oldbottom_tight.kicad_pcb"
OUTPUT = REPO / ".superpowers/sdd/2026-08-30-esc-usb-programming/reconcile-test.kicad_pcb"
SCRIPT = REPO / "tools/hardware/reconcile_esc_usb_footprints.py"
KICAD_PYTHON = Path(r"C:\Program Files\KiCad\9.0\bin\python.exe")


def test_reconciliation_preserves_existing_board_and_adds_every_usb_reference():
    result = subprocess.run(
        [str(KICAD_PYTHON), str(SCRIPT), "--source", str(SOURCE), "--output", str(OUTPUT), "--check"],
        cwd=REPO,
        text=True,
        capture_output=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "existing_transforms_changed=0" in result.stdout
    assert "missing_references=0" in result.stdout
    assert "stale_references=0" in result.stdout
    assert "new_inside_edge_cuts=0" in result.stdout
