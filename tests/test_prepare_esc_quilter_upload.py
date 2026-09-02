"""Quilter package must preserve the reconciled 165 mm ESC design."""

from pathlib import Path
import subprocess


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "tools/hardware/prepare_quilter_upload.py"
SOURCE = REPO / "hardware/esc/rev_b/reports/esc_165mm_usb_reconciled.kicad_pcb"
NETLIST = REPO / "hardware/esc/rev_b/reports/esc_usb_netlist.xml"
OUTPUT = REPO / ".superpowers/sdd/2026-08-30-esc-usb-programming/quilter-test"
KICAD_PYTHON = Path(r"C:\Program Files\KiCad\9.0\bin\python.exe")


def test_flat_package_preserves_165mm_board_and_has_exact_parity():
    result = subprocess.run(
        [str(KICAD_PYTHON), str(SCRIPT), "--source", str(SOURCE), "--netlist", str(NETLIST), "--output", str(OUTPUT)],
        cwd=REPO,
        text=True,
        capture_output=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert '"outline_exact_165mm": true' in result.stdout
    assert '"footprint_parity": true' in result.stdout
    assert '"source_layout_preserved": true' in result.stdout
    assert not [path for path in OUTPUT.iterdir() if path.is_dir()]
