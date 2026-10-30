import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.firmware_source_audit import AuditError, analyze_commit, discover_candidates, resolve_commit


def _git(repo: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    ).stdout.strip()


def _commit(repo: Path, name: str, content: str) -> str:
    path = repo / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    _git(repo, "add", name)
    _git(repo, "commit", "-m", f"fixture: {name}")
    return _git(repo, "rev-parse", "HEAD")


@pytest.fixture
def source_repo(tmp_path: Path) -> Path:
    repo = tmp_path / "repo"
    repo.mkdir()
    _git(repo, "init", "-b", "main")
    _git(repo, "config", "user.name", "Acceptance Fixture")
    _git(repo, "config", "user.email", "acceptance@example.invalid")
    (repo / "src").mkdir()
    (repo / "include").mkdir()
    (repo / "include/fixture.h").write_text("#define FIXTURE 1\n", encoding="utf-8")
    _git(repo, "add", "include/fixture.h")
    (repo / "hardware/main/rev_b").mkdir(parents=True)
    (repo / "hardware/main/rev_b/mcu_pinmap.json").write_text(
        '{"board":"fixture","mcu":"RP2354B","pins":{"1":{"net":"PWM_ESC1"}}}\n',
        encoding="utf-8",
    )
    _git(repo, "add", "hardware/main/rev_b/mcu_pinmap.json")
    return repo


def test_invalid_candidate_revision_is_rejected(source_repo: Path) -> None:
    _commit(source_repo, "meson.build", "project('fixture', 'c')\n")
    with pytest.raises(AuditError, match="invalid candidate revision"):
        resolve_commit(source_repo, "deadbeef-not-a-commit")


def test_stale_expected_sha_is_reported_and_candidate_history_is_discovered(source_repo: Path) -> None:
    _commit(source_repo, "meson.build", "project('fixture', 'c')\nsource = 'src/main.c'\ninc = include_directories('include')\n")
    old = _commit(source_repo, "src/main.c", "int main(void) { return 0; }\n")
    (source_repo / "src/main.c").write_text("int main(void) { return 1; }\n", encoding="utf-8")
    _git(source_repo, "add", "src/main.c")
    _git(source_repo, "commit", "-m", "fixture: change source")
    current = _git(source_repo, "rev-parse", "HEAD")

    report = analyze_commit(source_repo, old, expected_revision=current)
    assert report["status"] == "FAIL"
    assert "stale source SHA" in report["reasons"][0]

    candidates = discover_candidates(source_repo)
    assert {candidate["revision"]["commit_sha"] for candidate in candidates} >= {old, current}
    current_record = next(c for c in candidates if c["revision"]["commit_sha"] == current)
    source_record = next(f for f in current_record["source_files"] if f["path"] == "src/main.c")
    assert len(source_record["sha256"]) == 64
    assert current_record["pinmap"]["status"] == "PARSED"
    assert current_record["pinmap"]["mcu"] == "RP2354B"


def test_missing_manifest_path_fails_with_exact_source_and_include_paths(source_repo: Path) -> None:
    revision = _commit(
        source_repo,
        "meson.build",
        "project('fixture', 'c')\nsource = 'src/not_tracked.c'\ninc = include_directories('include/not_tracked')\n",
    )
    report = analyze_commit(source_repo, revision)
    assert report["status"] == "FAIL"
    assert report["meson_manifest"]["missing_source_paths"] == ["src/not_tracked.c"]
    assert report["meson_manifest"]["missing_include_paths"] == ["include/not_tracked"]
