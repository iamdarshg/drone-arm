#!/usr/bin/env python3
"""Audit Meson firmware inputs and discover historical source candidates.

This is a provenance/checkability tool. A complete build manifest does not
mean the code is compatible with a board or safe to run.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any


SOURCE_SUFFIXES = {".c", ".S", ".s", ".ld"}
MESON_PATH_RE = re.compile(r"['\"]((?:src|include|tools)(?:/[^'\"]*)?)['\"]")
INCLUDE_CALL_RE = re.compile(r"include_directories\(([^)]*)\)", re.DOTALL)
CAPABILITY_PATTERNS = {
    "rp2354b_target": re.compile(r"RP2354B", re.IGNORECASE),
    "can_fd": re.compile(r"CAN[-_ ]?FD|MCP2518FD", re.IGNORECASE),
    "stm32g431": re.compile(r"STM32G431", re.IGNORECASE),
    "foc": re.compile(r"\bFOC\b|field[- ]oriented control", re.IGNORECASE),
    "flight_control": re.compile(
        r"flight[-_ ]control|control[-_ ]allocator|state[-_ ]estimator",
        re.IGNORECASE,
    ),
    "six_cell_esc": re.compile(r"six.{0,20}(motor|ESC).{0,20}(cell|channel)|6.{0,10}ESC", re.IGNORECASE),
}


class AuditError(RuntimeError):
    """Raised when an audit input cannot be resolved as a Git commit."""


def _git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode:
        raise AuditError(result.stderr.strip() or f"git {' '.join(args)} failed")
    return result.stdout.strip()


def resolve_commit(repo: Path, revision: str) -> str:
    """Resolve only commit-ish revisions to a full immutable commit ID."""
    try:
        full = _git(repo, "rev-parse", "--verify", f"{revision}^{{commit}}")
    except AuditError as exc:
        raise AuditError(f"invalid candidate revision {revision!r}: {exc}") from exc
    if not re.fullmatch(r"[0-9a-f]{40,64}", full):
        raise AuditError(f"resolved revision is not a full commit SHA: {full!r}")
    return full


def _tree(repo: Path, commit: str) -> dict[str, str]:
    rows = _git(repo, "ls-tree", "-r", "-z", "--full-tree", commit).split("\0")
    tree: dict[str, str] = {}
    for row in rows:
        if not row:
            continue
        metadata, path = row.split("\t", 1)
        mode, kind, oid = metadata.split()
        if kind == "blob":
            tree[path] = oid
    return tree


def _meson_paths(
    repo: Path, commit: str, tree: dict[str, str]
) -> tuple[list[str], list[str], list[str], str | None]:
    oid = tree.get("meson.build")
    if oid is None:
        return [], [], [], None
    content = _git(repo, "cat-file", "blob", oid)
    # Remove Meson line comments before extracting literal paths.
    content = re.sub(r"(?m)#.*$", "", content)
    paths = set(MESON_PATH_RE.findall(content))
    files = sorted(p for p in paths if Path(p).suffix in SOURCE_SUFFIXES)
    build_tools = sorted(p for p in paths if p.startswith("tools/"))
    directories = set()
    for call in INCLUDE_CALL_RE.findall(content):
        directories.update(MESON_PATH_RE.findall(call))
    directories = sorted(p.rstrip("/") for p in directories if p.startswith(("src/", "include/")) or p in {"src", "include"})
    return files, directories, build_tools, content


def _is_present(path: str, tree: dict[str, str]) -> bool:
    return path in tree or any(name.startswith(path.rstrip("/") + "/") for name in tree)


def _hash_sources(repo: Path, tree: dict[str, str]) -> list[dict[str, str]]:
    records = []
    for path in sorted(
        p for p in tree if (p.startswith("src/") or p.startswith("include/"))
        and Path(p).suffix in {".c", ".h", ".S", ".s", ".ld"}
    ):
        oid = tree[path]
        data = subprocess.run(
            ["git", "-C", str(repo), "cat-file", "blob", oid],
            check=True,
            stdout=subprocess.PIPE,
        ).stdout
        records.append({"path": path, "git_blob_sha": oid, "sha256": hashlib.sha256(data).hexdigest()})
    return records


def _hash_paths(repo: Path, tree: dict[str, str], paths: list[str]) -> list[dict[str, str]]:
    records = []
    for path in sorted(set(paths)):
        oid = tree.get(path)
        if oid is None:
            continue
        data = subprocess.run(
            ["git", "-C", str(repo), "cat-file", "blob", oid],
            check=True,
            stdout=subprocess.PIPE,
        ).stdout
        records.append({"path": path, "git_blob_sha": oid, "sha256": hashlib.sha256(data).hexdigest()})
    return records


def _pinmap(repo: Path, tree: dict[str, str]) -> dict[str, Any]:
    path = "hardware/main/rev_b/mcu_pinmap.json"
    oid = tree.get(path)
    if oid is None:
        return {"path": path, "present": False, "status": "ABSENT"}
    raw = subprocess.run(
        ["git", "-C", str(repo), "cat-file", "blob", oid],
        check=True,
        stdout=subprocess.PIPE,
    ).stdout
    try:
        value = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        return {"path": path, "present": True, "status": "INVALID_JSON", "error": str(exc), "git_blob_sha": oid}
    pins = value.get("pins")
    net_values = [entry.get("net", "") for entry in pins.values()] if isinstance(pins, dict) else []
    return {
        "path": path,
        "present": True,
        "status": "PARSED",
        "git_blob_sha": oid,
        "board": value.get("board"),
        "mcu": value.get("mcu"),
        "pin_count": len(pins) if isinstance(pins, dict) else None,
        "sha256": hashlib.sha256(raw).hexdigest(),
        "declares_six_pwm_nets": bool(
            isinstance(pins, dict)
            and all(any(d.get("net", "").endswith(f"PWM_ESC{i}") for d in pins.values()) for i in range(1, 7))
        ),
        "can_interface_pin_count": sum("CAN" in net for net in net_values),
    }


def analyze_commit(repo: Path, revision: str, expected_revision: str | None = None) -> dict[str, Any]:
    commit = resolve_commit(repo, revision)
    expected = resolve_commit(repo, expected_revision) if expected_revision else None
    tree = _tree(repo, commit)
    meson_files, meson_dirs, build_tools, meson_content = _meson_paths(repo, commit, tree)
    missing_files = [path for path in meson_files if path not in tree]
    missing_dirs = [path for path in meson_dirs if not _is_present(path, tree)]
    missing_build_tools = [path for path in build_tools if path not in tree]
    source_records = _hash_sources(repo, tree)
    input_hashes = _hash_paths(
        repo,
        tree,
        ["meson.build", "arm-none-eabi.ini", "hardware/main/rev_b/mcu_pinmap.json", *build_tools],
    )
    source_text = "\n".join(
        subprocess.run(["git", "-C", str(repo), "cat-file", "blob", rec["git_blob_sha"]], check=True, stdout=subprocess.PIPE).stdout.decode("utf-8", "replace")
        for rec in source_records
    )
    capability = {name: bool(pattern.search(source_text)) for name, pattern in CAPABILITY_PATTERNS.items()}
    build_target_markers = {
        "rp2354b": bool(meson_content and re.search(r"RP2354B", meson_content, re.IGNORECASE)),
        "rp2350_uf2_family": bool(meson_content and re.search(r"rp2350-arm", meson_content, re.IGNORECASE)),
        "rp2350_arm_s_uf2_family": bool(meson_content and re.search(r"rp2350-arm-s", meson_content, re.IGNORECASE)),
    }
    manifest = {
        "meson_present": meson_content is not None,
        "source_paths": meson_files,
        "include_paths": meson_dirs,
        "missing_source_paths": missing_files,
        "missing_include_paths": missing_dirs,
        "build_tool_paths": build_tools,
        "missing_build_tool_paths": missing_build_tools,
        "complete": meson_content is not None and not missing_files and not missing_dirs and not missing_build_tools,
    }
    status = "FAIL" if expected and expected != commit else ("PASS" if manifest["complete"] else "FAIL")
    reasons = []
    if expected and expected != commit:
        reasons.append(f"stale source SHA: expected {expected}, inspected {commit}")
    if missing_files:
        reasons.append("Meson source files are absent from this tracked tree")
    if missing_dirs:
        reasons.append("Meson include directories are absent from this tracked tree")
    if missing_build_tools:
        reasons.append("Meson build helper paths are absent from this tracked tree")
    if not meson_content:
        reasons.append("meson.build is absent")
    pinmap_record = _pinmap(repo, tree)
    return {
        "revision": {"requested": revision, "commit_sha": commit, "expected_sha": expected},
        "status": status,
        "reasons": reasons,
        "meson_manifest": manifest,
        "source_files": source_records,
        "build_input_hashes": input_hashes,
        "pinmap": pinmap_record,
        "source_capabilities_detected": capability,
        "build_target_markers": build_target_markers,
        "production_firmware_compatible": bool(
            manifest["complete"]
            and capability["rp2354b_target"]
            and capability["flight_control"]
            and capability["can_fd"]
            and capability["six_cell_esc"]
            and capability["foc"]
            and capability["stm32g431"]
            and pinmap_record["status"] == "PARSED"
            and "RP2354B" in str(pinmap_record.get("mcu", ""))
            and pinmap_record["declares_six_pwm_nets"]
        ),
    }


def discover_candidates(repo: Path, limit: int = 64) -> list[dict[str, Any]]:
    """Inspect available ref tips and commits which changed a root Meson file."""
    commits = _git(
        repo,
        "log",
        "--all",
        f"--max-count={limit}",
        "--format=%H",
        "--",
        "meson.build",
        "src",
        "include",
    ).splitlines()
    ref_tips = _git(repo, "for-each-ref", "--format=%(objectname)", "refs/heads", "refs/remotes").splitlines()
    commits = list(dict.fromkeys(ref_tips + commits))
    found: list[dict[str, Any]] = []
    seen: dict[tuple[Any, ...], int] = {}
    for sha in commits:
        result = analyze_commit(repo, sha)
        manifest = result["meson_manifest"]
        fingerprint = (
            tuple(manifest["source_paths"]),
            tuple(manifest["missing_source_paths"]),
            tuple(manifest["missing_include_paths"]),
            tuple(manifest["build_tool_paths"]),
            tuple(manifest["missing_build_tool_paths"]),
            tuple((item["path"], item["git_blob_sha"]) for item in result["source_files"]),
        )
        if fingerprint in seen:
            index = seen[fingerprint]
            found[index]["equivalent_commit_shas"].append(result["revision"]["commit_sha"])
            continue
        result["equivalent_commit_shas"] = [result["revision"]["commit_sha"]]
        seen[fingerprint] = len(found)
        found.append(result)
    return found


def build_report(repo: Path, revision: str, expected_revision: str | None = None, candidate_limit: int = 64) -> dict[str, Any]:
    baseline = analyze_commit(repo, revision, expected_revision)
    candidates = discover_candidates(repo, candidate_limit)
    return {
        "schema": "firmware-source-audit/v1",
        "baseline": baseline,
        "historical_meson_candidates": candidates,
        "candidate_discovery": {
            "method": "available ref tips plus git log --all over meson.build and src/include changes, resolved in each commit tree",
            "candidate_count": len(candidates),
            "history_limit": candidate_limit,
            "production_compatible_count": sum(c["production_firmware_compatible"] for c in candidates),
        },
        "invocation": {
            "revision": revision,
            "expected_revision": expected_revision,
            "candidate_limit": candidate_limit,
            "baseline_exit_code": 0 if baseline["status"] == "PASS" else 1,
        },
        "interpretation": "Meson paths are literal-path checks only; actual Meson setup is the build-graph oracle. Manifest completeness and source markers are provenance signals, not evidence of target boot, safe output, or production firmware quality.",
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path.cwd())
    parser.add_argument("--revision", default="HEAD")
    parser.add_argument("--expected-revision")
    parser.add_argument("--candidate-limit", type=int, default=64)
    parser.add_argument("--output", type=Path, help="write JSON to this file instead of stdout")
    args = parser.parse_args(argv)
    try:
        report = build_report(args.repo.resolve(), args.revision, args.expected_revision, args.candidate_limit)
    except (AuditError, subprocess.CalledProcessError) as exc:
        print(json.dumps({"schema": "firmware-source-audit/v1", "status": "FAIL", "error": str(exc)}, indent=2), file=sys.stderr)
        return 2
    report["invocation"]["argv"] = sys.argv
    report["invocation"]["working_directory"] = str(Path.cwd())
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    else:
        sys.stdout.write(rendered)
    return 0 if report["baseline"]["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
