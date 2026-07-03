#!/usr/bin/env python3
"""Conservative repository governance checker.

The checker reports potential project-governance issues and never deletes,
moves, or rewrites files.
"""

from __future__ import annotations

import argparse
import fnmatch
import os
from pathlib import Path
from typing import Iterable


REQUIRED_PATHS = [
    "AGENTS.md",
    "PROJECT_STATE.md",
    "CHANGELOG.md",
    "docs/module_status.md",
]

ROOT_FORBIDDEN_NAMES = {
    "summary.md",
    "new_summary.md",
    "final.md",
    "analysis.md",
    "temp.md",
    "notes.md",
    "test.py",
    "old.py",
    "backup.py",
    "copy.py",
}

FORBIDDEN_ANYWHERE = {
    "summary.md",
    "new_summary.md",
    "final.md",
    "analysis.md",
    "temp.md",
}

SUSPICIOUS_PATTERNS = [
    "*.bak",
    "*.backup",
    "*_copy.py",
    "*-copy.py",
    "copy_*.py",
    "old_*.py",
    "*_old.py",
    "backup_*.py",
    "*_backup.py",
]

SKIP_DIRS = {
    ".git",
    ".hg",
    ".svn",
    ".venv",
    "venv",
    "env",
    "__pycache__",
    "node_modules",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    "dist",
    "build",
}

DEPRECATED_MARKERS = ("deprecated", "deprecate", "obsolete", "unused", "legacy")


def rel(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def is_under(path: Path, root: Path, parts: Iterable[str]) -> bool:
    try:
        relative_parts = path.relative_to(root).parts
    except ValueError:
        return False
    wanted = tuple(parts)
    return relative_parts[: len(wanted)] == wanted


def iter_files(root: Path):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        current = Path(dirpath)
        if is_under(current, root, (".codex", "skills")):
            dirnames[:] = []
            continue
        for filename in filenames:
            yield current / filename


def check(root: Path) -> list[tuple[str, str]]:
    findings: list[tuple[str, str]] = []

    for required in REQUIRED_PATHS:
        if not (root / required).exists():
            findings.append(("missing-required-doc", required))

    for child in root.iterdir():
        if child.is_file() and child.name.lower() in ROOT_FORBIDDEN_NAMES:
            findings.append(("root-forbidden-name", child.name))

    for file_path in iter_files(root):
        relative = rel(file_path, root)
        name = file_path.name.lower()

        if name in FORBIDDEN_ANYWHERE and not is_under(
            file_path, root, ("experiments", "runs")
        ):
            findings.append(("ambiguous-doc-name", relative))

        for pattern in SUSPICIOUS_PATTERNS:
            if fnmatch.fnmatch(name, pattern.lower()):
                findings.append(("backup-copy-or-old-file", relative))
                break

        lowered = relative.lower()
        looks_deprecated = any(marker in lowered for marker in DEPRECATED_MARKERS)
        archived = is_under(file_path, root, ("docs", "deprecated"))
        if looks_deprecated and not archived:
            findings.append(("deprecated-looking-outside-archive", relative))

    return findings


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Report project-governance cleanliness issues."
    )
    parser.add_argument("path", nargs="?", default=".", help="Repository root")
    args = parser.parse_args()

    root = Path(args.path).resolve()
    if not root.exists() or not root.is_dir():
        print(f"ERROR: not a directory: {root}")
        return 2

    findings = check(root)
    if not findings:
        print(f"OK: no governance cleanliness issues found in {root}")
        return 0

    print(f"Found {len(findings)} potential governance issue(s) in {root}:")
    for kind, target in findings:
        print(f"- {kind}: {target}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
