"""Create and verify the harness views of the canonical skill tree.

Two view modes are supported:

``link`` (default)  a single directory symlink (POSIX) or junction (Windows) to
                    ``.github/skills``.  Nothing is duplicated and nothing can drift.
``copy``            a byte-identical mirror directory.  Needed where the view has to
                    exist in a fresh ``git clone`` before any command runs, and where
                    a committed symlink would check out as a plain text file on Windows.

Either way ``.github/skills`` stays the only editable body: a copy view is only
accepted while it is byte-identical (``skillkit validate`` enforces this).
"""

from __future__ import annotations

import json
import os
import shutil
from dataclasses import dataclass
from pathlib import Path

from .model import CANONICAL_REL
from .paths import create_dir_link, is_link_like, points_to, remove_dir_link

STATE_REL = Path(".github/skills/_shared/.skillkit/views.json")

MODE_LINK = "link"
MODE_COPY = "copy"
VIEW_MODES = (MODE_LINK, MODE_COPY)

# Never mirrored: interpreter caches and skillkit's own state file.
COPY_SKIP_PARTS = frozenset({"__pycache__", ".skillkit"})
COPY_SKIP_SUFFIXES = frozenset({".pyc", ".pyo"})


@dataclass(frozen=True)
class View:
    key: str
    relative: Path
    label: str


VIEWS: tuple[View, ...] = (
    View("agents", Path(".agents/skills"), "DSH, Cursor, Gemini CLI, Copilot (VS 2026)"),
    View("claude", Path(".claude/skills"), "Claude Code"),
    View("codex", Path(".codex/skills"), "Codex"),
)

ALIASES = {
    "all": None,
    "agents": "agents",
    "dsh": "agents",
    "cursor": "agents",
    "gemini": "agents",
    "copilot": "agents",
    "claude": "claude",
    "claude-code": "claude",
    "codex": "codex",
}


class SyncError(RuntimeError):
    """Raised when a view cannot be created or verified safely."""


def select_views(harness: str) -> list[View]:
    key = harness.strip().lower()
    if key not in ALIASES:
        raise SyncError(f"unknown harness '{harness}'; choose from: {', '.join(sorted(ALIASES))}")
    selected = ALIASES[key]
    if selected is None:
        return list(VIEWS)
    return [view for view in VIEWS if view.key == selected]


def sync_views(
    repo_root: Path,
    *,
    harness: str = "agents",
    check: bool = False,
    force: bool = False,
    view_mode: str = MODE_LINK,
) -> list[str]:
    if view_mode not in VIEW_MODES:
        raise SyncError(f"unknown view mode '{view_mode}'; choose from: {', '.join(VIEW_MODES)}")

    canonical = (repo_root / CANONICAL_REL).resolve()
    if not canonical.is_dir():
        raise SyncError(f"canonical skill root not found: {canonical}")

    state = _load_state(repo_root)
    messages: list[str] = []
    for view in select_views(harness):
        path = repo_root / view.relative
        if check:
            messages.append(_check_view(path, canonical, view, view_mode))
            continue
        messages.append(_ensure_view(path, canonical, view, state, force, view_mode))

    if not check:
        _save_state(repo_root, state)
    return messages


def iter_files(root: Path):
    """Yield regular files under ``root``, skipping caches and skillkit state."""

    for path in sorted(root.rglob("*")):
        if not path.is_file() or is_link_like(path):
            continue
        parts = set(path.relative_to(root).parts)
        if parts & COPY_SKIP_PARTS or path.suffix in COPY_SKIP_SUFFIXES:
            continue
        yield path


def tree_diff(source: Path, target: Path) -> list[str]:
    """Relative paths that are missing, extra or byte-different in ``target``."""

    if is_link_like(target) or not target.is_dir():
        return ["."]
    source_files = {path.relative_to(source): path for path in iter_files(source)}
    target_files = {path.relative_to(target): path for path in iter_files(target)}

    diffs = [rel.as_posix() for rel in sorted(source_files.keys() - target_files.keys())]
    diffs += [rel.as_posix() for rel in sorted(target_files.keys() - source_files.keys())]
    for rel in sorted(source_files.keys() & target_files.keys()):
        if source_files[rel].read_bytes() != target_files[rel].read_bytes():
            diffs.append(rel.as_posix())
    return diffs


def mirror_tree(source: Path, target: Path) -> tuple[int, int]:
    """Make ``target`` a byte-identical copy of ``source``; return (written, removed)."""

    target.mkdir(parents=True, exist_ok=True)
    source_files = {path.relative_to(source): path for path in iter_files(source)}
    target_files = {path.relative_to(target): path for path in iter_files(target)}

    written = 0
    for rel, src in source_files.items():
        dst = target / rel
        if dst.is_file() and not is_link_like(dst) and src.read_bytes() == dst.read_bytes():
            continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        if is_link_like(dst):
            remove_dir_link(dst)
        shutil.copy2(src, dst)
        written += 1

    removed = 0
    for rel in sorted(target_files.keys() - source_files.keys()):
        (target / rel).unlink()
        removed += 1

    for directory in sorted(
        (path for path in target.rglob("*") if path.is_dir() and not is_link_like(path)),
        key=lambda path: len(path.parts),
        reverse=True,
    ):
        if not any(directory.iterdir()):
            directory.rmdir()
    return written, removed


def _ensure_view(
    path: Path,
    canonical: Path,
    view: View,
    state: dict,
    force: bool,
    view_mode: str,
) -> str:
    recorded = state["views"].get(view.relative.as_posix())

    if view_mode == MODE_COPY:
        if is_link_like(path):
            remove_dir_link(path)
        elif path.is_file():
            if not force:
                raise SyncError(f"{view.relative.as_posix()} exists and is not a directory")
            path.unlink()
        written, removed = mirror_tree(canonical, path)
        state["views"][view.relative.as_posix()] = {
            "target": canonical.as_posix(),
            "method": MODE_COPY,
        }
        if written or removed:
            return (
                f"copied: {view.relative.as_posix()} ({written} written, {removed} removed, "
                f"{view.label})"
            )
        return f"ok: {view.relative.as_posix()} == {canonical} (copy, {view.label})"

    if path.exists() or is_link_like(path):
        if is_link_like(path):
            if points_to(path, canonical):
                state["views"][view.relative.as_posix()] = {
                    "target": canonical.as_posix(),
                    "method": recorded.get("method", _link_method(path)) if recorded else _link_method(path),
                }
                return f"ok: {view.relative.as_posix()} -> {canonical} ({view.label})"
            if not force and not recorded:
                raise SyncError(
                    f"{view.relative.as_posix()} is a link to {path.resolve()}, not {canonical}; "
                    "pass --force to replace it"
                )
            remove_dir_link(path)
        elif path.is_dir():
            entries = list(path.iterdir())
            if entries and not force:
                if not tree_diff(canonical, path):
                    raise SyncError(
                        f"{view.relative.as_posix()} is a byte-identical copy view; "
                        "re-run with `--view-mode copy` to keep it as a copy, or "
                        "`--force` to replace it with a link"
                    )
                raise SyncError(
                    f"{view.relative.as_posix()} is a real directory with {len(entries)} entr(y/ies); "
                    "move it away or pass --force (skillkit never deletes unrecorded content)"
                )
            if entries:
                _remove_tree(path)
            else:
                path.rmdir()
        else:
            if not force:
                raise SyncError(f"{view.relative.as_posix()} exists and is not a directory")
            path.unlink()

    method = create_dir_link(path, canonical)
    if not points_to(path, canonical):
        raise SyncError(f"created {view.relative.as_posix()} but it does not resolve to {canonical}")
    state["views"][view.relative.as_posix()] = {
        "target": canonical.as_posix(),
        "method": method,
    }
    return f"created: {view.relative.as_posix()} -> {canonical} ({method}, {view.label})"


def _check_view(path: Path, canonical: Path, view: View, view_mode: str) -> str:
    if view_mode == MODE_COPY:
        if is_link_like(path):
            raise SyncError(
                f"{view.relative.as_posix()} is a link; a copy view must be a real directory "
                "(run `python .github/skills/_shared/skillkit/cli.py sync --view-mode copy`)"
            )
        if not path.is_dir():
            raise SyncError(
                f"{view.relative.as_posix()} is missing; run "
                "`python .github/skills/_shared/skillkit/cli.py sync --view-mode copy`"
            )
        diffs = tree_diff(canonical, path)
        if diffs:
            preview = ", ".join(diffs[:5]) + (" ..." if len(diffs) > 5 else "")
            raise SyncError(
                f"{view.relative.as_posix()} differs from {CANONICAL_REL.as_posix()} in "
                f"{len(diffs)} path(s): {preview}; run "
                "`python .github/skills/_shared/skillkit/cli.py sync --view-mode copy`"
            )
        return f"ok: {view.relative.as_posix()} == {canonical} (copy, {view.label})"

    if not path.exists():
        raise SyncError(
            f"{view.relative.as_posix()} is missing; run "
            "`python .github/skills/_shared/skillkit/cli.py sync`"
        )
    if not is_link_like(path):
        raise SyncError(
            f"{view.relative.as_posix()} is a real directory, not a link; "
            "skillkit refuses to treat it as a view"
        )
    if not points_to(path, canonical):
        raise SyncError(
            f"{view.relative.as_posix()} points to {path.resolve()}, not {canonical}"
        )
    return f"ok: {view.relative.as_posix()} -> {canonical} ({view.label})"


def remove_views(repo_root: Path, *, harness: str = "agents") -> list[str]:
    state = _load_state(repo_root)
    canonical = (repo_root / CANONICAL_REL).resolve()
    messages: list[str] = []
    for view in select_views(harness):
        path = repo_root / view.relative
        if not path.exists() and not is_link_like(path):
            messages.append(f"absent: {view.relative.as_posix()}")
            continue
        if is_link_like(path) and points_to(path, canonical):
            remove_dir_link(path)
            messages.append(f"removed: {view.relative.as_posix()}")
        elif state["views"].get(view.relative.as_posix()):
            if is_link_like(path):
                remove_dir_link(path)
            else:
                _remove_tree(path)
            messages.append(f"removed: {view.relative.as_posix()}")
        else:
            raise SyncError(
                f"{view.relative.as_posix()} is not a recorded skillkit view; refusing to delete"
            )
        state["views"].pop(view.relative.as_posix(), None)
        _prune_empty_parent(path)
    _save_state(repo_root, state)
    return messages


def _prune_empty_parent(view_path: Path) -> None:
    """Remove an empty .agents/.claude/.codex parent left after removing a view."""

    parent = view_path.parent
    if parent.name not in {".agents", ".claude", ".codex"}:
        return
    try:
        if parent.is_dir() and not any(parent.iterdir()):
            parent.rmdir()
    except OSError:
        pass


def status(repo_root: Path) -> str:
    canonical = (repo_root / CANONICAL_REL).resolve()
    lines = [
        f"canonical: {canonical}",
        "views (default: .agents; optional: .claude = Claude Code, .codex = Codex):",
    ]
    for view in VIEWS:
        path = repo_root / view.relative
        if not path.exists():
            lines.append(f"  {view.relative.as_posix():18} absent")
        elif is_link_like(path) and points_to(path, canonical):
            lines.append(f"  {view.relative.as_posix():18} ok -> {canonical}")
        elif is_link_like(path):
            lines.append(f"  {view.relative.as_posix():18} link -> {path.resolve()} (WRONG)")
        else:
            diffs = tree_diff(canonical, path)
            if diffs:
                lines.append(
                    f"  {view.relative.as_posix():18} copy, {len(diffs)} path(s) differ (run sync)"
                )
            else:
                lines.append(f"  {view.relative.as_posix():18} copy, identical to {canonical}")
    return "\n".join(lines)


def _load_state(repo_root: Path) -> dict:
    path = repo_root / STATE_REL
    if not path.is_file():
        return {"version": 1, "views": {}}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {"version": 1, "views": {}}
    if not isinstance(data, dict) or not isinstance(data.get("views"), dict):
        return {"version": 1, "views": {}}
    return data


def _save_state(repo_root: Path, state: dict) -> None:
    path = repo_root / STATE_REL
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _remove_tree(path: Path) -> None:
    for child in path.iterdir():
        if is_link_like(child):
            remove_dir_link(child)
        elif child.is_dir():
            _remove_tree(child)
        else:
            child.unlink()
    path.rmdir()


def _link_method(path: Path) -> str:
    if os.name == "nt" and not os.path.islink(path):
        return "junction"
    return "symlink"
