"""Filesystem link helpers that work on POSIX and Windows without privileges.

POSIX uses relative symlinks.  Windows first tries a symlink (Developer Mode or an
elevated shell) and falls back to a directory junction, which every user can create
on NTFS.
"""

from __future__ import annotations

import os
import stat
import subprocess
from pathlib import Path

_REPARSE_POINT = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)


def is_link_like(path: Path) -> bool:
    """True for a symlink or a Windows junction/reparse point."""

    if os.path.islink(path):
        return True
    if os.name != "nt":
        return False
    try:
        attributes = os.lstat(path).st_file_attributes  # type: ignore[attr-defined]
    except (AttributeError, OSError):
        return False
    return bool(attributes & _REPARSE_POINT)


def points_to(link: Path, target: Path) -> bool:
    """True when ``link`` resolves to ``target`` (symlink, junction or same path)."""

    try:
        if link.resolve() == target.resolve():
            return True
        return os.path.samefile(link, target)
    except OSError:
        return False


def create_dir_link(link: Path, target: Path) -> str:
    """Create ``link`` -> ``target`` and return the mechanism used."""

    link.parent.mkdir(parents=True, exist_ok=True)
    absolute_target = target.resolve()
    if os.name != "nt":
        relative = os.path.relpath(absolute_target, link.parent)
        os.symlink(relative, link, target_is_directory=True)
        return "symlink"

    try:
        os.symlink(absolute_target, link, target_is_directory=True)
        return "symlink"
    except OSError:
        # Junction: no administrator or Developer Mode required.
        _run_cmd(["mklink", "/J", str(link), str(absolute_target)])
        return "junction"


def remove_dir_link(link: Path) -> None:
    """Remove a symlink or junction without touching the target."""

    if os.name == "nt" and not os.path.islink(link):
        _run_cmd(["rmdir", str(link)])
        return
    link.unlink()


def _run_cmd(arguments: list[str]) -> None:
    # cmd builtins (mklink/rmdir) need cmd.exe.  Output is deliberately discarded:
    # the Windows sandbox does not allow every child process to own a pipe.
    result = subprocess.run(
        ["cmd", "/c", *arguments],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode != 0:
        raise OSError(
            f"command failed with exit code {result.returncode}: {' '.join(arguments)}"
        )
