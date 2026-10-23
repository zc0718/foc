"""Load the canonical skill tree and its manifest."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .frontmatter import FrontmatterError, parse_skill_file

CANONICAL_REL = Path(".github/skills")
MANIFEST_NAME = "manifest.json"


class SkillLoadError(RuntimeError):
    """Raised when the canonical tree cannot be loaded."""


@dataclass
class Skill:
    name: str
    directory: Path
    path: Path
    data: dict[str, Any]
    body: str
    raw: str
    relative: str = ""

    @property
    def description(self) -> str:
        value = self.data.get("description", "")
        return value if isinstance(value, str) else str(value)

    @property
    def relative_path(self) -> str:
        return self.relative or self.path.as_posix()

    @property
    def line_count(self) -> int:
        return len(self.body.splitlines())


@dataclass
class Manifest:
    path: Path
    data: dict[str, Any] = field(default_factory=dict)

    @property
    def skills(self) -> list[dict[str, Any]]:
        value = self.data.get("skills", [])
        return value if isinstance(value, list) else []

    @property
    def agent(self) -> dict[str, Any]:
        value = self.data.get("agent", {})
        return value if isinstance(value, dict) else {}


def find_repo_root(start: Path) -> Path:
    """Nearest ancestor with ``.git`` or ``metadata.json``; else ``start``."""

    current = start.resolve()
    for candidate in (current, *current.parents):
        if (candidate / ".git").exists() or (candidate / "metadata.json").exists():
            return candidate
    return current


def canonical_root(repo_root: Path) -> Path:
    return repo_root / CANONICAL_REL


def load_skills(repo_root: Path) -> list[Skill]:
    root = canonical_root(repo_root)
    if not root.is_dir():
        raise SkillLoadError(f"canonical skill root not found: {root}")

    skills: list[Skill] = []
    for directory in sorted(root.iterdir(), key=lambda item: item.name):
        if not directory.is_dir() or directory.name.startswith("_"):
            continue
        skill_path = directory / "SKILL.md"
        if not skill_path.is_file():
            continue
        try:
            parsed = parse_skill_file(skill_path.read_text(encoding="utf-8"))
        except FrontmatterError as exc:
            raise SkillLoadError(f"{skill_path}: {exc}") from exc
        name = parsed.data.get("name")
        if not isinstance(name, str) or not name:
            raise SkillLoadError(f"{skill_path}: frontmatter 'name' is required")
        skills.append(
            Skill(
                name=name,
                directory=directory,
                path=skill_path,
                data=parsed.data,
                body=parsed.body,
                raw=parsed.raw,
                relative=skill_path.relative_to(repo_root).as_posix(),
            )
        )
    if not skills:
        raise SkillLoadError(f"no SKILL.md files found under {root}")
    return skills


def load_manifest(repo_root: Path) -> Manifest:
    path = canonical_root(repo_root) / MANIFEST_NAME
    if not path.is_file():
        raise SkillLoadError(f"manifest not found: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SkillLoadError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise SkillLoadError(f"{path}: manifest root must be an object")
    return Manifest(path=path, data=data)
