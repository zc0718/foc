"""Contract validation for the canonical Agent Skills tree.

The checks are deliberately opinionated: this repository wants *one* editable skill
body.  A harness view may be a link to it, or a copy that is byte-identical to it;
anything else is a second body and fails.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

from .model import (
    CANONICAL_REL,
    Manifest,
    Skill,
    load_manifest,
    load_skills,
)
from .paths import is_link_like, points_to
from .sync import tree_diff

NAME_RE = re.compile(r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
LINK_RE = re.compile(r"\[[^\]]*\]\(([^)]+)\)")
ROUTED_NAME_RE = re.compile(r"`(het-[a-z0-9-]+)`")

ALLOWED_KEYS = {
    "name",
    "description",
    "license",
    "compatibility",
    "metadata",
    "allowed-tools",
    "argument-hint",
    "user-invocable",
    "disable-model-invocation",
    "whenToUse",
    "paths",
}

HARNESS_ROOTS: tuple[tuple[str, str], ...] = (
    (".agents/skills", "DSH / Cursor / Gemini / Copilot"),
    (".claude/skills", "Claude Code"),
    (".codex/skills", "Codex"),
    (".dsh/skills", "DSH (rank 100)"),
    (".cursor/skills", "Cursor"),
    (".gemini/skills", "Gemini CLI"),
)

ROUTING_REL = Path(".github/skills/_shared/routing.md")


@dataclass
class ValidationResult:
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def error(self, message: str) -> None:
        self.errors.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)

    @property
    def ok(self) -> bool:
        return not self.errors

    def print_report(self, strict: bool = False) -> None:
        for item in self.warnings:
            print(f"warning: {item}")
        for item in self.errors:
            print(f"error: {item}")
        if self.ok and (not strict or not self.warnings):
            print(f"ok: {len(self.warnings)} warning(s), 0 error(s)")
        else:
            print(f"failed: {len(self.errors)} error(s), {len(self.warnings)} warning(s)")


def validate(
    repo_root: Path,
    *,
    strict: bool = False,
) -> ValidationResult:
    result = ValidationResult()
    try:
        skills = load_skills(repo_root)
        manifest = load_manifest(repo_root)
    except Exception as exc:  # noqa: BLE001 - one readable failure is better than a traceback
        result.error(str(exc))
        return result

    _validate_skills(skills, result)
    _validate_manifest(repo_root, manifest, skills, result)
    _validate_routing(repo_root, manifest, skills, result)
    _validate_no_duplicate_bodies(repo_root, result)

    # Strictness is a CLI policy: the caller decides whether warnings fail.
    return result


def _validate_skills(skills: Iterable[Skill], result: ValidationResult) -> None:
    seen: dict[str, Path] = {}
    for skill in skills:
        if skill.name in seen:
            result.error(f"duplicate skill name '{skill.name}': {seen[skill.name]} and {skill.path}")
        else:
            seen[skill.name] = skill.path

        if not NAME_RE.fullmatch(skill.name):
            result.error(
                f"{skill.path}: name '{skill.name}' must be kebab-case "
                "(lowercase letters, digits, single hyphens)"
            )
        if len(skill.name) > 64:
            result.error(f"{skill.path}: name exceeds 64 characters")
        if skill.directory.name != skill.name:
            result.error(
                f"{skill.path}: name '{skill.name}' must match its directory "
                f"'{skill.directory.name}'"
            )

        description = skill.data.get("description")
        if not isinstance(description, str) or not description.strip():
            result.error(f"{skill.path}: description is required")
        elif len(description) > 1024:
            result.error(f"{skill.path}: description exceeds 1024 characters")
        elif "use when" not in description.lower():
            result.warning(f"{skill.path}: description has no 'Use when' trigger phrase")

        if "argument-hint" in skill.data and not isinstance(skill.data["argument-hint"], str):
            result.error(f"{skill.path}: argument-hint must be a string")
        for boolean_key in ("user-invocable", "disable-model-invocation"):
            if boolean_key in skill.data and not isinstance(skill.data[boolean_key], bool):
                result.error(f"{skill.path}: {boolean_key} must be a boolean")
        if "license" in skill.data and not isinstance(skill.data["license"], str):
            result.error(f"{skill.path}: license must be a string")
        if "compatibility" in skill.data:
            compatibility = skill.data["compatibility"]
            if not isinstance(compatibility, str):
                result.error(f"{skill.path}: compatibility must be a string")
            elif len(compatibility) > 500:
                result.error(f"{skill.path}: compatibility exceeds 500 characters")

        unknown = sorted(set(skill.data) - ALLOWED_KEYS)
        for key in unknown:
            result.warning(f"{skill.path}: unknown frontmatter key '{key}' (harnesses may ignore it)")

        if not skill.body.strip():
            result.error(f"{skill.path}: body is empty")
        if skill.line_count > 800:
            result.error(f"{skill.path}: body is {skill.line_count} lines (limit 800)")
        elif skill.line_count > 500:
            result.warning(f"{skill.path}: body is {skill.line_count} lines (recommended <= 500)")

        _validate_relative_links(skill, result)


def _validate_relative_links(skill: Skill, result: ValidationResult) -> None:
    for raw_target in LINK_RE.findall(skill.body):
        target = raw_target.strip()
        if target.startswith("<") and target.endswith(">"):
            target = target[1:-1].strip()
        # Drop an optional Markdown title: [x](path "title")
        target = target.split(' "', 1)[0].strip()
        if not target or target.startswith(("#", "http://", "https://", "mailto:", "tel:")):
            continue
        target = target.split("#", 1)[0]
        if not target or target.startswith("/"):
            continue
        resolved = (skill.directory / target).resolve()
        if not resolved.exists():
            result.error(f"{skill.path}: relative link target does not exist: {target}")


def _validate_manifest(
    repo_root: Path,
    manifest: Manifest,
    skills: list[Skill],
    result: ValidationResult,
) -> None:
    by_name = {skill.name: skill for skill in skills}
    manifest_names: set[str] = set()
    manifest_ids: set[str] = set()

    canonical_root = CANONICAL_REL.as_posix()
    declared_root = manifest.data.get("canonical_root")
    if declared_root is not None and declared_root != canonical_root:
        result.error(f"{manifest.path}: canonical_root must be '{canonical_root}'")

    families = manifest.data.get("family", {})
    if not isinstance(families, dict):
        result.error(f"{manifest.path}: 'family' must be an object")
        families = {}

    project = manifest.data.get("project")
    metadata_path = repo_root / "metadata.json"
    if metadata_path.is_file():
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            metadata = {}
        if project and metadata.get("name") and project != metadata["name"]:
            result.error(
                f"{manifest.path}: project '{project}' does not match metadata.json name "
                f"'{metadata['name']}'"
            )

    for entry in manifest.skills:
        if not isinstance(entry, dict):
            result.error(f"{manifest.path}: every skills[] entry must be an object")
            continue
        name = entry.get("name")
        if not isinstance(name, str) or not name:
            result.error(f"{manifest.path}: skills[] entry without a name")
            continue
        if name in manifest_names:
            result.error(f"{manifest.path}: duplicate skill name in manifest: {name}")
        manifest_names.add(name)

        skill_id = entry.get("id")
        if not isinstance(skill_id, str) or not skill_id:
            result.error(f"{manifest.path}: skill '{name}' has no id")
        elif skill_id in manifest_ids:
            result.error(f"{manifest.path}: duplicate skill id: {skill_id}")
        else:
            manifest_ids.add(skill_id)

        family = entry.get("family")
        if not isinstance(family, str) or not family:
            result.error(f"{manifest.path}: skill '{name}' has no family")
        elif family not in families:
            result.error(f"{manifest.path}: skill '{name}' uses unknown family '{family}'")

        intents = entry.get("intents")
        if not isinstance(intents, list) or not intents:
            result.error(f"{manifest.path}: skill '{name}' has no intents")
        elif not all(isinstance(intent, str) and intent for intent in intents):
            result.error(f"{manifest.path}: skill '{name}' has a non-string intent")

        if not isinstance(entry.get("read_only"), bool):
            result.error(f"{manifest.path}: skill '{name}' read_only must be a boolean")

        dependencies = entry.get("dependencies", [])
        if not isinstance(dependencies, list):
            result.error(f"{manifest.path}: skill '{name}' dependencies must be a list")
        else:
            for dependency in dependencies:
                if dependency not in by_name:
                    result.error(
                        f"{manifest.path}: skill '{name}' depends on unknown skill '{dependency}'"
                    )

        expected = f"{canonical_root}/{name}/SKILL.md"
        declared_path = entry.get("path")
        if declared_path != expected:
            result.error(
                f"{manifest.path}: skill '{name}' path must be '{expected}' "
                f"(found '{declared_path}')"
            )
        elif name not in by_name:
            result.error(f"{manifest.path}: skill '{name}' has no SKILL.md at '{expected}'")

    for name in sorted(set(by_name) - manifest_names - {"het-agent"}):
        result.error(f"{manifest.path}: canonical skill '{name}' is missing from manifest.skills")

    agent = manifest.agent
    agent_name = agent.get("name")
    if agent_name != "het-agent":
        result.error(f"{manifest.path}: agent.name must be 'het-agent'")
    agent_path = agent.get("path")
    expected_agent_path = f"{canonical_root}/het-agent/SKILL.md"
    if agent_path != expected_agent_path:
        result.error(f"{manifest.path}: agent.path must be '{expected_agent_path}'")
    if not (repo_root / expected_agent_path).is_file():
        result.error(f"{manifest.path}: agent path does not exist: {expected_agent_path}")
    if "het-agent" not in by_name:
        result.error(f"{manifest.path}: het-agent/SKILL.md is missing from the canonical tree")
    if "het-agent" in manifest_names:
        result.error(f"{manifest.path}: het-agent must live under 'agent', not 'skills'")

    routing_table = manifest.data.get("routing_table")
    if routing_table is not None:
        if routing_table != ROUTING_REL.as_posix():
            result.error(f"{manifest.path}: routing_table must be '{ROUTING_REL.as_posix()}'")
        elif not (repo_root / ROUTING_REL).is_file():
            result.error(f"{manifest.path}: routing_table file does not exist: {routing_table}")

    generated_views = manifest.data.get("generated_views")
    if generated_views is not None:
        if not isinstance(generated_views, dict):
            result.error(f"{manifest.path}: generated_views must be an object")
        else:
            known_views = {rel for rel, _ in HARNESS_ROOTS}
            for rel, harnesses in generated_views.items():
                if rel not in known_views:
                    result.error(f"{manifest.path}: unknown generated view '{rel}'")
                if not isinstance(harnesses, list) or not all(
                    isinstance(harness, str) and harness for harness in harnesses
                ):
                    result.error(f"{manifest.path}: generated_views['{rel}'] must be a string list")

    default_view = manifest.data.get("default_view")
    if default_view is not None:
        known_views = {rel for rel, _ in HARNESS_ROOTS}
        if default_view not in known_views:
            result.error(f"{manifest.path}: unknown default_view '{default_view}'")


def _validate_routing(
    repo_root: Path,
    manifest: Manifest,
    skills: list[Skill],
    result: ValidationResult,
) -> None:
    routing_path = repo_root / ROUTING_REL
    if not routing_path.is_file():
        result.warning(f"{routing_path}: routing table is missing")
        return
    routed = set(ROUTED_NAME_RE.findall(routing_path.read_text(encoding="utf-8")))
    known = {skill.name for skill in skills}
    manifest_names = {entry.get("name") for entry in manifest.skills}
    for name in sorted(routed - known):
        result.error(f"{routing_path}: routes to unknown skill '{name}'")
    for name in sorted(manifest_names - routed):
        result.warning(f"{routing_path}: manifest skill '{name}' is not in the routing table")
    if "het-agent" not in routed:
        result.warning(f"{routing_path}: routing table does not mention the router 'het-agent'")


def _validate_no_duplicate_bodies(repo_root: Path, result: ValidationResult) -> None:
    canonical = (repo_root / CANONICAL_REL).resolve()
    for relative, label in HARNESS_ROOTS:
        path = repo_root / relative
        if not path.exists():
            continue
        if is_link_like(path):
            if not points_to(path, canonical):
                result.error(f"{relative}: link points to {path.resolve()}, not {canonical}")
            continue
        if path.is_file():
            result.error(f"{relative}: expected a directory link, found a file")
            continue
        if not path.is_dir():
            continue
        entries = list(path.iterdir())
        if not entries:
            result.warning(f"{relative}: empty view; run `skillkit sync`")
            continue
        diffs = tree_diff(canonical, path)
        if diffs:
            preview = ", ".join(diffs[:5]) + (" ..." if len(diffs) > 5 else "")
            result.error(
                f"{relative}: copy view differs from {CANONICAL_REL.as_posix()} in "
                f"{len(diffs)} path(s): {preview}; edit the canonical tree and run "
                "`skillkit sync --view-mode copy`"
            )

    for base in (".agent", ".agents", ".dsh", ".claude", ".codex", ".cursor", ".gemini"):
        for candidate in (
            repo_root / base / "het-agent" / "SKILL.md",
            repo_root / base / "skills" / "het-agent" / "SKILL.md",
            repo_root / base / "agents" / "het-agent" / "SKILL.md",
        ):
            if not candidate.is_file() or points_to(candidate, canonical / "het-agent" / "SKILL.md"):
                continue
            # Inside a declared view root, a byte-identical mirror is a view, not a second body.
            if _in_view_root(repo_root, candidate) and _same_bytes(
                candidate, canonical / "het-agent" / "SKILL.md"
            ):
                continue
            result.error(
                f"{candidate.relative_to(repo_root).as_posix()}: second agent body; "
                "het-agent must exist only under .github/skills"
            )


def _in_view_root(repo_root: Path, path: Path) -> bool:
    """True when ``path`` sits under a path declared in HARNESS_ROOTS."""

    view_parts = {Path(relative).parts for relative, _ in HARNESS_ROOTS}
    try:
        relative = path.relative_to(repo_root)
    except ValueError:
        return False
    for depth in range(1, len(relative.parts) + 1):
        if relative.parts[:depth] in view_parts:
            return True
    return False


def _same_bytes(left: Path, right: Path) -> bool:
    try:
        return left.read_bytes() == right.read_bytes()
    except OSError:
        return False
