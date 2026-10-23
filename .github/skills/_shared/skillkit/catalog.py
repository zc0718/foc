"""Render a harness-neutral catalog of the canonical skills."""

from __future__ import annotations

import json
from pathlib import Path

from .model import Manifest, Skill


def _by_name(manifest: Manifest) -> dict[str, dict]:
    return {
        entry["name"]: entry
        for entry in manifest.skills
        if isinstance(entry, dict) and isinstance(entry.get("name"), str)
    }


def _entry_for(manifest: Manifest, by_name: dict[str, dict], skill: Skill) -> dict:
    entry = by_name.get(skill.name)
    if entry is not None:
        return entry
    if skill.name == manifest.agent.get("name"):
        return {
            "id": manifest.agent.get("id", "-"),
            "family": "agent",
            "read_only": False,
        }
    return {}


def render_markdown(skills: list[Skill], manifest: Manifest) -> str:
    by_name = _by_name(manifest)
    families = manifest.data.get("family", {})
    lines = [
        "| Skill | ID | Family | Description | Path |",
        "|---|---|---|---|---|",
    ]
    for skill in skills:
        entry = _entry_for(manifest, by_name, skill)
        family = entry.get("family", "-")
        family_label = families.get(family, family) if isinstance(family, str) else family
        skill_id = entry.get("id", "-")
        description = skill.description.replace("|", "\\|")
        lines.append(
            f"| `{skill.name}` | {skill_id} | {family_label} | {description} | "
            f"`{skill.relative_path}` |"
        )
    return "\n".join(lines) + "\n"


def render_json(skills: list[Skill], manifest: Manifest) -> str:
    by_name = _by_name(manifest)
    entries = []
    for skill in skills:
        entry = _entry_for(manifest, by_name, skill)
        entries.append(
            {
                "name": skill.name,
                "description": skill.description,
                "path": skill.relative_path,
                "id": entry.get("id"),
                "family": entry.get("family"),
                "read_only": entry.get("read_only"),
            }
        )
    payload = {
        "canonical_root": ".github/skills",
        "project": manifest.data.get("project"),
        "skills": entries,
        "agent": manifest.agent.get("name"),
    }
    return json.dumps(payload, ensure_ascii=False, indent=2) + "\n"


def render_compact(skills: list[Skill]) -> str:
    return "\n".join(f"- `{skill.name}`: {skill.description}" for skill in skills) + "\n"
