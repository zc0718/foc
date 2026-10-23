"""Read YAML frontmatter from an Agent Skills ``SKILL.md`` file.

PyYAML is used when it is installed.  The fallback parser covers the flat
``key: value`` subset this repository uses, which keeps ``skillkit`` runnable in
a bare Python 3.10+ environment.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from typing import Any

_FRONTMATTER_RE = re.compile(r"\A---[ \t]*\r?\n(.*?)\r?\n---[ \t]*(?:\r?\n|\Z)(.*)\Z", re.DOTALL)


class FrontmatterError(ValueError):
    """Raised when a skill file has no parseable frontmatter."""


@dataclass(frozen=True)
class ParsedSkillFile:
    data: dict[str, Any]
    body: str
    raw: str


def parse_skill_file(text: str) -> ParsedSkillFile:
    """Split ``text`` into frontmatter data and Markdown body."""

    if text.startswith("\ufeff"):
        text = text[1:]
    match = _FRONTMATTER_RE.match(text)
    if not match:
        raise FrontmatterError("missing YAML frontmatter delimited by '---' lines")
    data = _parse_yaml(match.group(1))
    return ParsedSkillFile(data=data, body=match.group(2), raw=text)


def _parse_yaml(frontmatter: str) -> dict[str, Any]:
    try:
        import yaml  # type: ignore[import-not-found]
    except Exception:  # pragma: no cover - exercised when PyYAML is absent
        return _parse_flat(frontmatter)

    try:
        data = yaml.safe_load(frontmatter)
    except Exception as exc:  # noqa: BLE001 - surface PyYAML's message verbatim
        raise FrontmatterError(f"invalid YAML frontmatter: {exc}") from exc
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise FrontmatterError("frontmatter must be a YAML mapping")
    return data


def _parse_flat(frontmatter: str) -> dict[str, Any]:
    data: dict[str, Any] = {}
    for lineno, line in enumerate(frontmatter.splitlines(), start=1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if line[:1].isspace():
            raise FrontmatterError(
                "nested YAML requires PyYAML; install PyYAML or keep the frontmatter flat "
                f"(line {lineno})"
            )
        if ":" not in line:
            raise FrontmatterError(f"expected 'key: value' (line {lineno})")
        key, raw_value = line.split(":", 1)
        key = key.strip()
        if not key:
            raise FrontmatterError(f"empty frontmatter key (line {lineno})")
        data[key] = _scalar(raw_value.strip())
    return data


def _scalar(raw: str) -> Any:
    if raw == "":
        return None
    if len(raw) >= 2 and raw[0] == "'" and raw[-1] == "'":
        return raw[1:-1].replace("''", "'")
    if len(raw) >= 2 and raw[0] == '"' and raw[-1] == '"':
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return raw[1:-1]
    lowered = raw.lower()
    if lowered in {"true", "yes", "on"}:
        return True
    if lowered in {"false", "no", "off"}:
        return False
    if lowered in {"null", "~"}:
        return None
    return raw
