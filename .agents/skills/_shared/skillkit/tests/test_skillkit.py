"""Unit tests for skillkit.  Run with:

    python -m unittest discover -s .github/skills/_shared/skillkit/tests -v
"""

from __future__ import annotations

import shutil
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from skillkit.frontmatter import FrontmatterError, parse_skill_file  # noqa: E402
from skillkit.model import find_repo_root  # noqa: E402
from skillkit.paths import is_link_like, points_to  # noqa: E402
from skillkit.sync import SyncError, remove_views, sync_views  # noqa: E402
from skillkit.validate import (  # noqa: E402
    ValidationResult,
    _validate_no_duplicate_bodies,
    validate,
)


class FrontmatterTests(unittest.TestCase):
    def test_quoted_values_and_booleans(self) -> None:
        parsed = parse_skill_file(
            "---\n"
            "name: demo-skill\n"
            "description: 'Use when: it is a demo; don''t panic.'\n"
            "argument-hint: \"scope (optional)\"\n"
            "user-invocable: true\n"
            "---\n"
            "\n"
            "# Body\n"
        )
        self.assertEqual(parsed.data["name"], "demo-skill")
        self.assertEqual(parsed.data["description"], "Use when: it is a demo; don't panic.")
        self.assertEqual(parsed.data["argument-hint"], "scope (optional)")
        self.assertIs(parsed.data["user-invocable"], True)
        self.assertIn("# Body", parsed.body)

    def test_missing_frontmatter(self) -> None:
        with self.assertRaises(FrontmatterError):
            parse_skill_file("# no frontmatter\n")

    def test_flat_fallback(self) -> None:
        from skillkit.frontmatter import _parse_flat

        data = _parse_flat("name: demo\ndescription: 'a: b'\nuser-invocable: false\n")
        self.assertEqual(data["name"], "demo")
        self.assertEqual(data["description"], "a: b")
        self.assertIs(data["user-invocable"], False)


class RepositoryContractTests(unittest.TestCase):
    def test_repository_is_valid(self) -> None:
        root = find_repo_root(Path(__file__).resolve())
        result = validate(root)
        self.assertEqual(result.errors, [])
        self.assertEqual(result.warnings, [])


class DuplicateBodyTests(unittest.TestCase):
    def _base(self, name: str) -> Path:
        repository = find_repo_root(Path(__file__).resolve())
        base = repository / "workspace" / name
        if base.exists():
            shutil.rmtree(base, ignore_errors=True)
        base.mkdir(parents=True)
        return base

    def test_diverging_copy_in_a_view_is_rejected(self) -> None:
        base = self._base("_skillkit_dup_view")
        try:
            canonical = base / ".github" / "skills" / "demo" / "SKILL.md"
            canonical.parent.mkdir(parents=True)
            canonical.write_text(
                "---\nname: demo\ndescription: 'Use when testing duplicates.'\n---\n\n# Demo\n",
                encoding="utf-8",
            )
            copy = base / ".claude" / "skills" / "demo" / "SKILL.md"
            copy.parent.mkdir(parents=True)
            copy.write_text(
                canonical.read_text(encoding="utf-8") + "\nlocal edit\n", encoding="utf-8"
            )

            result = ValidationResult()
            _validate_no_duplicate_bodies(base, result)
            self.assertTrue(result.errors)
            self.assertIn("differs from", " ".join(result.errors))
        finally:
            shutil.rmtree(base, ignore_errors=True)

    def test_identical_copy_in_a_view_is_accepted(self) -> None:
        base = self._base("_skillkit_mirror_view")
        try:
            canonical = base / ".github" / "skills" / "demo" / "SKILL.md"
            canonical.parent.mkdir(parents=True)
            canonical.write_text(
                "---\nname: demo\ndescription: 'Use when testing mirrors.'\n---\n\n# Demo\n",
                encoding="utf-8",
            )
            copy = base / ".agents" / "skills" / "demo" / "SKILL.md"
            copy.parent.mkdir(parents=True)
            copy.write_text(canonical.read_text(encoding="utf-8"), encoding="utf-8")

            result = ValidationResult()
            _validate_no_duplicate_bodies(base, result)
            self.assertEqual(result.errors, [])
        finally:
            shutil.rmtree(base, ignore_errors=True)

    def test_second_agent_body_is_rejected(self) -> None:
        base = self._base("_skillkit_dup_agent")
        try:
            canonical = base / ".github" / "skills" / "het-agent" / "SKILL.md"
            canonical.parent.mkdir(parents=True)
            canonical.write_text(
                "---\nname: het-agent\ndescription: 'Use when testing the router.'\n---\n\n# Router\n",
                encoding="utf-8",
            )
            second = base / ".agents" / "het-agent" / "SKILL.md"
            second.parent.mkdir(parents=True)
            second.write_text(canonical.read_text(encoding="utf-8"), encoding="utf-8")

            result = ValidationResult()
            _validate_no_duplicate_bodies(base, result)
            self.assertTrue(result.errors)
            self.assertIn("second agent body", " ".join(result.errors))
        finally:
            shutil.rmtree(base, ignore_errors=True)


class SyncTests(unittest.TestCase):
    def test_sync_check_and_remove(self) -> None:
        repository = find_repo_root(Path(__file__).resolve())
        base = repository / "workspace" / "_skillkit_test"
        if base.exists():
            shutil.rmtree(base, ignore_errors=True)
        root = base / "repo"
        root.mkdir(parents=True)
        try:
            (root / "metadata.json").write_text('{"name": "demo"}\n', encoding="utf-8")
            skill = root / ".github" / "skills" / "demo" / "SKILL.md"
            skill.parent.mkdir(parents=True)
            skill.write_text(
                "---\nname: demo\ndescription: 'Use when testing sync.'\n---\n\n# Demo\n",
                encoding="utf-8",
            )

            messages = sync_views(root)
            self.assertEqual(len(messages), 1)
            view = root / ".agents/skills"
            self.assertTrue(is_link_like(view))
            self.assertTrue(points_to(view, root / ".github" / "skills"))
            self.assertTrue((view / "demo" / "SKILL.md").is_file())
            self.assertEqual(len(sync_views(root, check=True)), 1)

            messages = sync_views(root, harness="all")
            self.assertEqual(len(messages), 3)  # agents is idempotent; claude + codex are new
            for relative in (".agents/skills", ".claude/skills", ".codex/skills"):
                view = root / relative
                self.assertTrue(is_link_like(view), relative)
                self.assertTrue(points_to(view, root / ".github" / "skills"), relative)
                self.assertTrue((view / "demo" / "SKILL.md").is_file(), relative)

            self.assertEqual(len(sync_views(root, check=True, harness="all")), 3)
            remove_views(root, harness="all")
            for relative in (".agents/skills", ".claude/skills", ".codex/skills"):
                self.assertFalse((root / relative).exists(), relative)
        finally:
            # Never let rmtree walk into a junction.
            try:
                remove_views(root, harness="all")
            except Exception:  # noqa: BLE001 - cleanup best effort
                pass
            shutil.rmtree(base, ignore_errors=True)

    def test_copy_view_is_materialized_and_survives_check(self) -> None:
        repository = find_repo_root(Path(__file__).resolve())
        base = repository / "workspace" / "_skillkit_copy_test"
        if base.exists():
            shutil.rmtree(base, ignore_errors=True)
        root = base / "repo"
        root.mkdir(parents=True)
        try:
            (root / "metadata.json").write_text('{"name": "demo"}\n', encoding="utf-8")
            skill = root / ".github" / "skills" / "demo" / "SKILL.md"
            skill.parent.mkdir(parents=True)
            skill.write_text(
                "---\nname: demo\ndescription: 'Use when testing copies.'\n---\n\n# Demo\n",
                encoding="utf-8",
            )
            cached = root / ".github" / "skills" / "demo" / "__pycache__" / "x.pyc"
            cached.parent.mkdir(parents=True)
            cached.write_text("binary", encoding="utf-8")

            messages = sync_views(root, view_mode="copy")
            self.assertEqual(len(messages), 1)
            view = root / ".agents/skills"
            self.assertTrue(view.is_dir())
            self.assertFalse(is_link_like(view))
            self.assertEqual(
                (view / "demo" / "SKILL.md").read_text(encoding="utf-8"),
                skill.read_text(encoding="utf-8"),
            )
            self.assertFalse((view / "demo" / "__pycache__").exists())

            # Idempotent, and a fresh clone passes --check.
            self.assertIn("ok:", sync_views(root, view_mode="copy")[0])
            self.assertIn("ok:", sync_views(root, check=True, view_mode="copy")[0])

            # Any local edit is drift, not a second body.
            (view / "demo" / "SKILL.md").write_text("tampered\n", encoding="utf-8")
            with self.assertRaises(SyncError):
                sync_views(root, check=True, view_mode="copy")

            sync_views(root, view_mode="copy")
            self.assertIn("ok:", sync_views(root, check=True, view_mode="copy")[0])
        finally:
            shutil.rmtree(base, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
