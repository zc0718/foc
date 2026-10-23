"""Command line entry point for skillkit."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if __package__ in (None, ""):
    # Support `python .github/skills/_shared/skillkit/cli.py` without installing.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from skillkit.catalog import render_compact, render_json, render_markdown
    from skillkit.model import find_repo_root, load_manifest, load_skills
    from skillkit.sync import SyncError, remove_views, status, sync_views
    from skillkit.validate import validate
else:
    from .catalog import render_compact, render_json, render_markdown
    from .model import find_repo_root, load_manifest, load_skills
    from .sync import SyncError, remove_views, status, sync_views
    from .validate import validate


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="skillkit",
        description="Validate and materialize the fcpp Agent Skills tree for multiple harnesses.",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=None,
        help="repository root (default: nearest ancestor with .git or metadata.json)",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    validate_parser = subparsers.add_parser("validate", help="validate skills, manifest and views")
    validate_parser.add_argument(
        "--strict", action="store_true", help="treat warnings as a failure"
    )
    validate_parser.add_argument(
        "--json", action="store_true", dest="as_json", help="emit the result as JSON"
    )

    sync_parser = subparsers.add_parser("sync", help="create or verify harness views")
    sync_parser.add_argument(
        "--harness",
        default="agents",
        help="agents (default: DSH/Cursor/Gemini/Copilot) | claude | codex | all",
    )
    sync_parser.add_argument(
        "--check", action="store_true", help="verify views without creating them"
    )
    sync_parser.add_argument(
        "--view-mode",
        choices=("link", "copy"),
        default="link",
        help="link = symlink/junction (default); copy = byte-identical mirror (for committed views)",
    )
    sync_parser.add_argument(
        "--force",
        action="store_true",
        help="replace an unrecorded real directory or link (destructive; use with care)",
    )
    sync_parser.add_argument(
        "--remove", action="store_true", help="remove generated views only"
    )

    subparsers.add_parser("status", help="show canonical root and view status")

    catalog_parser = subparsers.add_parser("catalog", help="print a harness-neutral catalog")
    catalog_parser.add_argument(
        "--format",
        choices=("markdown", "json", "compact"),
        default="markdown",
        help="output format (default: markdown)",
    )

    return parser


def main(argv: list[str] | None = None) -> int:
    # Keep bilingual output stable when the console code page is not UTF-8.
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            try:
                reconfigure(encoding="utf-8")
            except (OSError, ValueError):
                pass
    args = build_parser().parse_args(argv)
    repo_root = (args.root or find_repo_root(Path(__file__).resolve())).resolve()

    try:
        if args.command == "validate":
            return _run_validate(repo_root, args)
        if args.command == "sync":
            return _run_sync(repo_root, args)
        if args.command == "status":
            print(status(repo_root))
            return 0
        if args.command == "catalog":
            return _run_catalog(repo_root, args)
    except (SyncError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 2


def _run_validate(repo_root: Path, args: argparse.Namespace) -> int:
    result = validate(repo_root, strict=args.strict)
    if args.as_json:
        import json

        print(json.dumps({"errors": result.errors, "warnings": result.warnings}, indent=2))
    else:
        result.print_report(strict=args.strict)
    if result.errors:
        return 1
    if args.strict and result.warnings:
        return 1
    return 0


def _run_sync(repo_root: Path, args: argparse.Namespace) -> int:
    if args.remove:
        for message in remove_views(repo_root, harness=args.harness):
            print(message)
        return 0
    messages = sync_views(
        repo_root,
        harness=args.harness,
        check=args.check,
        force=args.force,
        view_mode=args.view_mode,
    )
    for message in messages:
        print(message)
    return 0


def _run_catalog(repo_root: Path, args: argparse.Namespace) -> int:
    skills = load_skills(repo_root)
    manifest = load_manifest(repo_root)
    if args.format == "json":
        print(render_json(skills, manifest), end="")
    elif args.format == "compact":
        print(render_compact(skills), end="")
    else:
        print(render_markdown(skills, manifest), end="")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
