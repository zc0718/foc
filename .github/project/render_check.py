#!/usr/bin/env python3
"""Every page the site publishes that is *rendered* from a registry is current.

Why this check is not in the gates that own those registries. A gate whose subject is the code has to
run with no documentation tree at all: delete `docs/sphinx/` and `gates.py deps` must still answer
"where may this package appear". Before this split it did not -- `deps` failed with
`third-party-vendored.rst: missing`, which is the rule this project states about itself broken in the
direction that matters: a module may not need a sibling module's artefact to run.

So the ownership is split by what each layer can answer:

* `symbols_gate.py` and `deps_gate.py` own the registries, and can write the page that renders them
  (`--render`, with `--out` for a caller that does not want the tree touched);
* this script asks whether the committed page still equals a fresh render. The subject is the
  documentation, so it is asked from the layer whose subject is the documentation -- the `docs` gate.

A page that is out of date is not a broken link: it is a second source of truth, which is the failure
the registries were created to prevent in the first place.

    python3 .github/project/render_check.py             # what the `docs` gate runs
    python3 .github/project/render_check.py --selftest  # prove a one-line drift is caught
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = ROOT / '.github' / 'project'

# (the script that owns the registry, the page its `--render` writes)
PAGES = (
    ('symbols_gate.py', ROOT / 'docs' / 'sphinx' / 'source' / 'symbols.rst'),
    ('deps_gate.py', ROOT / 'docs' / 'sphinx' / 'source' / 'third-party.rst'),
)


def fresh(script: str) -> str | None:
    """Render the page into a temp directory. `None` when the renderer itself failed."""
    with tempfile.TemporaryDirectory(prefix='foc-render-') as workdir:
        out = Path(workdir) / 'page.rst'
        done = subprocess.run([sys.executable, str(HERE / script), '--render', '--out', str(out)],
                              cwd=ROOT, text=True, capture_output=True)
        if done.returncode != 0:
            print(f'[render] {script} --render failed:\n{done.stderr.strip()}', flush=True)
            return None
        return out.read_text(encoding='utf-8')


def judge(script: str, page: Path, rendered: str | None) -> str:
    """One verdict per page, as a string so the selftest can call it with a mutation."""
    try:
        rel = page.relative_to(ROOT).as_posix()
    except ValueError:
        # The selftest hands this function a page in a temp directory; a path outside the repository
        # is not an error, it just has no repository-relative form to print.
        rel = page.as_posix()
    if rendered is None:
        return f'{rel}: {script} --render failed'
    if not page.exists():
        return f'{rel}: missing, run {script} --render'
    if page.read_text(encoding='utf-8') != rendered:
        return f'{rel}: stale, run {script} --render'
    return ''


def selftest() -> int:
    """A page that differs by one byte must be caught, and an identical page must not."""
    sample = 'a page\n'
    caught = 0
    with tempfile.TemporaryDirectory(prefix='foc-render-selftest-') as workdir:
        page = Path(workdir) / 'page.rst'
        page.write_text(sample, encoding='utf-8')
        if judge('x.py', page, sample):
            print('[render-selftest] identical page reported as a problem -> NOT CAUGHT', file=sys.stderr)
        else:
            caught += 1
            print('[render-selftest] ok   an identical page is clean')
        page.write_text(sample + ' ', encoding='utf-8')
        if not judge('x.py', page, sample):
            print('[render-selftest] one trailing space not caught', file=sys.stderr)
        else:
            caught += 1
            print('[render-selftest] ok   one trailing space is stale')
        page.unlink()
        if not judge('x.py', page, sample):
            print('[render-selftest] a missing page not caught', file=sys.stderr)
        else:
            caught += 1
            print('[render-selftest] ok   a missing page is a problem')
        if not judge('x.py', page, None):
            print('[render-selftest] a failed render not caught', file=sys.stderr)
        else:
            caught += 1
            print('[render-selftest] ok   a failed render is a problem')
    return 0 if caught == 4 else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--selftest', action='store_true', help='prove a drifted page is still caught')
    args = parser.parse_args()
    if args.selftest:
        return selftest()

    problems = []
    for script, page in PAGES:
        verdict = judge(script, page, fresh(script))
        if verdict:
            problems.append(verdict)
    if problems:
        for line in problems:
            print(f'[render] {line}', flush=True)
        print(f'[render] FAIL ({len(problems)} problem(s) over {len(PAGES)} page(s))', flush=True)
        return 1
    print(f'[render] OK ({len(PAGES)} rendered page(s) match their registries)', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
