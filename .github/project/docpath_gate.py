"""Every command a reproduction page names must point at a file that exists."""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PAGES = [ROOT / "docs" / "sphinx" / "source" / "reproducing.rst"]
COMMAND = re.compile(r"python3\s+(\.github/[A-Za-z0-9_./-]+)")
# A gate name is only a name: `gates.py demoX` is an argparse error, not a red test, and a reader who meets one
# concludes their checkout is broken. The list on the page is also a promise about coverage, so a gate that exists
# and is not on it is a page that has quietly stopped being an inventory.
# 门禁名字只是个名字：`gates.py demoX` 是 argparse 报错而不是红测试，读者遇到只会以为自己的检出坏了。页面上的名单同时是
# 一条覆盖承诺，所以"门存在却不在名单上"就是这一页悄悄不再是一份清单。
GATE = re.compile(r"gates\.py\s+([a-z][a-z-]*(?:\s+[a-z][a-z-]*)*)")


def gate_names() -> tuple[set[str], set[str]]:
    """The project's gates and its local-only ones, read from the dispatcher rather than imported."""
    source = (ROOT / ".github" / "project" / "gates.py").read_text(encoding="utf-8", errors="replace")
    listed = re.search(r"PROJECT_GATES\s*=\s*\(([^)]*)\)", source)
    names = set(re.findall(r"'([a-z][a-z-]*)'", listed.group(1))) if listed else set()
    local = re.search(r"LOCAL_ONLY\s*=\s*\{([^}]*)\}", source)
    return names, set(re.findall(r"'([a-z][a-z-]*)'", local.group(1))) if local else set()


def main() -> int:
    problems = []
    for page in PAGES:
        if not page.is_file():
            problems.append(f"{page.relative_to(ROOT)} does not exist -- the page that explains the numbers is gone")
            continue
        text = page.read_text(encoding="utf-8", errors="replace")
        for rel in sorted(set(COMMAND.findall(text))):
            if not (ROOT / rel).is_file():
                problems.append(f"the page names `{rel}`, which is not in the tree -- a reader following it fails on "
                                f"their first command")
        # A shell command split across lines with a trailing backslash is still one command; matching per line would
        # see only its first word and accuse the page of listing nothing.
        # 尾随反斜杠把一条命令拆成多行，它仍然是一条命令；逐行匹配只会看到第一个词，然后反过来指责页面什么都没列。
        joined = re.sub(r"\\\n\s*", " ", text)
        known, local_only = gate_names()
        named: set[str] = set()
        for run in GATE.findall(joined):
            named |= {word for word in run.split() if word not in {"all"}}
        for name in sorted(named - known):
            problems.append(f"the page names the gate `{name}`, which `PROJECT_GATES` does not have -- a reader gets "
                            f"an argparse error and blames their checkout")
        listed_local = named & local_only
        missing = (known - local_only) - named - {"all"}
        for name in sorted(missing):
            problems.append(f"`{name}` is a runnable gate that the page does not list -- the list stops being an "
                            f"inventory")
    for problem in problems:
        print(f"[docpaths] {problem}", file=sys.stderr)
    if problems:
        print(f"[docpaths] FAIL ({len(problems)} problem(s))", file=sys.stderr)
        return 1
    print("[docpaths] OK (every command the reproduction page names points at a file that exists)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
