"""Check that the Pages site in web/ is complete before it is deployed.

Every file the page points at (scripts, stylesheet, icon, images) and every module an ES import
names must exist in the repository, with the exact case: GitHub Pages serves from a case
sensitive filesystem, so a wrong case works on Windows and 404s once deployed.

    uv run python scripts/check_web.py          # or: python3 scripts/check_web.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

WEB = Path(__file__).resolve().parent.parent / "web"
ENTRY = WEB / "index.html"

# src="..." / href="..." in the HTML, and from "./module.js" in the JavaScript.
HTML_REF = re.compile(r'(?:src|href)\s*=\s*"([^"]+)"')
JS_IMPORT = re.compile(r'(?:import|export)[^"\';]*?from\s*["\']([^"\']+)["\']')


def is_local(reference: str) -> bool:
    """A reference to a file in the site, not a URL, an anchor or a data: URI."""
    return not reference.startswith(("http://", "https://", "//", "#", "data:", "mailto:"))


def existing_case_sensitive(path: Path) -> bool:
    """True when the file exists with exactly this spelling, even on Windows."""
    if not path.exists():
        return False
    return path.name in (entry.name for entry in path.parent.iterdir())


def check_file(path: Path, pattern: re.Pattern[str], problems: list[str]) -> None:
    for reference in pattern.findall(path.read_text(encoding="utf-8")):
        if not is_local(reference):
            continue
        target = (path.parent / reference.split("?")[0].split("#")[0]).resolve()
        if not existing_case_sensitive(target):
            problems.append(f"{path.relative_to(WEB.parent)}: references missing {reference}")


def main() -> int:
    problems: list[str] = []

    if not ENTRY.is_file():
        print(f"error: no {ENTRY}", file=sys.stderr)
        return 1

    check_file(ENTRY, HTML_REF, problems)
    for script in sorted(WEB.glob("*.js")):
        check_file(script, JS_IMPORT, problems)

    if problems:
        print("The site is not deployable:", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1

    checked = len(list(WEB.glob("*.js"))) + 1
    print(f"web/: {checked} files checked, every reference resolves")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
