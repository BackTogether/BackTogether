"""Validate commit messages / PR titles against COMMITS.md.

Usage:
    python scripts/check_commit_msg.py <commit-msg-file>     # git commit-msg hook
    python scripts/check_commit_msg.py --text "feat: add x"  # PR title
    python scripts/check_commit_msg.py --range BASE..HEAD    # every commit in a range (CI)

Exit code 0 if valid, 1 otherwise. Keep it stdlib-only (runs before `uv sync` in CI).
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

TYPES = ("feat", "fix", "refactor", "docs", "chore", "style", "perf")
MAX_LEN = 72
PATTERN = re.compile(
    r"^(?P<type>" + "|".join(TYPES) + r")"
    r"(\((?P<scope>[a-z0-9][a-z0-9._/-]*)\))?"
    r": (?P<summary>\S.*)$"
)
FORBIDDEN = re.compile(
    r"co-authored-by|signed-off-by|authored-by|claude|codex|anthropic|openai|chatgpt|copilot"
    r"|\bgpt\b|\bai\b",
    re.IGNORECASE,
)
SCISSORS = "# ------------------------ >8 ------------------------"


def clean(raw: str) -> str:
    """Drop git comment lines, the verbose diff after the scissors and trailing blanks."""
    lines = []
    for line in raw.splitlines():
        if line.startswith(SCISSORS):
            break
        if line.startswith("#"):
            continue
        lines.append(line.rstrip())
    return "\n".join(lines).strip()


def validate(message: str) -> list[str]:
    msg = clean(message)
    if not msg:
        return ["message is empty"]
    errors = []
    lines = msg.splitlines()
    title = lines[0]
    if len(lines) > 1:
        errors.append("must be a single line (no body, no trailers)")
    if len(title) > MAX_LEN:
        errors.append(f"title is {len(title)} chars, max is {MAX_LEN}")
    match = PATTERN.match(title)
    if not match:
        errors.append(f"must match '<type>(<scope>): <summary>', type in: {', '.join(TYPES)}")
    else:
        summary = match["summary"]
        if summary[0].isupper():
            errors.append("summary must start in lowercase imperative ('add', not 'Add')")
        if summary.endswith("."):
            errors.append("summary must not end with a period")
    if hit := FORBIDDEN.search(msg):
        errors.append(f"must not contain signatures or mention tools/AI (found {hit.group(0)!r})")
    return errors


def report(label: str, message: str) -> bool:
    errors = validate(message)
    if errors:
        print(f"✗ {label}: {clean(message).splitlines()[0] if clean(message) else ''!r}")
        for error in errors:
            print(f"    - {error}")
        print("  See COMMITS.md")
    return not errors


def commits_in_range(rev_range: str) -> list[tuple[str, str]]:
    shas = subprocess.run(
        ["git", "rev-list", "--no-merges", rev_range], capture_output=True, text=True, check=True
    ).stdout.split()
    return [
        (
            sha[:7],
            subprocess.run(
                ["git", "log", "-1", "--format=%B", sha], capture_output=True, text=True, check=True
            ).stdout,
        )
        for sha in shas
    ]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("file", nargs="?", help="commit message file (commit-msg hook)")
    group.add_argument("--text", help="message to check (e.g. PR title)")
    group.add_argument("--range", dest="rev_range", help="git revision range, e.g. a..b")
    args = parser.parse_args(argv)

    if args.text is not None:
        ok = report("PR title", args.text)
    elif args.rev_range:
        results = [report(f"commit {sha}", msg) for sha, msg in commits_in_range(args.rev_range)]
        ok = all(results)
        print(f"checked {len(results)} commit(s)")
    else:
        ok = report("commit message", Path(args.file).read_text(encoding="utf-8"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
