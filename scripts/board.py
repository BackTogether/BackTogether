"""Team task board on GitHub Issues + Projects, usable by people and their agents.

Usage:
    uv run python scripts/board.py setup                   # check gh login, scopes and board access
    uv run python scripts/board.py new "title" --to LOGIN [--to LOGIN] [--area drone]
                                         [--files "a.py, b/"] [--done "criteria"] [--body TEXT]
    uv run python scripts/board.py move <issue> <status>   # backlog todo doing blocked review done
    uv run python scripts/board.py list [--mine] [--status doing] [--all]
    uv run python scripts/board.py say <issue> "text"      # comment signed as your agent
    uv run python scripts/board.py inbox                   # new mentions/assignments/replies
    uv run python scripts/board.py watch [--interval 60]   # block until inbox news, then exit
    uv run python scripts/board.py hook session|prompt     # Claude Code hook (settings.json)

Stdlib only; needs `gh` logged in with the `project` scope (`gh auth refresh -s project`).
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from datetime import UTC, datetime, timedelta
from pathlib import Path

PROJECT_OWNER = "gra1111"
PROJECT_NUMBER = 1
BOARD_URL = f"https://github.com/users/{PROJECT_OWNER}/projects/{PROJECT_NUMBER}"
REPO = "gra1111/leukerbard-robotics-AI-hackathon-2026"

STATUSES = ("Backlog", "Todo", "In Progress", "Blocked", "In Review", "Done")
STATUS_ALIASES = {"doing": "In Progress", "progress": "In Progress", "review": "In Review"}
AGENT_SIGNATURE = "🤖 **{login}'s agent:**"
GH_TIMEOUT_S = 20
FIRST_RUN_LOOKBACK = timedelta(hours=24)
HOOK_MIN_GAP_S = 90
SNIPPET_LEN = 280


def run_gh(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["gh", *args], capture_output=True, text=True, timeout=GH_TIMEOUT_S, check=False
    )


def gh(*args: str) -> str:
    result = run_gh(*args)
    if result.returncode != 0:
        sys.exit(f"gh {' '.join(args[:3])} failed: {result.stderr.strip()}")
    return result.stdout


def gh_json(*args: str):
    return json.loads(gh(*args) or "null")


def my_login() -> str:
    return gh("api", "user", "--jq", ".login").strip()


def resolve_status(name: str) -> str:
    key = name.strip().lower()
    for status in STATUSES:
        if status.lower() == key:
            return status
    if key in STATUS_ALIASES:
        return STATUS_ALIASES[key]
    sys.exit(f"Unknown status {name!r}. Use one of: {', '.join(STATUSES)}")


# ---------------------------------------------------------------- setup


def setup_problems() -> list[str]:
    """What stops this machine from using the board, each with the fix the human must run."""
    if not shutil.which("gh"):
        return ["GitHub CLI missing: install it (`brew install gh`), then `gh auth login`."]
    if run_gh("auth", "status").returncode != 0:
        return ["Not logged in to GitHub: run `! gh auth login -h github.com -s project`."]
    headers = run_gh("api", "user", "--include").stdout.lower()
    scope_line = next((h for h in headers.splitlines() if h.startswith("x-oauth-scopes:")), None)
    if scope_line and "project" not in scope_line.replace(" ", "").split(":")[1].split(","):
        return [
            "Token lacks the `project` scope: run `! gh auth refresh -h github.com -s project`."
        ]
    if run_gh("project", "view", *project_ref()).returncode != 0:
        return [f"No access to the board {BOARD_URL}: ask {PROJECT_OWNER} to add you."]
    return []


def cmd_setup(_: argparse.Namespace) -> None:
    problems = setup_problems()
    if not problems:
        print(f"Board setup OK: {BOARD_URL}")
        return
    print("Board setup incomplete:\n" + "\n".join(f"- {p}" for p in problems))
    sys.exit(1)


# ---------------------------------------------------------------- project board


def project_ref() -> list[str]:
    if not PROJECT_NUMBER:
        sys.exit("PROJECT_NUMBER is not set in scripts/board.py")
    return [str(PROJECT_NUMBER), "--owner", PROJECT_OWNER]


def set_status(issue_url: str, status: str) -> None:
    """Add the issue to the board (idempotent) and set its Status column."""
    project_id = gh_json("project", "view", *project_ref(), "--format", "json")["id"]
    fields = gh_json("project", "field-list", *project_ref(), "--format", "json")["fields"]
    field = next(f for f in fields if f["name"] == "Status")
    option = next(o for o in field["options"] if o["name"] == status)
    item = gh_json("project", "item-add", *project_ref(), "--url", issue_url, "--format", "json")
    gh(
        "project", "item-edit", "--id", item["id"], "--project-id", project_id,
        "--field-id", field["id"], "--single-select-option-id", option["id"],
    )  # fmt: skip


def cmd_new(args: argparse.Namespace) -> None:
    login = my_login()
    mentions = " ".join(f"@{who.lstrip('@')}" for who in args.to)
    sections = [args.body or "", f"**For:** {mentions}" if mentions else ""]
    if args.files:
        sections.append(f"**Files:** {args.files}")
    if args.done:
        sections.append(f"**Done when:** {args.done}")
    sections.append(f"_{AGENT_SIGNATURE.format(login=login)} opened via scripts/board.py_")
    cmd = ["issue", "create", "--repo", REPO, "--title", args.title]
    cmd += ["--body", "\n\n".join(s for s in sections if s)]
    for who in args.to:
        cmd += ["--assignee", who.lstrip("@")]
    if args.area:
        cmd += ["--label", f"area: {args.area}"]
    if any(who.lstrip("@") != login for who in args.to):
        cmd += ["--label", "request"]
    url = gh(*cmd).strip()
    set_status(url, resolve_status(args.status))
    print(url)


def cmd_move(args: argparse.Namespace) -> None:
    status = resolve_status(args.status)
    issue = gh_json("issue", "view", args.issue, "--repo", REPO, "--json", "url,state,assignees")
    set_status(issue["url"], status)
    if status == "In Progress" and not issue["assignees"]:
        gh("issue", "edit", args.issue, "--repo", REPO, "--add-assignee", "@me")
    if status == "Done" and issue["state"] == "OPEN":
        gh("issue", "close", args.issue, "--repo", REPO)
    if status != "Done" and issue["state"] == "CLOSED":
        gh("issue", "reopen", args.issue, "--repo", REPO)
    print(f"#{args.issue} -> {status}")


def cmd_list(args: argparse.Namespace) -> None:
    data = gh_json("project", "item-list", *project_ref(), "--format", "json", "--limit", "300")
    me = my_login() if args.mine else None
    wanted = resolve_status(args.status) if args.status else None
    rows = []
    for item in data["items"]:
        status = item.get("status") or "No status"
        assignees = item.get("assignees") or []
        if wanted and status != wanted:
            continue
        if not wanted and not args.all and status == "Done":
            continue
        if me and me not in assignees:
            continue
        number = (item.get("content") or {}).get("number", "")
        rows.append((STATUSES.index(status) if status in STATUSES else -1, number, status, item))
    for _, number, status, item in sorted(rows, key=lambda r: (r[0], str(r[1]))):
        who = ",".join(item.get("assignees") or []) or "-"
        print(f"#{number:<4} {status:<12} {who:<28} {item.get('title', '')}")
    if not rows:
        print("Nothing here.")


def cmd_say(args: argparse.Namespace) -> None:
    body = f"{AGENT_SIGNATURE.format(login=my_login())} {args.text}"
    print(gh("issue", "comment", args.issue, "--repo", REPO, "--body", body).strip())


# ---------------------------------------------------------------- inbox


def state_path() -> Path:
    """Per-clone state, inside .git so it is never committed."""
    git_dir = subprocess.run(
        [
            "git",
            "-C",
            str(Path(__file__).parent),
            "rev-parse",
            "--path-format=absolute",
            "--git-common-dir",
        ],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    return Path(git_dir) / "board-inbox.json"


def load_state() -> dict:
    try:
        return json.loads(state_path().read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def save_state(state: dict) -> None:
    state_path().write_text(json.dumps(state, indent=1))


def describe(thread: dict, me: str) -> str | None:
    """One line per new thread event; None if the latest activity is our own."""
    subject = thread["subject"]
    number = subject["url"].rsplit("/", 1)[-1] if subject.get("url") else "?"
    kind = "issues" if subject["type"] == "Issue" else "pull"
    line = f"#{number} [{thread['reason']}] {subject['title']}"
    comment_url = subject.get("latest_comment_url") or ""
    if "/comments/" in comment_url:
        comment = gh_json("api", comment_url)
        author = comment["user"]["login"]
        if author == me:
            return None
        snippet = " ".join(comment["body"].split())[:SNIPPET_LEN]
        line += f"\n    @{author}: {snippet}"
    return f"{line}\n    https://github.com/{REPO}/{kind}/{number}"


def check_inbox() -> list[str]:
    """New participating notifications for this repo since the last check, oldest first."""
    state = load_state()
    now = datetime.now(UTC)
    since = state.get("last_check") or (now - FIRST_RUN_LOOKBACK).isoformat()
    threads = gh_json(
        "api", f"repos/{REPO}/notifications?participating=true&all=true&since={since}"
    )
    seen = state.get("seen", {})
    me = my_login() if threads else ""
    news = []
    for thread in sorted(threads or [], key=lambda t: t["updated_at"]):
        if seen.get(thread["id"]) == thread["updated_at"]:
            continue
        seen[thread["id"]] = thread["updated_at"]
        line = describe(thread, me)
        if line:
            news.append(line)
    save_state({"last_check": now.isoformat(), "seen": seen})
    return news


def format_news(news: list[str]) -> str:
    return "\n".join(
        [
            f"Board inbox: {len(news)} new item(s) for you in {REPO}",
            *news,
            "Read the full thread with `gh issue view <n> --comments`; reply with board.py say.",
        ]
    )


def cmd_inbox(_: argparse.Namespace) -> None:
    news = check_inbox()
    print(format_news(news) if news else "Board inbox: nothing new.")


def cmd_watch(args: argparse.Namespace) -> None:
    while True:
        news = check_inbox()
        if news:
            print(format_news(news))
            return
        time.sleep(args.interval)


def cmd_hook(args: argparse.Namespace) -> None:
    """Never fails or blocks the session: emits news as extra context, or nothing."""
    parts = []
    if args.event == "session":
        try:
            problems = setup_problems()
        except Exception:
            problems = []
        if problems:
            parts.append(
                "Task board setup needed. Before anything else, tell your human to run this "
                "once (the `!` prefix runs it in this session; it opens a browser to approve):\n"
                + "\n".join(f"- {p}" for p in problems)
            )
            emit(args.event, parts)
            return
    try:
        last = load_state().get("last_check")
        recent = last and datetime.now(UTC) - datetime.fromisoformat(last) < timedelta(
            seconds=HOOK_MIN_GAP_S
        )
        if args.event == "prompt" and recent:
            return
        news = check_inbox()
    except (Exception, SystemExit):
        return
    if news:
        parts.append(format_news(news))
    if args.event == "session":
        parts.append(
            f"Team board: {BOARD_URL} (see AGENTS.md > Task board). To hear about mentions "
            "while idle, run `uv run python scripts/board.py watch` as a background command."
        )
    emit(args.event, parts)


def emit(event: str, parts: list[str]) -> None:
    """Print Claude Code hook JSON that adds `parts` to the session context."""
    if not parts:
        return
    name = "SessionStart" if event == "session" else "UserPromptSubmit"
    output = {"hookEventName": name, "additionalContext": "\n\n".join(parts)}
    print(json.dumps({"hookSpecificOutput": output}))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("setup", help="check gh login, scopes, board access").set_defaults(
        func=cmd_setup
    )

    new = sub.add_parser("new", help="create a task and put it on the board")
    new.add_argument("title")
    new.add_argument("--to", action="append", default=[], help="GitHub login to assign + tag")
    new.add_argument("--area", help="area label, e.g. drone, bobac, integration")
    new.add_argument("--status", default="Todo")
    new.add_argument("--files", help="files/folders the task may touch")
    new.add_argument("--done", help="verifiable done criteria")
    new.add_argument("--body", help="description")
    new.set_defaults(func=cmd_new)

    move = sub.add_parser("move", help="change a task's status")
    move.add_argument("issue")
    move.add_argument("status")
    move.set_defaults(func=cmd_move)

    lst = sub.add_parser("list", help="show the board (Done hidden unless --all)")
    lst.add_argument("--mine", action="store_true")
    lst.add_argument("--status")
    lst.add_argument("--all", action="store_true")
    lst.set_defaults(func=cmd_list)

    say = sub.add_parser("say", help="comment on an issue, signed as your agent")
    say.add_argument("issue")
    say.add_argument("text")
    say.set_defaults(func=cmd_say)

    sub.add_parser("inbox", help="new items for you").set_defaults(func=cmd_inbox)

    watch = sub.add_parser("watch", help="wait for new inbox items, print them, exit")
    watch.add_argument("--interval", type=int, default=60, help="seconds between checks")
    watch.set_defaults(func=cmd_watch)

    hook = sub.add_parser("hook", help="Claude Code hook entry point")
    hook.add_argument("event", choices=["session", "prompt"])
    hook.set_defaults(func=cmd_hook)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
