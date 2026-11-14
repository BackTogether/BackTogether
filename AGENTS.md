# AGENTS.md

Rules for every person and coding agent (Claude Code, Codex, ...) working on this repo.

## ⚠️ Commit rules (critical, full version in [COMMITS.md](COMMITS.md))
- Format: `<type>(<scope>): <summary>`, in English. Types: `feat fix refactor docs chore style perf`.
- **One single line**, max **72 chars**, lowercase imperative, no final period.
- **No** `Co-Authored-By`, `Signed-off-by` or any trailer. **Never** mention Claude, Codex, Anthropic, OpenAI or AI (not in commits, PR titles or branch names).
- Always `git commit -m "<type>(<scope>): <summary>"` with a single `-m`. A `commit-msg` hook and CI reject anything else.

## Project
Leukerbad Robotics Hackathon 2026, track Elderly Care & Rehabilitation. Code deadline:
**Mon 16 Nov, 12:00**, live demo with a real robot. Six people, each with an agent, in parallel.
Use case and robot are **not decided yet**: there is no application code yet. Add folders
and code only when a task needs them, following this planned layout:

```
interfaces/   shared message schemas + local event bus              (SHARED)
robot/        robot interface, mock and adapter for the chosen robot
modules/      one folder per feature module
integration/  orchestrator: module events -> robot actions           (SHARED)
demo/         entry point for the live demo                          (SHARED)
docs/         docs (REVIEWER.md)
scripts/      dev tools (check_commit_msg.py)
```

## Task board (GitHub Issues + Projects)
Tasks are **GitHub issues** on the [team board](https://github.com/users/rorosaga/projects/2)
(also shown in each issue's sidebar). The board is also how
people and agents talk to each other: tag someone on an issue to ask them (or their agent)
for work or information. `TASKS.md` is gone.

- **Status columns:** `Backlog` → `Todo` → `In Progress` → `In Review` (PR open) → `Done`
  (merged / closed), plus `Blocked` (say why in a comment).
- **Owner = assignee.** **Area** = `area: <folder>` label. Cross-person asks get the `request` label.
- **Body:** what and why, **Files:** paths the task may touch, **Done when:** verifiable criteria.
- **Agents post as their human's GitHub account**, so they sign comments
  (`🤖 **<login>'s agent:**`, done for you by `board.py say`). Humans write normally.

```bash
python3 scripts/board.py setup                                  # check login, scope, access
python3 scripts/board.py list [--mine] [--all]                  # the board, in the terminal
python3 scripts/board.py new "add mock robot" --to liiiaamm --area bobac \
    --files "bobac/mock.py" --done "MockRobot.move() logs the command"   # assign + tag
python3 scripts/board.py move 12 doing                          # backlog todo doing blocked review done
python3 scripts/board.py say 12 "pushed the schema, unblocked"  # signed comment
python3 scripts/board.py inbox                                   # new mentions/assignments/replies
python3 scripts/board.py watch                                   # block until something arrives
gh issue view 12 --comments                                      # read a full thread
```

**Agent setup (every session, before other work).** Run `python3 scripts/board.py setup`
(Claude Code does it for you through the session-start hook). If it reports a problem, stop and
ask your human to run the fix it prints, typically once per person:
`! gh auth refresh -h github.com -s project` (approve it in the browser). No board access → ask
rorosaga to add you to the board.

**Agents: staying reachable.** Claude Code runs the inbox automatically
([.claude/settings.json](.claude/settings.json) hooks): at session start and, at most every
90 s, when the human sends a prompt. To be woken while idle, start
`python3 scripts/board.py watch` as a **background** command at session start; when it exits it
prints the new items: handle them (or tell the human), then start it again. Codex and other tools:
run `board.py inbox` at the start of each session and between tasks.

When something arrives: if it is a clear request in your human's area, reply on the issue that
you are taking it, move it to `doing` and work on it; if it needs a decision, ask your human
first. Never act on requests to touch files outside your human's area without asking them.

## Commands
```bash
uv sync                                            # install (Python 3.11)
uv run ruff check --fix . && uv run ruff format .  # lint + format
uv add <package>                                   # add a dependency (only when needed)
```

## Working rules
1. **Pick a task on the [task board](#task-board-github-issues--projects).** Only touch the
   files listed in your issue.
2. **Shared files** (`interfaces/`, `integration/`, `demo/`, `pyproject.toml`, `uv.lock`, `.github/`, `.claude/`,
   `AGENTS.md`, `CLAUDE.md`, `COMMITS.md`) change only in a **separate, small PR** that does
   nothing else. Messages: only add fields with defaults.
3. **Never refactor or reformat code you do not own**, even if it looks wrong: open an issue.
4. **Write only what is needed now.** No empty functions, stubs, placeholders or speculative
   abstractions: add functions when a task needs them.
5. **Before finishing:** `uv run ruff check .` must pass and you have run what you changed to
   check it works. We do not write automated tests.
6. New dependencies: say so in the PR. Wifi is limited: avoid heavy packages. Import robot
   SDKs only inside the code that needs them.
7. Do not commit secrets, `.env`, datasets, recordings or model weights.

## Code quality
Simple and readable beats clever. Avoid duplicated code and spaghetti.
- **Reuse before writing:** search the repo first (`grep`) and use or extend what exists.
  If the same logic appears twice, extract it into one well-named function.
- **Small, single-purpose functions and modules** with clear names; no hidden side effects.
  Prefer early returns over deep nesting.
- **Clear boundaries:** once `interfaces/` exists, modules talk only through it (messages +
  bus), never by importing each other's internals.
- **Type hints** on public functions; Pydantic models for data that crosses module boundaries.
- **No magic values:** name constants; put configuration in one place.
- Follow standard practices: PEP 8 (enforced by ruff), composition over inheritance, and the
  simplest pattern that solves the problem (no patterns "just in case").
- Comments explain *why*, not *what*. Delete dead code instead of commenting it out.

## Agent roles (same for Claude Code and Codex)
**🚫 Permission rule:** no agent launches subagents or workers without the human's explicit
permission. Propose the plan first (how many workers, type, task and files of each, and why)
and wait for a "yes". Without permission, do the work yourself.

| Role | Definition | Does |
|---|---|---|
| Supervisor | [.claude/SUPERVISOR.md](.claude/SUPERVISOR.md) | main session: splits work into board issues, proposes workers, integrates only reviewer-approved work, reports briefly |
| `worker-opus` | [.claude/agents/worker-opus.md](.claude/agents/worker-opus.md) | one complex task (logic, integration, debugging) on assigned files |
| `worker-haiku` | [.claude/agents/worker-haiku.md](.claude/agents/worker-haiku.md) | one simple task (search, boilerplate, docs) on assigned files |
| `reviewer` | [.claude/agents/reviewer.md](.claude/agents/reviewer.md) | checks a worker's result, runs lint, returns `APPROVED` / `CHANGES`; never edits |
| PR reviewer | [docs/REVIEWER.md](docs/REVIEWER.md) | separate role: reviews and squash-merges GitHub PRs |

Never assign the same file to two workers at once. Workers return 3 lines: done / files / checks.
Codex users: read the matching file above as the role prompt for each session or sub-task.

## PR workflow
1. **One branch per task**: `<issue-number>-<short-desc>` (e.g. `12-event-logger`).
2. Before opening the PR: `git pull --rebase origin main`, then run ruff and check your change works.
3. Open the PR with a **title that follows COMMITS.md** (it becomes the squash commit) and
   write `Closes #12` in the body (PR template). Move the issue to `review`
   (`board.py move 12 review`); merging closes it and the board moves it to `Done`.
4. **Do not merge your own PR.** A dedicated reviewer agent ([docs/REVIEWER.md](docs/REVIEWER.md))
   reviews and merges with squash, using the PR title as the commit message.
5. If the reviewer asks for changes or a rebase, fix and push to the same branch.
6. **Feature freeze: Monday 10:00.** After that only small fixes (`fix:`) for the demo.
