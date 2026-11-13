# AGENTS.md

Rules for every person and coding agent (Claude Code, Codex, ...) working on this repo.

## ⚠️ Commit rules (critical, full version in [COMMITS.md](COMMITS.md))
- Format: `<type>(<scope>): <summary>`, in English. Types: `feat fix refactor docs chore style perf`.
- **One single line**, max **72 chars**, lowercase imperative, no final period.
- **No** `Co-Authored-By`, `Signed-off-by` or any trailer. **Never** mention Claude, Codex, Anthropic, OpenAI or AI (not in commits, PR titles or branch names).
- Always `git commit -m "<type>(<scope>): <summary>"` with a single `-m`. A `commit-msg` hook and CI reject anything else.

## Project
Leukerbad Robotics Hackathon 2026, track Elderly Care & Rehabilitation. Code deadline:
**Mon 16 Nov, 12:00**, live demo with a real robot. Four people, each with an agent, in parallel.
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

## Commands
```bash
uv sync                                            # install (Python 3.11)
uv run ruff check --fix . && uv run ruff format .  # lint + format
uv add <package>                                   # add a dependency (only when needed)
```

## Working rules
1. **Pick a task in [TASKS.md](TASKS.md).** Only touch the files listed in your task.
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
| Supervisor | [.claude/SUPERVISOR.md](.claude/SUPERVISOR.md) | main session: splits work into TASKS.md tasks, proposes workers, integrates only reviewer-approved work, reports briefly |
| `worker-opus` | [.claude/agents/worker-opus.md](.claude/agents/worker-opus.md) | one complex task (logic, integration, debugging) on assigned files |
| `worker-haiku` | [.claude/agents/worker-haiku.md](.claude/agents/worker-haiku.md) | one simple task (search, boilerplate, docs) on assigned files |
| `reviewer` | [.claude/agents/reviewer.md](.claude/agents/reviewer.md) | checks a worker's result, runs lint, returns `APPROVED` / `CHANGES`; never edits |
| PR reviewer | [docs/REVIEWER.md](docs/REVIEWER.md) | separate role: reviews and squash-merges GitHub PRs |

Never assign the same file to two workers at once. Workers return 3 lines: done / files / checks.
Codex users: read the matching file above as the role prompt for each session or sub-task.

## PR workflow
1. **One branch per task**: `<task-id>-<short-desc>` (e.g. `t03-event-logger`).
2. Before opening the PR: `git pull --rebase origin main`, then run ruff and check your change works.
3. Open the PR with a **title that follows COMMITS.md** (it becomes the squash commit) and
   reference the task (`Task: T03`) using the PR template. Set the task to `review` in TASKS.md.
4. **Do not merge your own PR.** A dedicated reviewer agent ([docs/REVIEWER.md](docs/REVIEWER.md))
   reviews and merges with squash, using the PR title as the commit message.
5. If the reviewer asks for changes or a rebase, fix and push to the same branch.
6. **Feature freeze: Monday 10:00.** After that only small fixes (`fix:`) for the demo.
