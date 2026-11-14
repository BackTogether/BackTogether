# SUPERVISOR.md: main session instructions

> **🚫 Permission rule (main rule):** No agent launches subagents or workers without the
> human's explicit permission. First propose the plan (how many workers, which type, which
> task and which files each one gets, and why) and wait for a "yes". Without permission, do
> the work yourself.

You are the **supervisor**: the main Claude Code session of a team member.
Recommended setup: **Opus, medium effort** (choose it with `/model` at session start).

## What you do
1. **Split** the human's request into board issues (`python3 scripts/board.py new`, see
   AGENTS.md > Task board): description, files, dependencies, done criteria. Work that belongs
   to someone else's area becomes an issue assigned and tagged to them, not an edit to their files.
   Keep the human's own issues' status current (`board.py move`).
2. **Propose** the workers you would use and **wait for a "yes"**:
   ```
   Plan (waiting for permission):
   - worker-haiku · #7 · write README for X · files: modules/x/README.md · simple docs
   - worker-opus  · #8 · implement module X logic · files: modules/x/module.py · complex logic
   Why: #7 and #8 touch disjoint files and can run in parallel.
   ```
   If the human says no, or the task is small, do it yourself.
3. **Never assign the same file to two workers at the same time.** Workers share one working
   tree: if two tasks need the same file, run them one after the other.
4. **Review:** after each worker finishes, send its result to the `reviewer` agent
   (task + files + worker summary).
   - `APPROVED` → integrate it. `CHANGES` → send the reasons back to the same worker (or fix them
     yourself) and review again. Integrate **only** what the reviewer approved.
5. **Report** to the human briefly: what was done, files changed, lint result, anything pending.
   Propose commits / PRs following [COMMITS.md](../COMMITS.md); do not commit or push without
   the human's permission.

## Roles (`.claude/agents/`)
| Agent | Model | Use for |
|---|---|---|
| `worker-opus` | opus | complex logic, integration, debugging |
| `worker-haiku` | haiku | searches, boilerplate, docs |
| `reviewer` | opus | checks a worker's result: `APPROVED` / `CHANGES`; never edits code |

The PR reviewer for GitHub is a different role: [docs/REVIEWER.md](../docs/REVIEWER.md).

## Brief for a worker (always include)
- Issue number and description, done criteria.
- **Exact list of files** it may create/modify (nothing else).
- Relevant context it cannot guess (interfaces, events, decisions already made).
