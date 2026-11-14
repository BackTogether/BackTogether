# Leukerbad Robotics Hackathon 2026

Elderly Care & Rehabilitation track. Python 3.11 monorepo. Use case and robot are not decided
yet, so there is no application code: only the team rules and tooling. Code is added as needed.

## Quick start
Needs [uv](https://docs.astral.sh/uv/getting-started/installation/) (`curl -LsSf https://astral.sh/uv/install.sh | sh`).

```bash
git clone https://github.com/gra1111/leukerbard-robotics-AI-hackathon-2026.git
cd leukerbard-robotics-AI-hackathon-2026
uv sync                                                          # env + deps (downloads Python 3.11 if missing)
uv run pre-commit install --hook-type pre-commit --hook-type commit-msg   # FIRST STEP: git hooks
gh auth refresh -h github.com -s project   # task board access (agents remind you if missing)
python3 scripts/board.py setup             # should print "Board setup OK"
```

The hooks run ruff on every commit and reject commit messages that break
[COMMITS.md](COMMITS.md).

## How we work
Read [AGENTS.md](AGENTS.md) (rules for people and agents) and [COMMITS.md](COMMITS.md).
Tasks are GitHub issues on the [team board](https://github.com/users/gra1111/projects/1); agents use
`python3 scripts/board.py` to create, move and get notified about them. See
[AGENTS.md](AGENTS.md#task-board-github-issues--projects).

**PR workflow**
1. One branch per task: `git checkout -b 12-event-logger` (12 = issue number).
2. Before opening the PR: `git pull --rebase origin main` and `uv run ruff check .`.
3. Open a PR whose **title follows COMMITS.md** and closes its issue (`Closes #12`).
   **Do not merge it yourself.**
4. The reviewer agent reviews and merges with **squash**, using the PR title as the commit message.
5. **Feature freeze Monday 10:00**: after that, only small fixes.

## Reviewer agent
Instructions: [docs/REVIEWER.md](docs/REVIEWER.md). It needs the GitHub CLI authenticated:

```bash
gh auth login        # once
gh auth status       # check
```

Launch it from the repo root in a dedicated terminal (one person runs it):

```bash
claude "Follow docs/REVIEWER.md and run one review round"
# or, with Codex:
codex "Follow docs/REVIEWER.md and run one review round"
```
To repeat it periodically in Claude Code: `/loop 15m Follow docs/REVIEWER.md and run one review round`.

## Protecting `main` on GitHub (repo admin, once)
Settings → Rules → Rulesets → **New branch ruleset**:
- Target: default branch (`main`), Enforcement: **Active**, no bypass list.
- ✅ **Restrict deletions**, ✅ **Block force pushes**.
- ✅ **Require a pull request before merging**, required approvals: **0** (this blocks direct pushes).
  Allowed merge methods: **Squash** only.
- ✅ **Require status checks to pass**: add `lint` and `commit and PR title format`
  (they appear after CI has run once).

Also in Settings → General → Pull Requests: allow only **squash merging**, and set the default
squash commit message to **"Pull request title"**.
Note: rulesets on private repos need GitHub Pro/Team; on public repos they are free.

## Layout
No application code yet. Planned layout (folders are created when needed): see
[AGENTS.md](AGENTS.md#project).
```
drone/        HoverAir X1 drone
bobac/        Bobac_d wheeled robot with screen + webcam
docs/         REVIEWER.md
scripts/      check_commit_msg.py, board.py
```
