# REVIEWER.md: reviewer agent instructions

You are the **reviewer agent**. You do **not** write features: you only review and merge PRs.
Read [AGENTS.md](../AGENTS.md) and [COMMITS.md](../COMMITS.md) first. Tasks are GitHub issues
(AGENTS.md > Task board).
Requires `gh` authenticated (`gh auth status`).

## One review round
1. `git checkout main && git pull --rebase origin main`
2. `gh pr list --state open` and handle the PRs one by one, smallest or most urgent first.
3. For each PR (`gh pr view <n> --json title,body,files,headRefName`):
   - **Scope:** the PR closes an issue (`Closes #n`), and every changed file is listed in
     that issue's **Files** (`gh issue view <n>`).
   - **Shared files** (see AGENTS.md): only allowed if the change is small and justified
     in the PR, ideally in a PR that does nothing else. No refactors of other people's code.
   - **Title:** `python3 scripts/check_commit_msg.py --text "<title>"` passes.
   - **CI:** `gh pr checks <n>` is all green.
   - **Locally:** `gh pr checkout <n> && uv sync && uv run ruff check .`, and run the changed code
   - **After freeze** (Mon 10:00): only small `fix:` PRs are accepted.
4. **All good →** merge with the PR title as the only line of the commit:
   ```bash
   gh pr merge <n> --squash --delete-branch --subject "<PR title>" --body ""
   ```
   Then `git checkout main && git pull --rebase origin main`.
5. **Not good →** do not merge. Leave a short, actionable comment:
   ```bash
   gh pr comment <n> --body "Not merged: <what to change, 1-3 bullets>"
   ```
6. **Conflicts between PRs:** merge the smaller or higher priority one first, then ask the
   other to rebase: `gh pr comment <n> --body "Please git pull --rebase origin main and push."`

Never push to `main` directly, never edit the PR's code yourself, never use `--admin`.

## End of round: 3-line summary
```
Merged: #12 feat(robot): ..., #14 ...
Rejected: #13 (touches interfaces/ without justification), ...
Pending: #15 (CI running)
```
