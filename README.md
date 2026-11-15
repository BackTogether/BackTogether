# Leukerbad Robotics Hackathon 2026

Resilient & Safe City track. Python 3.11 monorepo. A drone finds a specific lost child from a
photo sent by a parent; a ground robot (Bobac) goes to the child, checks on them and starts a
video call with the parent. Details in [AGENTS.md](AGENTS.md#project).

## Quick start
Needs [uv](https://docs.astral.sh/uv/getting-started/installation/) (`curl -LsSf https://astral.sh/uv/install.sh | sh`).

```bash
git clone https://github.com/gra1111/leukerbard-robotics-AI-hackathon-2026.git
cd leukerbard-robotics-AI-hackathon-2026
uv sync                                                          # env + deps (downloads Python 3.11 if missing)
uv run pre-commit install --hook-type pre-commit --hook-type commit-msg   # FIRST STEP: git hooks
gh auth refresh -h github.com -s project   # task board access (agents remind you if missing)
uv run python scripts/board.py setup             # should print "Board setup OK"
```

The hooks run ruff on every commit and reject commit messages that break
[COMMITS.md](COMMITS.md).

## How we work
Read [AGENTS.md](AGENTS.md) (rules for people and agents) and [COMMITS.md](COMMITS.md).
Tasks are GitHub issues on the [team board](https://github.com/users/gra1111/projects/1); agents use
`uv run python scripts/board.py` to create, move and get notified about them. See
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

## Website (GitHub Pages)
[web/](web/) is the "Find My Child" page parents use at an event: they add a photo of their child
and the page POSTs a search request that starts the drone, with a 3D view of the Hover Air X1
launching from the Bobac3 base. Setup, the request format and how to point it at the real robot
are in [web/README.md](web/README.md).

```bash
cd web && python3 -m http.server 8080     # run it locally on http://localhost:8080
python3 scripts/check_web.py              # what CI checks before deploying
```

Pushing to `main` with changes under `web/` deploys it through
[.github/workflows/pages.yml](.github/workflows/pages.yml). **Once, by a repo admin:**
Settings → Pages → Build and deployment → Source: **GitHub Actions**.

## Ethernet connection (drone laptop ↔ robot laptop)
The laptop that flies the drone has its Wi-Fi on the drone (`HoverX1_060039`, no internet), so it
cannot reach the robot. A second laptop on the event Wi-Fi carries its child reports to the robot,
joined to it by an Ethernet cable (the robot drives away, so the cable never goes to the robot):

```
drone ── drone Wi-Fi ── LAPTOP A (<LAPTOP_A_IP>) ══ Ethernet ══ LAPTOP B (<LAPTOP_B_IP>) ── event Wi-Fi ── robot :8001
```

Laptop A sends to `<LAPTOP_B_IP>:8001`; laptop B forwards that port to `child_goal_server` on the robot
through ssh (as `bobac3`, see [bobac/SSH.md](bobac/SSH.md)). Both laptops are Windows here; the
robot must run a launch with `child_goal_server` on port 8001.

**Once, on both laptops** (PowerShell **as administrator**; `Get-NetAdapter` gives the adapter's
name, `Ethernet` below). Fixed IPs with no gateway, so each keeps its own Wi-Fi for everything else:
```powershell
New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress <LAPTOP_A_IP> -PrefixLength 24   # A; <LAPTOP_B_IP> on B
New-NetFirewallRule -DisplayName "cable ping" -Direction Inbound -Protocol ICMPv4 -IcmpType 8 -RemoteAddress <ETHERNET_SUBNET>/24 -Profile Any -Action Allow
```

**Once, on laptop B:**
1. Port 8001 open to the cable only (as administrator):
   ```powershell
   New-NetFirewallRule -DisplayName "robot tunnel 8001" -Direction Inbound -Protocol TCP -LocalPort 8001 -LocalAddress <LAPTOP_B_IP> -RemoteAddress <ETHERNET_SUBNET>/24 -Profile Any -Action Allow
   ```
2. The robot in `%USERPROFILE%\.ssh\config`, with a key the robot accepts and **no passphrase**
   (the tunnel cannot type one). Check with `ssh bobac-robot`:
   ```
   Host bobac-robot
       HostName <ROBOT_IP>
       User bobac3
       IdentityFile ~/.ssh/bobac_robot
       IdentitiesOnly yes
       ServerAliveInterval 15
       ServerAliveCountMax 3
       ExitOnForwardFailure yes
   ```
3. The tunnel, `%USERPROFILE%\robot-tunnel.ps1`. It listens only on the cable and localhost
   (never on the event Wi-Fi: nobody else can send the robot a goal) and reconnects by itself:
   ```powershell
   while ($true) {
       ssh -N -o BatchMode=yes -L <LAPTOP_B_IP>:8001:127.0.0.1:8001 -L 127.0.0.1:8001:127.0.0.1:8001 bobac-robot
       Start-Sleep 3
   }
   ```
4. Optional, start it at login: `Win+R` → `shell:startup` → new shortcut to
   `powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Minimized -File "%USERPROFILE%\robot-tunnel.ps1"`.

**Every time:**
```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "$HOME\robot-tunnel.ps1"   # B: leave it open
curl.exe http://127.0.0.1:8001/status                                          # B: "state": "idle"
Test-NetConnection <LAPTOP_B_IP> -Port 8001                                        # A: TcpTestSucceeded : True
```
```bash
python3 -m drone.report_child --to http://<LAPTOP_B_IP>:8001 --ping    # A, WSL, repo root
python3 -m drone.report_child --to http://<LAPTOP_B_IP>:8001 --watch   # A: before drone/find
```

| Symptom | Cause |
|---|---|
| `curl` on B gets no answer | tunnel down (its window says why), or the robot runs no `child_goal_server` on 8001 |
| A's `Test-NetConnection` fails | cable not up on both (`Get-NetAdapter`), IPs, or B's firewall rule |
| B loses internet | the cable got a gateway: `Remove-NetRoute -InterfaceAlias "Ethernet" -DestinationPrefix 0.0.0.0/0` |
| Robot IP changed | `hostname -I` on the robot, then `HostName` in B's ssh config |

The robot must not move between the drone's takeoff and the report: the drone reports the child
relative to where the robot stood at takeoff.

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
docs/         REVIEWER.md
scripts/      check_commit_msg.py, board.py, check_web.py
web/          the GitHub Pages site (see web/README.md)
```
