# Run commands on the car over SSH

How an agent on a developer's machine works on the Jetson without anyone logging an agent into it.
The Jetson is shared, so no personal agent accounts, API keys or agent sessions live on it. The agent
runs on the developer's machine and sends single commands over SSH, so all context stays local.

## 1. Check access first

Run a no-op with password prompts off, so it fails fast instead of hanging:

```bash
ssh -o BatchMode=yes -o ConnectTimeout=5 <host> true && echo OK
```

The car's host alias (in `~/.ssh/config`) is usually `micro-orin` (user `wato`). If the human hasn't
said which car, ask.

If it fails, stop and ask the human. Don't try to work around it:

- **"Permission denied (publickey,password)":** their SSH key isn't on the car. Ask whether they
  have a key (`ls ~/.ssh/id_ed25519.pub`). If not, they run `ssh-keygen -t ed25519`. Then they run
  `ssh-copy-id <user>@<host>` and type the car's password themselves.
- **"Could not resolve hostname" or a timeout:** the car is off, on another network, or not on
  Tailscale. Ask how they normally reach it, and suggest a `~/.ssh/config` entry:

  ```text
  Host micro-orin
      HostName micro-orin          # or the IP / Tailscale name
      User wato
      IdentityFile ~/.ssh/id_ed25519
  ```

Never ask for, store or type the car's password, and never put credentials on the car.

## 2. Find the repo on the car

Ask which checkout to use, or list them: `ssh <host> 'ls -d ~/wato_f1tenth*'`. Then confirm the
branch and state before changing anything:

```bash
ssh <host> 'cd <repo> && git log --oneline -1 && git status --short'
```

## 3. Rules still apply on the car

- AGENTS.md rule 1: ask before `./watod up|build|down` on the car, the same as locally.
- AGENTS.md rule 2: no `sudo` or hardware commands (`nvpmodel`, `jetson_clocks`, `taskset`).
  The car has no passwordless sudo anyway. Write the command out for the human to run.
- The car's personal settings live in its `watod-config.local.sh` (git-ignored). Editing that
  file for a test the human asked for is fine. Say what you changed.
- Never commit or push from the car.

## 4. Getting code onto the car

- **Normal:** commit on the developer's machine, the human pushes, and the car runs `git pull`.
- **Quick test before committing:** `scp` the changed files into the car's repo, build and test.
  Once the human has pushed, run `git checkout -- <files>` on the car before `git pull`, and
  delete any *new* files you copied: `git checkout` leaves untracked files alone, and `git pull`
  refuses to overwrite them. Check they match first:
  `git show origin/<branch>:<file> | cmp - <file>`.

## 5. Long commands

SSH drops kill foreground commands. Run builds detached, with a log and an exit marker:

```bash
ssh <host> 'cd <repo> && nohup ./watod build > ~/build.log 2>&1; echo "EXIT $?" >> ~/build.log'
```

Run that as a background task if your tool supports it. Otherwise add `&` inside the quotes and
check `tail ~/build.log` later. Don't `pkill -f <pattern>` over SSH: the pattern also matches your
own SSH command line, so it kills your session. Stop things by PID.

## 6. Checking the running stack

Container names are `watod_<car user>-<service>-1`, for example `watod_wato-interfacing_bringup-1`.
ROS commands need both setup files sourced, and a `timeout` so they return:

```bash
ssh <host> 'docker ps --format "{{.Names}} {{.Status}}"'
ssh <host> 'docker exec watod_wato-interfacing_bringup-1 bash -c "source /opt/ros/jazzy/setup.bash; \
  source /opt/watonomous/setup.bash; timeout 6 ros2 topic hz /hal/scan"'
ssh <host> 'docker logs --tail 50 watod_wato-world_modeling_bringup-1 2>&1'
```

Before anything that can move the car, tell the human what to do: wheels off the ground, and don't
hold the dead-man switch until the checks pass.

## Known car quirks

- Docker on the Jetson sets `"dns": ["8.8.8.8", "1.1.1.1"]` in `/etc/docker/daemon.json`.
  Without it, builds fail on Wi-Fi networks that hand out IPv6 DNS servers
  (see `deploy-to-car.md`).
- BlueZ 5.64 on the Jetson: use `./watod controller` for the gamepad. It runs bluetoothctl on the
  host, so it works over SSH.
