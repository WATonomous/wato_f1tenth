#!/usr/bin/env python3
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Triage an old-layout branch before porting it (agents/workflows/update-legacy-branch.md).

Read-only: runs only git plumbing, writes nothing. Prints a Markdown report of the branch's own
commits and authors, what each changed file becomes (a package to port and where to, an edit to a
package that moved, something deleted upstream, old infra, junk), old topic names to rename, Humble
leftovers, and the legacy_replay.py command for the ports it can map.

    python3 agents/scripts/legacy_triage.py origin/stanley_controller_test [--target HEAD]
"""

import argparse
import collections
import difflib
import os
import re
import subprocess
import sys

CONTRACT = "src/infrastructure/hal_contract/config/contract.yaml"

JUNK = re.compile(
    r"(\.(db3|mcap|pyc|bag)$|(^|/)bags?/|(^|/)(build|install|log)/|__pycache__|\.DS_Store$|\.vscode/)"
)
# Replaced wholesale by the template's infra on the new layout: never port these
OLD_INFRA = re.compile(
    r"^(docker/|modules/|watod|watod_scripts/|\.github/|dev_config|\.devcontainer/|\.env)"
)

# Simulator topics a shared node must not read (sim-to-car.md): what to use instead
AUTODRIVE = [
    (r"imu", "/hal/imu"),
    (r"lidar|scan", "/hal/scan"),
    (
        r"throttle|steering|command",
        "/hal/drive/autonomy (publish AckermannDriveStamped)",
    ),
    (r"odom|ips|pose", "/hal/localization/odom (pose) or /hal/odom (speed)"),
]

# Packages whose job the new layout already does
REPLACED = [
    (r"bag|recorder", "`./watod bag record` (bag_presets/) replaces this"),
    (r"^bringup_robot$|^action$", "split into the <module>_bringup packages"),
    (r"^gym_vis$", "deleted: AutoDRIVE replaced the gym sim"),
    (
        r"^(transport_drivers|asio_cmake_module|io_context|serial_driver|udp_driver)$",
        "released for Jazzy (ros-jazzy-serial-driver); the vendored copy was deleted",
    ),
]

# (keyword pattern, destination, kind) for packages that don't exist on the target yet
MODULE_GUESS = [
    (r"local_plan|frenet|overtak|opponent", "src/action", "local planner"),
    (r"control|pursuit|persuit|stanley|mpc|lqr|mppi", "src/action", "controller"),
    (r"msgs$", "src/wato_msgs", "messages"),
    (
        r"global_plan|raceline|map|locali[sz]|slam|ekf|filter|costmap|occupancy",
        "src/world_modeling",
        "world modeling",
    ),
    (
        r"vesc|driver|teleop|joy|mux|safety|e_?br[ae]+k|battery|urg|lidar|realsense|recorder",
        "src/interfacing",
        "interfacing",
    ),
    (r"sim|autodrive|gym", "src/sim", "sim"),
    (r"percep|detect|track|camera", "src/perception", "perception"),
]

HUMBLE = [
    (re.compile(r"std::experimental"), "std::experimental -> std (C++17)"),
    (
        re.compile(r"declare_parameter(<[^>]+>)?\(\s*\"[^\"]+\"\s*\)"),
        "parameter declared without a default",
    ),
    (
        re.compile(r"declare_parameter\(\s*'[^']+'\s*\)"),
        "parameter declared without a default",
    ),
    (re.compile(r"\btests_require\b"), "tests_require in setup.py (remove)"),
    (re.compile(r"\bdistutils\b"), "distutils (gone in Python 3.12)"),
    (
        re.compile(r"use_sim_time['\"]?\s*[:=,]\s*True"),
        "use_sim_time True (false on the car)",
    ),
]

TEXT_SUFFIXES = (
    ".py",
    ".cpp",
    ".hpp",
    ".h",
    ".c",
    ".cc",
    ".yaml",
    ".yml",
    ".xml",
    ".txt",
    ".launch",
    ".json",
    ".cfg",
    ".sh",
)


def git(*args, check=True):
    r = subprocess.run(["git", *args], capture_output=True, text=True)
    if check and r.returncode != 0:
        sys.exit(f"git {' '.join(args)} failed:\n{r.stderr}")
    return r.stdout


def packages_at(ref):
    """{package dir: package name} for every package.xml under src/ in ref (not agents/templates)."""
    pkgs = {}
    for path in git("ls-tree", "-r", "--name-only", ref, "--", "src").splitlines():
        if path.endswith("/package.xml"):
            m = re.search(
                r"<name>\s*([^<\s]+)\s*</name>",
                git("show", f"{ref}:{path}", check=False),
            )
            if m:
                pkgs[path.rsplit("/", 1)[0]] = m.group(1)
    return pkgs


def owner(path, pkg_dirs):
    best = None
    for d in pkg_dirs:
        if path.startswith(d + "/") and (best is None or len(d) > len(best)):
            best = d
    return best


def legacy_map(ref):
    """Old topic name -> replacement, from this checkout's contract (else the target's)."""
    root = git("rev-parse", "--show-toplevel").strip()
    try:
        with open(f"{root}/{CONTRACT}") as f:
            text = f.read()
    except OSError:
        text = git("show", f"{ref}:{CONTRACT}", check=False)
    section = text.split("\nlegacy:\n", 1)[1] if "\nlegacy:\n" in text else ""
    return dict(re.findall(r"^\s+(/[\w/]+):\s*(/[\w/]+)", section, re.M))


def earlier_dirs(mb, branch):
    """{package name: dirs it lived in during the branch's own commits} (packages that moved)."""
    dirs = collections.defaultdict(set)
    log = git(
        "log",
        "--no-merges",
        "--format=@%H",
        "--name-only",
        f"{mb}..{branch}",
        "--",
        "*package.xml",
    )
    commit = None
    for line in log.splitlines():
        if line.startswith("@"):
            commit = line[1:]
        elif line.endswith("package.xml"):
            m = re.search(
                r"<name>\s*([^<\s]+)\s*</name>",
                git("show", f"{commit}:{line}", check=False),
            )
            if m:
                dirs[m.group(1)].add(line.rsplit("/", 1)[0])
    return dirs


def renamed_files(mb, target):
    """{old file: new file} for files git sees as renamed between mb and target."""
    out = {}
    for line in git(
        "diff", "-M", "--diff-filter=R", "--name-status", mb, target
    ).splitlines():
        _, src, dst = line.split("\t")
        out[src] = dst
    return out


def file_maps(d, dest, changed_paths, renames, target_files):
    """Extra OLD=NEW maps for changed files in d that were renamed, not just moved with the package.

    Git's rename detection links files by content; a file rewritten during the port (pure_persuit.cpp
    -> pure_pursuit.cpp) needs the name fallback: same folder and extension, the closest file name.
    """
    out = []
    for path in sorted(changed_paths):
        moved = dest + path[len(d) :]
        if path in renames and renames[path] != moved:
            out.append((path, renames[path]))
        elif moved not in target_files:
            folder, ext = os.path.dirname(moved), os.path.splitext(moved)[1]
            candidates = [
                f
                for f in target_files
                if os.path.dirname(f) == folder and f.endswith(ext)
            ]
            best = max(
                candidates,
                key=lambda f: difflib.SequenceMatcher(
                    None, os.path.basename(f), os.path.basename(moved)
                ).ratio(),
                default=None,
            )
            if (
                best
                and difflib.SequenceMatcher(
                    None, os.path.basename(best), os.path.basename(moved)
                ).ratio()
                >= 0.75
            ):
                out.append((path, best))
    return out


def replaced_by(name):
    for pattern, note in REPLACED:
        if re.search(pattern, name):
            return note
    return None


def guess_module(name):
    for pattern, dest, kind in MODULE_GUESS:
        if re.search(pattern, name):
            return dest, kind
    return None, None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "branch", help="the old branch, for example origin/stanley_controller_test"
    )
    ap.add_argument(
        "--target", default="HEAD", help="branch with the new layout (default HEAD)"
    )
    args = ap.parse_args()

    branch, target = args.branch, args.target
    mb = git("merge-base", branch, target).strip()
    if not git("rev-list", "--no-merges", f"{mb}..{branch}").strip():
        print(
            f"# Triage: `{branch}` onto `{target}`\n\nNothing to port: every commit on it is already in `{target}`."
        )
        return
    commits = [
        line.split("\t", 4)
        for line in git(
            "log",
            "--reverse",
            "--no-merges",
            "--format=%h\t%an\t%ae\t%as\t%s",
            f"{mb}..{branch}",
        ).splitlines()
    ]
    merges = git("rev-list", "--merges", "--count", f"{mb}..{branch}").strip()

    base_pkgs = packages_at(mb)
    branch_pkgs = packages_at(branch)
    target_pkgs = packages_at(target)
    target_by_name = {name: d for d, name in target_pkgs.items()}
    legacy = legacy_map(target)
    history = earlier_dirs(mb, branch)
    renames = renamed_files(mb, target)
    target_files = set(git("ls-tree", "-r", "--name-only", target).splitlines())

    changed = []
    for line in git("diff", "--no-renames", "--name-status", mb, branch).splitlines():
        status, path = line.split("\t", 1)
        changed.append((status, path))

    junk, infra, loose = [], [], []
    by_pkg = collections.defaultdict(list)
    for status, path in changed:
        if JUNK.search(path):
            junk.append(path)
            continue
        d = owner(path, branch_pkgs) or owner(path, base_pkgs)
        if d:
            by_pkg[d].append((status, path))
        elif OLD_INFRA.search(path):
            infra.append(path)
        else:
            loose.append(path)

    porter = f"{git('config', 'user.name', check=False).strip()} <{git('config', 'user.email', check=False).strip()}>"
    out = [f"# Triage: `{branch}` onto `{target}`", ""]
    out += [
        f"- Split from the new layout's history at `{mb[:10]}` ({git('log', '-1', '--format=%as', mb).strip()}).",
        f"- {len(commits)} own commits ({merges} merge commits skipped), {len(changed)} changed files.",
        f"- Porter (added as `Co-authored-by` on every ported commit): `{porter}`.",
        "",
        "## Commits and authors",
        "",
        "| Commit | Date | Author | Subject |",
        "| --- | --- | --- | --- |",
    ]
    for h, an, ae, date, subject in commits:
        out.append(f"| `{h}` | {date} | {an} <{ae}> | {subject[:70]} |")

    maps, decisions = [], []
    out += [
        "",
        "## Packages",
        "",
        "| Package | On the branch | Status | Port to | Files | Authors |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for d in sorted(by_pkg):
        name = branch_pkgs.get(d) or base_pkgs.get(d)
        files = by_pkg[d]
        authors = collections.Counter(
            git(
                "log", "--no-merges", "--format=%an <%ae>", f"{mb}..{branch}", "--", d
            ).splitlines()
        )
        who = ", ".join(f"{a} ({n})" for a, n in authors.most_common())
        moved_from = sorted(history.get(name, set()) - {d})
        similar = difflib.get_close_matches(name, target_by_name, n=1, cutoff=0.85)
        if replaced_by(name):
            dest, status = "decide", f"replaced: {replaced_by(name)}"
            decisions.append(
                f"`{name}` (`{d}`): {replaced_by(name)}. Drop it, or move specific edits by hand?"
            )
        elif name in target_by_name:
            dest = target_by_name[name]
            status = (
                "edits a package that exists on the target"
                if dest == d
                else "edits a package that moved"
            )
            if dest != d:
                maps.append((d, dest))
                maps += file_maps(d, dest, [p for _, p in files], renames, target_files)
        elif similar and name not in target_by_name:
            dest = target_by_name[similar[0]]
            status = f"probably `{similar[0]}` on the target (similar name)"
            maps.append((d, dest))
            maps += [(old, dest) for old in moved_from]
            maps += file_maps(d, dest, [p for _, p in files], renames, target_files)
            decisions.append(
                f"Confirm `{name}` (`{d}`) is `{similar[0]}` (`{dest}`); matched by name."
            )
        elif d in base_pkgs and d not in branch_pkgs:
            dest, status = "-", "package deleted on this branch"
        elif d in base_pkgs:
            moved = collections.Counter(
                owner(v, target_pkgs)
                for k, v in renames.items()
                if k.startswith(d + "/") and owner(v, target_pkgs)
            )
            if moved:
                dest = moved.most_common(1)[0][0]
                status = f"edits a package renamed to `{target_pkgs[dest]}` (by file similarity)"
                maps.append((d, dest))
                maps += file_maps(d, dest, [p for _, p in files], renames, target_files)
                decisions.append(
                    f"Confirm `{d}` (`{name}`) became `{dest}` (`{target_pkgs[dest]}`); git matched them by file content."
                )
            else:
                dest, status = "decide", "edits a package deleted on the new layout"
                decisions.append(
                    f"`{d}` (`{name}`) was removed on the new layout: drop the edits or move them to its replacement."
                )
        else:
            module, kind = guess_module(name)
            if module:
                dest = f"{module}/{name}"
                status = f"new package ({kind})"
                maps.append((d, dest))
                # earlier commits used another path: map those too, or the replay skips them
                maps += [(old, dest) for old in moved_from]
                if kind in ("controller", "local planner"):
                    decisions.append(
                        f"`{name}` looks like a {kind}: after the replay, register it with "
                        f"`add-{'controller' if kind == 'controller' else 'local-planner'}.md` (porting section)."
                    )
            else:
                dest, status = "decide", "new package, module unknown"
                decisions.append(f"`{name}` (`{d}`): which module does it belong in?")
        if moved_from:
            status += f" (earlier at {', '.join(f'`{m}`' for m in moved_from)})"
        out.append(f"| `{name}` | `{d}` | {status} | `{dest}` | {len(files)} | {who} |")

    if loose:
        out += ["", "## Other files (review: port, move or drop)", ""] + [
            f"- `{p}`" for p in loose
        ]
        decisions.append(
            "Files outside any package are listed under 'Other files': decide each one."
        )
    if infra:
        out += ["", "## Old infra (replaced by the template's; not ported)", ""] + [
            f"- `{p}`" for p in infra
        ]
    if junk:
        out += ["", "## Junk (dropped: bags, build output, caches)", ""] + [
            f"- `{p}`" for p in junk
        ]

    topic_hits, humble_hits = [], []
    pattern = (
        re.compile(
            r"(?<![\w/.])("
            + "|".join(re.escape(k) for k in sorted(legacy, key=len, reverse=True))
            + r")(?![\w/])"
        )
        if legacy
        else None
    )
    for status, path in changed:
        if (
            status == "D"
            or JUNK.search(path)
            or OLD_INFRA.search(path)
            or not path.endswith(TEXT_SUFFIXES)
        ):
            continue
        d = owner(path, branch_pkgs)
        in_interfacing = bool(d and re.search(r"vesc|mux|teleop|interfacing", d))
        in_sim = bool(d and re.search(r"sim", d))
        for n, text in enumerate(
            git("show", f"{branch}:{path}", check=False).splitlines(), 1
        ):
            if pattern:
                for m in pattern.finditer(text):
                    old = m.group(1)
                    new = legacy[old]
                    if old == "/ackermann_cmd" and not in_interfacing:
                        new = "/hal/drive/autonomy (only drivers read the mux output)"
                    topic_hits.append(f"| `{path}:{n}` | `{old}` | `{new}` |")
            if not in_sim:
                for m in re.finditer(r"/autodrive/[\w/]+", text):
                    new = next(
                        (t for rx, t in AUTODRIVE if re.search(rx, m.group(0))),
                        "a /hal topic",
                    )
                    topic_hits.append(
                        f"| `{path}:{n}` | `{m.group(0)}` | sim-only: `{new}` |"
                    )
            for rx, what in HUMBLE:
                if rx.search(text):
                    humble_hits.append(f"- `{path}:{n}`: {what}")

    if topic_hits:
        out += [
            "",
            "## Old topic names (rename in YAML parameters, not remaps)",
            "",
            "| Where | Old | New |",
            "| --- | --- | --- |",
        ]
        out += sorted(set(topic_hits))
    if humble_hits:
        out += ["", "## Humble leftovers (port-to-jazzy.md)", ""] + sorted(
            set(humble_hits)
        )

    out += ["", "## Decisions for the human", ""]
    out += [f"{i}. {d}" for i, d in enumerate(decisions, 1)] or [
        "None: everything maps automatically."
    ]

    if maps:
        flags = " ".join(f"--map {a}={b}" for a, b in maps)
        out += [
            "",
            "## Replay",
            "",
            "On a fresh `port/<branch>` branch cut from the target, after the human approves this report:",
            "",
            "```bash",
            f"python3 agents/scripts/legacy_replay.py {branch} {flags}",
            "```",
        ]
    print("\n".join(out))


if __name__ == "__main__":
    main()
