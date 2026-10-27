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
"""Replay an old branch's own commits onto a port/ branch at their new paths, keeping authorship.

Each commit under the mapped directories becomes one commit here, through git format-patch and
git am -3, so it keeps its original author, date and message. Every ported commit gets two
trailers: "Ported-from: <branch> <sha>" (so a re-run skips it) and "Co-authored-by: <you>" from
your git config. There is no option to change the author (AGENTS.md rule 15).
Junk (bags, build output, caches) is left out. See agents/workflows/update-legacy-branch.md.

    python3 agents/scripts/legacy_replay.py origin/stanley_controller_test \\
        --map src/robot/stanley_controller=src/action/stanley_controller [--dry-run]

On a conflict it stops inside git am: resolve, git add, git am --continue, then run it again.
"""

import argparse
import os
import re
import subprocess
import sys

JUNK_PATHSPECS = [
    f":(exclude,glob){g}"
    for g in (
        "**/*.db3",
        "**/*.mcap",
        "**/*.bag",
        "**/bags/**",
        "**/__pycache__/**",
        "**/*.pyc",
        "**/build/**",
        "**/install/**",
        "**/log/**",
    )
]
# A human porter only: an agent never appears in authorship (AGENTS.md rule 12)
AGENT_IDENTITY = re.compile(
    r"claude|anthropic|openai|codex|copilot|cursor|gpt|\[bot\]", re.I
)


def git(*args, check=True, stdin=None):
    r = subprocess.run(["git", *args], capture_output=True, text=True, input=stdin)
    if check and r.returncode != 0:
        sys.exit(f"git {' '.join(args)} failed:\n{r.stderr}{r.stdout}")
    return r


def rewrite_path(path, maps):
    for old, new in maps:
        if path == old or path.startswith(old + "/"):
            return new + path[len(old) :]
    return path


def rewrite_sections(body, maps):
    """Rewrite the paths in each file section of a patch; drop moves that the mapping makes no-ops."""
    sections = re.split(r"(?m)^(?=diff --git )", body)
    kept = [sections[0]]
    for sec in sections[1:]:
        lines = sec.split("\n")
        m = re.match(r"diff --git a/(\S+) b/(\S+)", lines[0])
        a, b = rewrite_path(m.group(1), maps), rewrite_path(m.group(2), maps)
        out = [f"diff --git a/{a} b/{b}"]
        for line in lines[1:]:
            for prefix in (
                "--- a/",
                "+++ b/",
                "rename from ",
                "rename to ",
                "copy from ",
                "copy to ",
            ):
                if line.startswith(prefix):
                    line = prefix + rewrite_path(line[len(prefix) :], maps)
                    break
            out.append(line)
        if a == b and any(x.startswith("rename from ") for x in out):
            if "similarity index 100%" in out:
                continue  # moved between two old paths that now map to the same place
            # a rename with edits that now stays in place: keep it as a plain edit
            out = [
                x
                for x in out
                if not x.startswith(("similarity index", "rename from", "rename to"))
            ]
        kept.append("\n".join(out))
    return "".join(kept), len(kept) - 1


def add_trailers(patch, trailers):
    """Insert trailers at the end of the commit message (before the '---' that starts the diffstat)."""
    lines = patch.split("\n")
    end = lines.index("---")
    msg = lines[:end]
    while msg and not msg[-1].strip():
        msg.pop()
    last = msg[-1] if msg else ""
    if not re.match(r"^[\w-]+: ", last) or last.startswith("Subject: "):
        msg.append("")
    return "\n".join(msg + trailers + [""] + lines[end:])


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "branch", help="the old branch, for example origin/stanley_controller_test"
    )
    ap.add_argument(
        "--map",
        action="append",
        required=True,
        metavar="OLD=NEW",
        help="old directory = new directory",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="list what each commit would bring, apply nothing",
    )
    args = ap.parse_args()

    maps = sorted(
        (tuple(m.rstrip("/").split("=", 1)) for m in args.map), key=lambda x: -len(x[0])
    )
    if any(len(m) != 2 or not all(m) for m in maps):
        sys.exit("--map takes OLD=NEW directories")

    current = git("rev-parse", "--abbrev-ref", "HEAD").stdout.strip()
    if not current.startswith("port/"):
        sys.exit(
            f"On '{current}'. Cut a port/<branch> branch from the target first (update-legacy-branch.md)."
        )
    git_dir = git("rev-parse", "--git-dir").stdout.strip()
    if os.path.isdir(os.path.join(git_dir, "rebase-apply")):
        sys.exit(
            "A git am is in progress: resolve it (git am --continue or --abort), then run this again."
        )
    if (
        not args.dry_run
        and git("status", "--porcelain", "--untracked-files=no").stdout.strip()
    ):
        sys.exit(
            "The working tree has uncommitted changes; commit or stash them first."
        )

    name = git("config", "user.name", check=False).stdout.strip()
    email = git("config", "user.email", check=False).stdout.strip()
    if not name or not email:
        sys.exit("Set git config user.name and user.email to the human doing the port.")
    if AGENT_IDENTITY.search(f"{name} {email}"):
        sys.exit(
            f"git identity '{name} <{email}>' looks like an agent; the porter must be a human."
        )

    mb = git("merge-base", args.branch, "HEAD").stdout.strip()
    commits = git(
        "rev-list", "--reverse", "--no-merges", f"{mb}..{args.branch}"
    ).stdout.split()
    done = set(
        re.findall(
            r"^Ported-from: \S+ ([0-9a-f]{40})$",
            git("log", "--format=%B", "HEAD").stdout,
            re.M,
        )
    )
    old_dirs = [old for old, _ in maps]

    ported = skipped = already = 0
    for sha in commits:
        subject = git("log", "-1", "--format=%h %an: %s", sha).stdout.strip()
        if sha in done:
            already += 1
            continue
        patch = git(
            "format-patch",
            "-1",
            "-M",
            "--stdout",
            "--binary",
            "--no-signature",
            sha,
            "--",
            *old_dirs,
            *JUNK_PATHSPECS,
        ).stdout
        if "\ndiff --git " not in patch:
            skipped += 1
            continue
        head, sep, body = patch.partition("\ndiff --git ")
        body, files = rewrite_sections("diff --git " + body, maps)
        if not files:
            print(
                f"skip  {subject} (only moves between paths that map to the same place)"
            )
            skipped += 1
            continue
        author_email = git("log", "-1", "--format=%ae", sha).stdout.strip()
        trailers = [f"Ported-from: {args.branch} {sha}"]
        if author_email.lower() != email.lower():
            trailers.append(f"Co-authored-by: {name} <{email}>")
        patch = add_trailers(head + "\n", trailers) + body

        if args.dry_run:
            paths = sorted(set(re.findall(r"(?m)^diff --git a/\S+ b/(\S+)", body)))
            print(f"would port  {subject}\n    " + "\n    ".join(paths))
            ported += 1
            continue
        r = git("am", "-3", "--quiet", check=False, stdin=patch)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            sys.exit(
                f"Conflict porting {subject}.\n"
                "Resolve the files, git add them, run git am --continue, then run this command again\n"
                "(ported commits are skipped). Or git am --abort to stop."
            )
        print(f"port  {subject}")
        ported += 1

    verb = "would port" if args.dry_run else "ported"
    print(
        f"\n{verb} {ported}, skipped {skipped} (nothing under the mapped paths), already ported {already}"
    )


if __name__ == "__main__":
    main()
