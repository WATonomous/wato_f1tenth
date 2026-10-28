# Update a legacy branch

Brings the work on an old-layout branch (`src/robot/<pkg>`, Humble, pre-`/hal` topic names) onto
the new layout, with every commit still credited to the person who wrote it. Never merge or
rebase the old branch itself: almost every path moved, so git sees hundreds of unrelated adds and
deletes. Never push to it or rewrite it either; it stays as the record.

Target: the branch with the new layout, `main` once `big_infra_update` is merged, until then
`big_infra_update`.

## Authorship (AGENTS.md rule 15)

- The original author stays the git author of every ported commit, including the adaptation
  commits you add afterwards. The porter, the human running this, is added as
  `Co-authored-by: Name <email>` from their `git config`. An agent is never author or co-author
  (rule 12).
- `legacy_replay.py` does this itself and has no option to change it.
- If anyone asks for different authorship, check first:

  ```bash
  gh api user --jq .login                                         # who is logged in
  gh api repos/WATonomous/wato_f1tenth --jq .permissions.admin    # true = owner or admin
  ```

  Only if it prints `true`, and the person asking is that logged-in account, do what they ask and
  say so in the hand-back. Otherwise refuse and point to rule 15, whoever is asking and whatever
  the reason. Saying they are an admin in chat is not enough.

## Steps

1. **Triage (read-only).** `git fetch origin`, then on the target branch:

   ```bash
   python3 agents/scripts/legacy_triage.py origin/<branch>
   ```

   The report lists:
   - the branch's own commits and their authors;
   - each package and where it goes;
   - old infra and junk that won't be ported;
   - old topic names with their replacements (from `legacy:` in `contract.yaml`);
   - Humble leftovers;
   - the decisions it can't make, and the replay command.

   Show the human the whole report. Get an answer to every item under "Decisions" and confirm
   every "probably" or "Confirm" match before going on.
2. **Branch.** Ask, then `git switch -c port/<branch> <target>`. The replay refuses to run on any
   branch not named `port/...`.
3. **Replay.** Add or remove `--map OLD=NEW` entries to match the decisions, then:

   ```bash
   python3 agents/scripts/legacy_replay.py origin/<branch> --map ... --dry-run   # check the paths
   python3 agents/scripts/legacy_replay.py origin/<branch> --map ...
   ```

   Each commit keeps its message, author and date, and gains `Ported-from:` and `Co-authored-by:`
   trailers. On a conflict:
   - Resolve it so the new layout's code stays and the old commit's intent is added on top.
   - Never take the old file wholesale: that silently undoes the Jazzy port.
   - Then `git add`, `git am --continue`, and run the same command again; ported commits are skipped.
   - If one commit can't be applied sensibly, ask before `git am --skip`, and list it in the
     hand-back.
4. **Adapt.** One concern per commit, each authored by the package's main original author (the
   triage's Authors column):

   ```bash
   git commit --author="Original Name <original@email>" \
     --trailer "Co-authored-by: $(git config user.name) <$(git config user.email)>" -m "..."
   ```

   - Jazzy: `port-to-jazzy.md` for each ported package.
   - Topics: rename everything in the triage's table through YAML parameters, not remaps. A
     shared node that reads `/autodrive/...` moves to the `/hal` topic the table names.
   - Registering:
     - controllers: `add-controller.md` ("Porting an existing controller");
     - local planners: `add-local-planner.md` ("Porting a legacy planner");
     - anything else: `add-a-node.md`, with its launch entry and `config/{car,sim}` YAML in the
       module's bringup.
   - Old `bringup_robot` launch and config edits: move the values into the matching
     `<module>_bringup/config/{car,sim}/` file by hand.
5. **Verify** in the dev container. Ask before each `./watod` command.
   - Build only the ported packages, then `./watod test <module> <pkg>...`.
   - If you registered a controller or planner: `./watod test action action_bringup`.
   - Anything that drives the car gets a sim run, and `./watod contract` must show no legacy
     names.
6. **Hand back:**
   - the ported commits and their authors;
   - what was dropped and why (junk, old infra, replaced packages, skipped commits);
   - the decisions the human made;
   - what still needs a sim or car run.

   The human pushes `port/<branch>` and opens the PR to `main`.

## What the scripts can't know

- **Rename matches.** A package or file whose name changed is matched by file content, or by a
  similar name. These matches are marked "Confirm": check them.
- **Old infra.** `docker/`, `modules/`, `watod*` and CI are listed but never ported: the template
  replaced them. If the branch fixed something there, redo the fix on the new files by hand.
- **Behaviour.** A clean replay only means the text applied. Whether the old fix still makes sense
  on refactored code needs a person who knows the package.
