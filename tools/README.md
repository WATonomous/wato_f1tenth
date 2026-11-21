# tools

Standalone repos used alongside the stack. They are cloned here by `watod`, git-ignored in this
repo, and follow their own default branch (no pinned commit):

| Folder | Repo | What it is |
| --- | --- | --- |
| `wato_f1tenth_raceline_generator/` | [WATonomous/wato_f1tenth_raceline_generator](https://github.com/WATonomous/wato_f1tenth_raceline_generator) | Turns a track map into a raceline (`race_tracks/tracks/<name>/raceline.csv`) |
| `wato_f1tenth_model_farm/` | [WATonomous/wato_f1tenth_model_farm](https://github.com/WATonomous/wato_f1tenth_model_farm) | Model farm: the Data Standard, auto-labelling and model training (conda env, WATcloud jobs). Renamed from `wato_f1tenth_model_fram` |

```bash
./watod tools          # clone the missing ones, pull the rest to the latest main/master
./watod tools status   # branch, commit, and how far behind each one is
```

A repo on another branch or with uncommitted changes is only fetched, never pulled, so your work
in it is safe. Commit and push changes from inside its own folder; this repo does not track them.
To add a repo, add a line to `REPOS` in `watod_scripts/watod-tools.sh`.
