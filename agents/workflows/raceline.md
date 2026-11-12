# Make or fix a raceline

How a track's racelines are made with the raceline generator in `tools/`, both directions, and how to
give the car more room at speed. The generator is its own repo; this repo keeps only the results in
`src/world_modeling/race_tracks/tracks/<track>/`. **The only step that needs a person is drawing the
track's edges** (when the map is not already one clean ring). Agents run everything else.

## Where things are

| What | Where |
| --- | --- |
| Generator | `tools/wato_f1tenth_raceline_generator/` ([repo](https://github.com/WATonomous/wato_f1tenth_raceline_generator), default branch `master`) |
| Get or update it | `./watod tools` (`./watod tools status`) |
| Its agent notes | `tools/wato_f1tenth_raceline_generator/AGENTS.md`: environment, commands, how to read a run |
| Conda env | `wato_raceline` (Python 3.8): `tools/wato_f1tenth_raceline_generator/setup_env.sh` creates or updates it |
| The car, as the optimizer sees it | `tools/wato_f1tenth_raceline_generator/params/wato_car.yaml` (mass, limits, grip, edge margin) |
| Measuring the car (grip, mass, drag) | `tools/wato_f1tenth_raceline_generator/docs/measuring_the_car.md` |
| Tracks the stack uses | `src/world_modeling/race_tracks/tracks/<track>/`, picked with `TRACK` and `DIRECTION` (see its README) |

`raceline-gen.md` at the repo root describes the older `Raceline-Optimization` repo. Use this file.

## Rules

- The generator is plain Python in the conda env `wato_raceline`, not ROS. Run it on the host, never
  in a stack container and never with `colcon`. If the env is missing, create it yourself (in the
  background; it downloads packages): `conda env list | grep -q '^wato_raceline ' || tools/wato_f1tenth_raceline_generator/setup_env.sh`.
- The track editor's window needs a person. Agents never draw or invent track edges.
- Never edit generated files by hand (`raceline*.csv`, `track_info.yaml`, `maps/*_track.*`).
- Commits to the generator go in its own repo, from inside its folder. Ask before pushing there.
- A person looks at the reports and checks the raceline in Foxglove before the car drives it.

## The whole job

```bash
cd tools/wato_f1tenth_raceline_generator
conda activate wato_raceline      # or call the env's python directly
python make_raceline.py <track>                 # 1st run: copies race_tracks/tracks/<track>/map.* in
```

1. If it stops with exit code 2 ("not one clean ring"), ask the person to draw the edges:
   `python track_editor.py maps/<track>` (outer edge, `i`, inner edge, optionally `f` for start/finish,
   `s`, close). Then run `make_raceline.py` again.
2. It then builds the track, the centerline, and optimizes **both directions in parallel** with the car
   profile (a few minutes), and writes the reports `outputs/<track>_track/report_ccw.png`,
   `report_cw.png` and `report_compare.png` (or `outputs/<track>/...` for a closed track). Look at them;
   show them to the person.
3. Export to the stack: `python make_raceline.py <track> --export`. It writes
   `raceline_ccw.csv`, `raceline_cw.csv`, the MPPI versions, `raceline.csv` (= the default direction:
   the track's current one unless `--default-direction` is given) and `track_info.yaml` into
   `race_tracks/tracks/<track>/`. The track's `map.pgm` is not touched (localization keeps the original).
4. Check before driving: `TRACK=<track>` (and `DIRECTION=ccw|cw`) in `watod-config.local.sh`, run the
   stack, and confirm in Foxglove that `/global_planner/path` lies on `/map` and the edges
   (`/global_planner/edges`) sit inside the walls.

### Reading a run

- The table at the end: lap time, top speed, minimum clearance and spots under the edge margin, per
  direction. Details: `outputs/<name>/<dir>/summary_<dir>.json` (`solver.attempts` says how each solve went).
- `ERROR: the track is X m wide at (x, y) but the car needs Y m`: the drawn track is too narrow there
  for the car plus its edge margin. Widen it in the editor, or lower `track.edge_margin_m` in the profile.
- `N spot(s) under the margin`: even after narrowing the corridor around them (twice), the line comes
  closer to the drawn edges than the margin at those spots (red circles in the report), usually because
  the track is barely wider than the car plus margin there. Widen those spots.
- A direction that fails at every smoothing level (`s_reg` 1, 3, 10): see `HAIRPIN_FIX_NOTES.md`.

## Direction and start point

- The centerline is always written counter-clockwise; the optimizer drives it either way
  (`main_globaltraj_f110.py --direction ccw|cw`). Before this, the direction was an accident of the
  map's shape: check `track_info.yaml` or the report titles, never assume.
- Start/finish: set with `f` in the editor (saved in `maps/<map>_track_lines.json`); otherwise the
  straightest stretch. Both directions start there.

## More room from the walls at speed

`track.edge_margin_m` in `params/wato_car.yaml` (0.40 m now) is the clearance from the car's side to the
drawn track edge; the generator holds it against the real edges, not the optimizer's smoothed copy. The
other car values (top speed 10 m/s, 3.3 m/s² up, 4.0 m/s² down, grip) shape the speed profile; keep the
limits at or below the stack's (`make_raceline.py` warns if they differ from `vesc.yaml`).
