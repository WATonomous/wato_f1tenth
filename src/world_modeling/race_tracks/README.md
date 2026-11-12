# race_tracks

One folder per track. `TRACK=<folder>` in `watod-config.local.sh` picks the track for the whole
stack: the particle filter's map, the global planner's raceline and MPPI's. Default:
`wato_office` on the car, `autodrive` in sim. `watod` stops on a name with no folder here.

`DIRECTION=ccw` or `DIRECTION=cw` picks which way round the car drives (counter-clockwise or
clockwise, as seen in Foxglove with x right and y up), on tracks generated for both directions.
Empty (the default) uses the track's `raceline.csv`, its default direction.

```text
tracks/<name>/
  map.yaml, map.pgm        map_server map for localization (image: map.pgm); the original map, not the drawn track
  raceline.csv             the raceline the global planner publishes (the default direction)
  raceline_ccw.csv         counter-clockwise (DIRECTION=ccw)      } written by the raceline generator
  raceline_cw.csv          clockwise (DIRECTION=cw)               } (make_raceline.py --export)
  raceline_mppi*.csv       optional: MPPI's own format, same naming; only tracks with one can run CONTROLLER=mppi
  track_info.yaml          generated tracks: default direction, lap time and top speed per direction, car profile
```

| Track | Where | Default direction | Raceline from |
| --- | --- | --- | --- |
| `wato_office` | WATO office loop (car default) | counter-clockwise | the office MPPI raceline |
| `e7_fifth_v3` | E7 fifth floor | counter-clockwise | raceline generator, both directions (drawn edges, `track_info.yaml`) |
| `e7_fifth_v3-fixed` | E7 fifth floor (same map as `e7_fifth_v3`) | counter-clockwise | raceline generator with braking at 2.5 m/s^2 (was 4.0), so the car's nose dips less and the lidar stays off the floor; 2026-11-12 |
| `e7_fifth_big` | E7 fifth floor, larger loop | counter-clockwise | its reference raceline |
| `autodrive` | AutoDRIVE sim track (sim default); same geometry as the F1TENTH gym's Porto | clockwise | the sim raceline; edges measured to the map's walls |

## raceline.csv

```text
x_m,y_m,v_mps,dr_m,dl_m
```

One row per waypoint, map frame, a closed loop in the direction of travel. `v_mps` is the target
speed (published in each pose's `position.z`). `dr_m` / `dl_m` are the distances to the right and
left edge of the track we laid out (the edges drawn in the generator's track editor), which need not
be the walls. The global planner publishes them on `/global_planner/reference_track`.

## Adding or regenerating a track

The raceline generator (`tools/wato_f1tenth_raceline_generator`, get it with `./watod tools`) does it
all; see `agents/workflows/raceline.md`.

1. Make the map (`MAPPING=true`, then save it) and copy it in here as `map.yaml` + `map.pgm`.
2. In the generator: draw the track's edges (`python track_editor.py maps/<name>`, only if the map is
   not already one clean ring), then `python make_raceline.py <name> --export`. It optimizes both
   directions with the car profile and writes the files above. `raceline.csv` keeps the track's
   current direction unless `--default-direction` says otherwise.
3. Look at the reports (`outputs/<name>/report_*.png` in the generator), then check in Foxglove
   before driving: `/map` and `/global_planner/path` must line up, inside the walls.
