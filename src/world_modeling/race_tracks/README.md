# race_tracks

One folder per track. `TRACK=<folder>` in `watod-config.local.sh` picks the track for the whole
stack: the particle filter's map, the global planner's raceline and MPPI's. Default:
`wato_office` on the car, `autodrive` in sim. `watod` stops on a name with no folder here.

```text
tracks/<name>/
  map.yaml, map.pgm     map_server map (image: map.pgm)
  raceline.csv          the raceline the global planner publishes
  raceline_mppi.csv     optional: MPPI's own format; only tracks with one can run CONTROLLER=mppi
```

| Track | Where | Raceline from |
| --- | --- | --- |
| `wato_office` | WATO office loop (car default) | the office MPPI raceline |
| `e7_fifth_v3` | E7 fifth floor | the v3 optimal raceline with edge distances |
| `e7_fifth_big` | E7 fifth floor, larger loop | its reference raceline |
| `autodrive` | AutoDRIVE sim track (sim default); same geometry as the F1TENTH gym's Porto | the sim raceline; edges measured to the map's walls |

## raceline.csv

```text
x_m,y_m,v_mps,dr_m,dl_m
```

One row per waypoint, map frame, a closed loop. `v_mps` is the target speed (published in each
pose's `position.z`). `dr_m` / `dl_m` are the distances to the right and left edge of the track
we laid out, which need not be the walls. The global planner publishes them on
`/global_planner/reference_track`.

## Adding a track

1. Make the map (`MAPPING=true`, then save it) and copy it in as `map.yaml` + `map.pgm`.
2. Make the raceline and its edge distances, in the format above. `generate_raceline_reference.py`
   (global_planner) adds `dr_m,dl_m` to an `x,y,v` raceline from a centerline with widths.
3. Check it in Foxglove before driving: `/map` and `/global_planner/path` must line up.
