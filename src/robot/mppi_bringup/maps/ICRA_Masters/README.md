# ICRA Masters — bridge maps

Placeholder maps for the dual-region (under/over) bridge stack. Fill these in
at the venue:

- `under_map.yaml` / `under_map.pgm` — lower track with bridge pillars baked in
- `over_map.yaml`  / `over_map.pgm`  — upper bridge deck (guardrails)

**Both must be SLAMmed from the same physical start point** so PF particles
(which live in world meters) transfer correctly across the swap.

**Both must share resolution** (e.g. 0.05 m/px). PF's sensor-model lookup
table is sized by the under map's resolution; if the over map differs, the
over-region sensor likelihood will be miscalibrated.

After SLAMming:
1. Copy PGMs + YAMLs here, replacing the TODO placeholders.
2. (Optional) GIMP cleanup — see comments in each yaml.
3. Update `params_ICRA1.yaml`:
   - `over_wall_cost_map_yaml: '/abs/path/to/.../share/mppi_bringup/maps/ICRA_Masters/over_map.yaml'`
4. Update `particle_filter/config/localize.yaml`:
   - `over_map_yaml: '/abs/path/to/.../share/particle_filter/maps/ICRA_Masters/over_map.yaml'`
5. Update `config/region_ICRA_Masters.yaml` with bubble centres / radii in
   world-frame meters (look at the map in rviz; bubbles will publish on
   `/region/bubbles_viz` once the node is up).
