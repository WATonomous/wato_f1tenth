#!/usr/bin/env python3
"""A/B check of mppi_cpp against the original JAX mppi_example.

Run inside the robot dev container after building mppi_cpp:
  g++ -O2 -std=c++17 -I include -I /usr/include/eigen3 test/compare_main.cpp src/track.cpp \
      src/wall_sdf.cpp ../../../build/mppi_cpp/libmppi_cuda.a -lyaml-cpp \
      -L/usr/local/cuda/lib64 -lcudart -o /tmp/compare_main
  python3 test/compare_with_jax.py /tmp/compare_main <raceline.csv> <map.yaml>
"""
import subprocess
import sys
import types

import numpy as np

sys.path.insert(0, sys.path[0] + '/../../mppi_example')
from mppi_example.infer_env import InferEnv  # noqa: E402
from mppi_example.utils.Track import Track  # noqa: E402
import jax.numpy as jnp  # noqa: E402

binary, csv_path, map_yaml = sys.argv[1:4]
out = {}
for line in subprocess.run([binary, csv_path, map_yaml], check=True, capture_output=True, text=True).stdout.splitlines():
    name, *vals = line.split()
    out[name] = np.array([float(v) for v in vals])

cfg = types.SimpleNamespace(
    n_steps=12, sim_time_step=0.1, use_waypoint_speed_profile=True, speed_profile_scale=0.4,
    speed_profile_min_speed=1.0, speed_profile_max_speed=9.0, speed_profile_lookahead_steps=3,
    speed_profile_iterations=3, friction=0.5, friction_max=1.5, ref_vel=2.0, state_predictor='dynamic_ST',
    norm_params=np.array([[5.4, 5.4], [-5.4, -5.4]]), wall_cost_enabled=True, wall_cost_map_yaml=map_yaml,
    opponent_auto_wall_check_enabled=False)
track, cfg = Track.load_map_from_csv(csv_path, cfg)
env = InferEnv(track, cfg, DT=cfg.sim_time_step)
T = cfg.n_steps

states = [np.array([11.2, -0.55, 0.05, 2.0, -0.05, 0.1, 0.02]),
          np.array([5.0, 3.0, -0.1, 4.0, 2.0, -0.5, -0.05]),
          np.array([0.0, 0.0, 0.0, 1.5, 1.0, 0.0, 0.0])]
worst = 0.0


def check(name, a, b, tol):
    global worst
    a, b = np.asarray(a, float).ravel(), np.asarray(b, float).ravel()
    err = np.max(np.abs(a - b)) if a.size == b.size else np.inf
    worst = max(worst, err / tol)
    print(f'{name:10s} max|diff|={err:.3e} (tol {tol:g}) {"OK" if err <= tol else "MISMATCH"}')


refs = []
for i, s in enumerate(states):
    ref, _ = env.get_refernece_traj(s, max(cfg.ref_vel, s[3]), T)
    refs.append(ref)
    check(f'ref{i}', ref, out[f'ref{i}'], 1e-6)

sdf = np.asarray(env.wall_sdf)
h, w = sdf.shape
ox, oy, res = float(env.wall_origin[0]), float(env.wall_origin[1]), env.wall_resolution
probes = np.array([[np.float32(ox - 1) + np.float32(w * res + 2) * (i % 20) / np.float32(19),
                    np.float32(oy - 1) + np.float32(h * res + 2) * (i // 20) / np.float32(9)] for i in range(200)],
                  dtype=np.float32)
check('sdf', env.sample_wall_distance(jnp.asarray(probes)), out['sdf'], 1e-4)

actions = np.array([[0.6 * np.sin(np.float32(0.7) * t + np.float32(0.3)), 0.5 * np.cos(np.float32(0.5) * t)]
                    for t in range(T)], dtype=np.float32)
cost_params = jnp.array([10.0, 0.3, 0.6, 2.0, 0.4, 3.0, 80.0, 0.5, 0.425, 20.0, 0.4, 0.4, 0.9, 1.0,
                         0.0, 1.2, 0.7, 0.0, 0.55, 1.5], dtype=jnp.float32)
reward_weights = jnp.array([0.25, 0.5, 0.01], dtype=jnp.float32)
for i in range(2):
    mu = env.get_reference_frictions(states[i], T)
    x = jnp.asarray(states[i], dtype=jnp.float32)
    roll = []
    for t in range(T):
        x, _, _ = env.step(x, jnp.asarray(actions[t]), None, jnp.asarray(cfg.norm_params, dtype=jnp.float32), mu[t])
        roll.append(np.asarray(x))
    roll = np.array(roll)
    check(f'roll{i}', roll, out[f'roll{i}'], 2e-3)
    check(f'hroll{i}', roll, out[f'hroll{i}'], 2e-3)
    opp = refs[i][1:T + 1, :2] + np.array([0.2, 0.1])
    r = np.asarray(env.reward_fn_xy(jnp.asarray(roll), jnp.asarray(refs[i], dtype=jnp.float32), reward_weights,
                                    cost_params, jnp.asarray(opp, dtype=jnp.float32)))
    R = np.cumsum(r[::-1])[::-1]
    check(f'R{i}', R, out[f'R{i}'], 5e-2)
    werr = out[f'weights_err{i}'][0]
    print(f'weights{i}   host-recomputed a_opt max|diff|={werr:.3e}  da steer range {out[f"weights_err{i}"][1]:.3f}..{out[f"weights_err{i}"][2]:.3f}')
    worst = max(worst, werr / 1e-4)
print('ALL OK' if worst <= 1.0 else 'SOME MISMATCHES')
