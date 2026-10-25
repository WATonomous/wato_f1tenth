"""Interactive tuner for the wall-distance penalty used in infer_env.py.

The penalty equation visualized here is:

    wall_penalty = max(0, wall_margin - wall_dist) ** wall_power

- `wall_dist`  : distance from the car to the nearest wall (x-axis)
- `wall_margin`: distance at which the penalty turns on (knee of the curve)
- `wall_power` : how sharply the penalty grows once inside the margin

Run:  python3 wall_penalty_tuner.py
"""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

INIT_MARGIN = 0.30   # meters
INIT_POWER = 0.02
INIT_WEIGHT = 1.0    # wall_weight multiplier (for reference)

wall_dist = np.linspace(0.0, 1.5, 500)


def penalty(dist, margin, power):
    return np.maximum(0.0, margin - dist) ** power


fig, ax = plt.subplots(figsize=(9, 6))
plt.subplots_adjust(left=0.10, bottom=0.30, right=0.97, top=0.93)

(line,) = ax.plot(wall_dist, penalty(wall_dist, INIT_MARGIN, INIT_POWER), lw=2)
(weighted_line,) = ax.plot(
    wall_dist,
    INIT_WEIGHT * penalty(wall_dist, INIT_MARGIN, INIT_POWER),
    lw=1.5,
    ls="--",
    color="tab:orange",
    label="weighted (wall_weight * penalty)",
)
margin_marker = ax.axvline(INIT_MARGIN, color="tab:red", ls=":", lw=1, label="wall_margin")

ax.set_xlabel("wall_dist [m]")
ax.set_ylabel("penalty")
ax.set_title("wall_penalty = max(0, wall_margin - wall_dist) ** wall_power")
ax.grid(True, alpha=0.3)
ax.set_xlim(0.0, 1.5)
ax.set_ylim(0.0, 1.0)
ax.legend(loc="upper right")

ax_margin = plt.axes([0.10, 0.18, 0.80, 0.04])
ax_power = plt.axes([0.10, 0.11, 0.80, 0.04])
ax_weight = plt.axes([0.10, 0.04, 0.80, 0.04])

s_margin = Slider(ax_margin, "wall_margin [m]", 0.0, 1.5, valinit=INIT_MARGIN, valstep=0.01)
s_power = Slider(ax_power, "wall_power", -6.0, 6.0, valinit=INIT_POWER, valstep=0.02)
s_weight = Slider(ax_weight, "wall_weight", 0.0, 500.0, valinit=INIT_WEIGHT, valstep=0.5)


def update(_):
    m = s_margin.val
    p = s_power.val
    w = s_weight.val
    y = penalty(wall_dist, m, p)
    line.set_ydata(y)
    weighted_line.set_ydata(w * y)
    margin_marker.set_xdata([m, m])

    peak = max(y.max(), (w * y).max(), 1e-6)
    ax.set_ylim(0.0, peak * 1.15)
    fig.canvas.draw_idle()


s_margin.on_changed(update)
s_power.on_changed(update)
s_weight.on_changed(update)

update(None)
plt.show()
