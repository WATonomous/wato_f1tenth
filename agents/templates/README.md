# Templates

Starting points for swappable algorithms in the action module. They live outside `src/` so colcon
never builds them in place. Copy one into `src/action/`, rename it, and register it.

| Template | Package | Workflow |
| --- | --- | --- |
| `controller/cpp`, `controller/python` | `my_controller` | `agents/workflows/add-controller.md` |
| `local_planner/cpp`, `local_planner/python` | `my_local_planner` | `agents/workflows/add-local-planner.md` |

Each has a `bringup/` folder with the launch file and the parameter YAML that go into
`action_bringup`. The C++ and Python versions share them: both install `<pkg>_node`.

What you write: `compute()` (controller) or `plan()` (local planner), its `Params`, and its tests.
The node around it already handles the topics, QoS, the dead man switch, picking the global or
local path, rate limiting, and a stop on stale inputs.

Each template builds and passes its tests as it is. A controller holds the car stopped and a local
planner keeps `/local_planner/active` false until you fill in the stub. If you change a template,
build it once in the action dev container (copy it into `src/action/`, build, test, delete it).
