# Infra

For changes to `watod`, `watod_scripts/`, `docker/`, `modules/`, CI, or the agent docs.
Infra files are coupled: changing one usually means changing several. **Ask before running any
build or `up`** (`AGENTS.md` rule 1).

## Today: files that change together

| Change | Files |
| --- | --- |
| Add a module | `modules/docker-compose.<module>.yaml`, `docker/<module>/<module>.Dockerfile`, the `<MODULE>_IMAGE` line in `watod_scripts/watod-setup-docker-env.sh`, the module list comment in `watod-config.sh` |
| Add a message package | the package under `src/wato_msgs/`, plus the Foxglove image build so the bridge can decode it |
| New car device | `device_cgroup_rules` and volumes in the car branch's `docker-compose.robot.yaml` |
| Change a port | `watod_scripts/watod-setup-docker-env.sh` (ports are derived from `uid * 20`) |

Old traps to know about: `PLATFORM=arm64` only writes `BASE_IMAGE_OVERRIDE`, which the car
branches' Dockerfiles ignore. The default `REGISTRY_URL` still points at `wato_asd_training`.

## Target: files that change together (template, after step 2)

Adding a module means copying an existing one end to end:

- `docker/<module>.Dockerfile`
- the `_source`, `_deps` and `_bringup` services in `modules/docker-compose.yaml`
- matching entries in `modules/docker-compose.{dep,dev,test,watcloud}.yaml`
- `<MODULE>_IMAGE` in `watod_scripts/watod-setup-env.sh`
- `get_all_modules()` in `watod`
- the module list in `watod_scripts/watod-setup-dev-env.sh`
- the module lists in `.github/templates/` (`check_src_changes`, `docker_context`)

Other template rules:

- Message packages are also copied into `docker/infrastructure.Dockerfile`.
- Defaults go in `watod_scripts/watod-setup-env.sh`. CI blocks PRs that change `watod-config.sh`.
- f1tenth overrides: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, a fixed `ROS_DOMAIN_ID`, and no
  `rmw_zenohd` in the infrastructure launch.
- The template's `.gitignore` ignores `.claude/*` except `CLAUDE.md`. When merging it, add
  `!.claude/settings.json` and `!.claude/skills/`, or the agent setup disappears from git.

## Platform detection (step 3)

`watod` sets `PLATFORM` (car or sim) and `GPU` (nvidia or none):

| Machine | Detected by | Compose overlay |
| --- | --- | --- |
| Jetson | `/etc/nv_tegra_release` exists | `docker-compose.jetson.yaml` (`runtime: nvidia`, devices) |
| PC with NVIDIA GPU | `nvidia-smi` works and Docker has the `nvidia` runtime | `docker-compose.gpu.yaml` |
| PC without GPU | neither | none |

Keep detection in one function in `watod`, and allow `PLATFORM` / `GPU` overrides in
`watod-config.local.sh`.

## Images

- Base images: plain Ubuntu 24.04 + Jazzy (template `base:jazzy-ubuntu24.04`), built for amd64
  and arm64. Jetson image: the same base plus CUDA 12.8, cuDNN 9 and TensorRT 10 for Ubuntu 24.04.
- Don't use vendor images as-is (`dustynv/*`, `l4t-jetpack`), and don't pin to JetPack's bundled
  CUDA/TensorRT versions. Rebuild TensorRT engines on the target when versions change.
- Don't build `colcon` workspaces under QEMU. arm64 builds run on the self-hosted Jetson runner (step 5).

## Agent docs

- `AGENTS.md` stays under 200 lines and only indexes; details go in `agents/`.
- `CLAUDE.md` only imports `AGENTS.md`. `.claude/skills/*` point to `agents/workflows/*` and add
  no rules of their own.
- When infra changes what a doc says (commands, paths, module names), update the doc in the same change.
