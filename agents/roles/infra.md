# Infra

For changes to `watod`, `watod_scripts/`, `docker/`, `modules/`, CI, or the agent docs.
Infra files are coupled: changing one usually means changing several. **Ask before running any
build or `up`** (`AGENTS.md` rule 1).

## Files that change together

Adding a module means copying an existing one end to end (the `sim` module is the latest example):

- `docker/<module>.Dockerfile`
- the `_source`, `_deps` and `_bringup` services in `modules/docker-compose.yaml`
- matching entries in `modules/docker-compose.{dep,dev,test,watcloud}.yaml`
- `<MODULE>_IMAGE` in `watod_scripts/watod-setup-env.sh`
- `get_all_modules()` in `watod`
- the module list in `watod_scripts/watod-setup-dev-env.sh`
- the module lists in `.github/templates/` (`check_src_changes`, `docker_context`)
- the module list comment in `watod-config.sh`

Other rules:

- Message packages are copied into `docker/infrastructure.Dockerfile` and added to
  `infrastructure_bringup/package.xml`, so Foxglove and `ros2 bag` can decode them.
- Defaults go in `watod_scripts/watod-setup-env.sh`. `watod-config.sh` only documents them.
- New environment variables for launch files (like `PLATFORM`) are written to `.env` in
  `watod-setup-env.sh` and passed in the bringup's `environment:` in `docker-compose.yaml`.
- f1tenth departs from the template in: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
  `ROS_DOMAIN_ID=0`, no Zenoh router, log viewer or topic healthchecker, root-anchored
  `/maps/` etc. in `.gitignore`, and pre-commit excludes for vendored packages.

## Template updates

The template is merged in as a git remote, so its updates come in with a plain merge:

```bash
git fetch template
git merge template/main
```

Expect conflicts in the files listed above. Keep the f1tenth departures, and keep
`!.claude/settings.json` and `!.claude/skills/` in `.gitignore` (the template ignores `.claude/*`).

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

- Base images are built locally with `./watod build-base` (`watod_scripts/watod-build-base.sh`,
  layers in `docker/base/`): Jazzy CPU (amd64 and arm64), x86 CUDA 12.9 + cuDNN + TensorRT 10.16,
  Jetson CUDA 12.8 + cuDNN 9.8 + TensorRT 10.7. GPU modules pick their base per architecture
  with `FROM base_${TARGETARCH}` stages.
- Don't use vendor images as-is (`dustynv/*`, `l4t-jetpack`), and don't pin to JetPack's bundled
  CUDA/TensorRT versions. Rebuild TensorRT engines on the target when versions change.
- Don't build `colcon` workspaces under QEMU. arm64 builds run on the self-hosted Jetson runner (step 5).

## Agent docs

- `AGENTS.md` stays under 200 lines and only indexes; details go in `agents/`.
- `CLAUDE.md` only imports `AGENTS.md`. `.claude/skills/*` point to `agents/workflows/*` and add
  no rules of their own.
- When infra changes what a doc says (commands, paths, module names), update the doc in the same change.
