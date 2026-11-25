# Build: where, how, and how to keep it fast

What to build on which machine, which command a change needs, and what keeps rebuilds fast.
**Ask before any `./watod build|up|down`** (`AGENTS.md` rule 1); on the car, also read
`remote-car.md`.

## Images

| Image | Built by | Rebuild when |
| --- | --- | --- |
| Base (`ghcr.io/watonomous/wato_f1tenth/base:*`): Jazzy, plus CUDA + TensorRT where there is a GPU | `./watod build-base` | `docker/base/` changes, or once on a new machine |
| Module `source` / `deps` / bringup images (`:source_<TAG>`, `:deps_<hash>`, `:<TAG>`) | `./watod -m <module> build` | anything in that module's `src/`, its Dockerfile or `template.Dockerfile` changes |
| Module test image (`:test_<TAG>`, the `build` stage) | `./watod test <module>` | built on demand |
| Dev image (`<module>:dev`, tagged `:dev_<hash>`) | `./watod -m <module>:dev up` | only when that module's Dockerfile, `template.Dockerfile` or `package.xml` changes (source is mounted) |
| Lint image (`lint:lint_<hash>`, `docker/lint.Dockerfile`) | `./watod lint` (auto-built) | only when `docker/lint.Dockerfile` changes; hook envs cached in `watod_pre_commit_cache` volume |

## Image tagging across branches (`IMAGE_TAG_MODE`)

By default (`IMAGE_TAG_MODE=auto`, implemented in `watod_scripts/watod-image-tags.sh`), branches share
images instead of creating a full duplicate set per branch:

- **Base images:** canonical shared tags (`jazzy-ubuntu24.04`, etc.) when `docker/base/` matches `main`;
  automatically suffixed `-<branch>` (or `-dirty-<hash>` on `main`) when a branch edits `docker/base/`.
- **Module `deps` and `dev` images:** content-hashed per module (`:deps_<hash>`, `:dev_<hash>` over
  Dockerfiles, `docker/config/`, and that module's `package.xml` files). Unchanged environments are
  reused across branches with no rebuilds.
- **Module deploy/test/source images:** floating `:local` tag (or `:local-<COMPOSE_PROJECT_NAME>` when
  `COMPOSE_PROJECT_NAME` is customized on a shared host like the Jetson). `watod up` warns if `:local`
  was built from a different branch or checkout. CI always uses the sanitized branch name.

Override in `watod-config.local.sh` with `IMAGE_TAG_MODE="branch"` (tag everything by branch),
`BASE_TAG_MODE="shared|branch"`, `BASE_TAG_SUFFIX="..."`, or `TAG="..."`.

## Which machine

- **PC (x86):** sim work, and any build for the PC. `build-base` builds the CPU base, plus the
  CUDA 12.9 + TensorRT base if the PC has an NVIDIA GPU.
- **Car (Jetson, arm64):** anything that runs on the car is built on the car. Never cross-build
  arm64 images under QEMU (`roles/infra.md`). `build-base` there builds the Jetson CUDA 12.8 base.

## Which command a change needs

| You changed | Run |
| --- | --- |
| Code in one package, while developing | a dev container: `colcon build --packages-select <pkg>` (seconds; see below) |
| Code, to test or run the deploy image | `./watod -m <module> build` |
| A `package.xml` dependency | `./watod -m <module> build` (the dependency install re-runs) |
| `docker/<module>.Dockerfile` or `template.Dockerfile` | `./watod -m <module> build` for every module it affects |
| `docker/base/` | `./watod build-base`, then the modules |

`--no-cache` (`./watod -m <module> build --no-cache`) is almost never needed: only if you suspect a
stale cache, for example an apt mirror that has since fixed a broken package. It re-runs every step
of the module's images without using the cache.

**Fastest loop:** list the module as `"<module>:dev"` in `ACTIVE_MODULES`, `./watod up`, then
`./watod -t <module>_bringup_dev` and `colcon build --packages-select <pkg>` inside it
(`build-and-test.md`). Your source is mounted, so only changed files compile.

## On the car

- **Plugged into power.** A full-CPU build browned the Jetson out on battery (2026-10-27).
- **One module at a time.** Several images at once run the 8 GB Jetson out of memory:
  `for m in infrastructure interfacing world_modeling action; do ./watod -m $m build || break; done`.
  Never `-m "a b"` in one string: that silently builds only infrastructure.
- **Detached over SSH**, with a log and an exit marker (`remote-car.md`, "Long commands").
- **Disk:** check `df -h /` before a build; images and the build cache take tens of GB.

## Why rebuilds are fast, and how not to break it

Each module builds `source` and `deps` images first, then its bringup image starts `FROM` them by tag.
Three things keep that cached. Keep them when you edit `watod` or the Dockerfiles:

1. **No provenance attestations** (`watod` exports `BUILDX_NO_DEFAULT_ATTESTATIONS=1`). With them,
   every export got a new image ID, and everything built `FROM` it started over.
2. **The source is copied in `build`, not `rosdep_install`** in `template.Dockerfile`, so a code
   change skips dependency install/cleanup and never invalidates the `develop` (`:dev`) image. Don't
   add steps after the copy that don't need the source.
3. **ccache** in a BuildKit cache mount, shared by every module: unchanged files come from the cache.
   The build log prints its hit rate after `colcon build` (`ccache --show-stats`).

Also: the dependency scan runs without `apt-get update` (`rosdep -s` doesn't need the apt index), and
there is no GHCR `cache_from` until step 5 publishes images.

Measured on the laptop (RTX 5070, 24 threads, fast internet), 2026-10-29:

| | infrastructure | interfacing | world_modeling | action | sim |
| --- | --- | --- | --- | --- | --- |
| Cold build, before | 139 s | 734 s (slow mirror) | 174 s | 144 s | 241 s |
| Rebuild with no change, before | 135 s (everything re-ran) | | | | |
| Rebuild with no change, now | 4 s | 2 s | 2 s | 2 s | 2 s |
| One-line C++ change, now | | | | 16 s | |

`build-base` (CPU + GPU bases) took 365 s with the upstream CUDA layers already pulled.

On the car (Jetson Orin, 8 GB, one module at a time, plugged in), 2026-10-29:

| | infrastructure | interfacing | world_modeling | action |
| --- | --- | --- | --- | --- |
| First build after the change (fills ccache) | 212 s | 181 s | 185 s | 114 s |
| Rebuild with no change | 1 s | 2 s | 3 s | 2 s |
| One-line C++ change | | | | 23 s |

Before, every rebuild on the car re-ran everything (2-4 min per module).

## Clearing space and caches

- `./watod prune` (pass `-n`/`--dry-run` to preview, `--bases` for stale branch-suffixed base images)
  removes unused `wato_f1tenth` tags while keeping active `.env` and container images intact.
- `docker builder prune` (or `./watod prune --cache`) clears **every project's** build cache on the
  machine, including ccache, so the next build of everything is cold. Ask first if the machine is shared.
- `docker image prune` removes dangling images only, which is safe.
- The f1tenth base images are only local until step 5. Deleting them means running
  `./watod build-base` again.

## Reading the build output

- `exporting to image` takes 40–60 s per image the first time, and nothing when cached.
- An apt download stuck at `DL:0B` is a stalled mirror connection. Stop the build and run it again;
  nothing half-installed is cached.
- On Wi-Fi with IPv6 DNS, `Temporary failure resolving`: see `deploy-to-car.md`, "Known traps".
