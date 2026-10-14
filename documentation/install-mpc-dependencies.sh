#!/usr/bin/env bash
# Run with: bash documentation/install-mpc-dependencies.sh
set -euo pipefail

MPC_DEPS_ROOT="$HOME/.local/opt/wato-mpc"
mkdir -p "$MPC_DEPS_ROOT"

ACADOS_REVISION=988136c9ed5871246b9f25005a540600b2daf3d2
if [[ ! -d "$MPC_DEPS_ROOT/acados/.git" ]]; then
  git clone --no-checkout https://github.com/acados/acados.git "$MPC_DEPS_ROOT/acados"
  git -C "$MPC_DEPS_ROOT/acados" checkout --detach "$ACADOS_REVISION"
  git -C "$MPC_DEPS_ROOT/acados" submodule update --init --recursive
fi
if [[ "$(git -C "$MPC_DEPS_ROOT/acados" rev-parse HEAD)" != "$ACADOS_REVISION" ]]; then
  echo "Existing acados differs from tested revision $ACADOS_REVISION; keeping it unchanged." >&2
  exit 1
fi

cmake -S "$MPC_DEPS_ROOT/acados" \
  -B "$MPC_DEPS_ROOT/acados/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBLASFEO_TARGET=GENERIC \
  -DHPIPM_TARGET=GENERIC \
  -DCMAKE_INSTALL_PREFIX="$MPC_DEPS_ROOT/acados"

cmake --build "$MPC_DEPS_ROOT/acados/build" --parallel 4
cmake --install "$MPC_DEPS_ROOT/acados/build"

python3 -m venv "$MPC_DEPS_ROOT/venv"
"$MPC_DEPS_ROOT/venv/bin/python" -m pip install --upgrade pip
"$MPC_DEPS_ROOT/venv/bin/python" -m pip install \
  "casadi==3.8.1" \
  -e "$MPC_DEPS_ROOT/acados/interfaces/acados_template"

ACADOS_SOURCE_DIR="$MPC_DEPS_ROOT/acados" "$MPC_DEPS_ROOT/venv/bin/python" -c \
  'from acados_template.utils import get_tera; get_tera()'

cat <<'ACTIVATION'

Installation finished. ROS simulation also needs these distro packages:

sudo apt install "ros-$ROS_DISTRO-ackermann-msgs" "ros-$ROS_DISTRO-diagnostic-updater"

Optional terminal activation (the prototype CMake uses these paths directly):

source "$HOME/.local/opt/wato-mpc/venv/bin/activate"
export ACADOS_SOURCE_DIR="$HOME/.local/opt/wato-mpc/acados"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ACTIVATION
