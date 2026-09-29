#!/usr/bin/env bash
# Install the pinned C++ deps for Horkin Core (Ubuntu amd64).
# Pinocchio 2.7.0 + hpp-fcl 2.4.4 via robotpkg. Do not install 3.x / 4.x.
#
#   ./scripts/setup_deps.sh
#   source scripts/env.sh
#   cmake -S . -B build

set -euo pipefail

PINOCCHIO_VER="2.7.0"
HPP_FCL_VER="2.4.4"
PREFIX="/opt/openrobots"

if [[ "$(uname -m)" != "x86_64" ]]; then
  echo "This script pins robotpkg amd64 binaries (Ubuntu x86_64)." >&2
  echo "On aarch64/Jetson, install matching Pinocchio ${PINOCCHIO_VER} for that arch." >&2
  exit 1
fi

if ! command -v apt-get >/dev/null; then
  echo "Need apt (Debian/Ubuntu)." >&2
  exit 1
fi

if ! command -v sudo >/dev/null; then
  echo "Need sudo to install packages." >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CODENAME="$(lsb_release -cs)"

echo "==> Ubuntu ${CODENAME}  Pinocchio ${PINOCCHIO_VER}  hpp-fcl ${HPP_FCL_VER}"

if [[ ! -f /etc/apt/keyrings/robotpkg.asc ]]; then
  echo "==> Adding robotpkg apt key"
  sudo mkdir -p /etc/apt/keyrings
  curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | sudo tee /etc/apt/keyrings/robotpkg.asc >/dev/null
fi

if [[ ! -f /etc/apt/sources.list.d/robotpkg.list ]]; then
  echo "==> Adding robotpkg apt source"
  echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub ${CODENAME} robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list >/dev/null
fi

echo "==> Pinning apt versions (block 3.x/4.x upgrades)"
sudo cp "${SCRIPT_DIR}/apt-preferences-horkin" /etc/apt/preferences.d/horkin

sudo apt-get update

if ! apt-cache madison robotpkg-pinocchio | grep -q " ${PINOCCHIO_VER} "; then
  echo "robotpkg-pinocchio ${PINOCCHIO_VER} is not in the apt index for ${CODENAME}." >&2
  apt-cache policy robotpkg-pinocchio >&2 || true
  exit 1
fi

echo "==> Installing robotpkg-pinocchio=${PINOCCHIO_VER} robotpkg-hpp-fcl=${HPP_FCL_VER}"
sudo apt-get install -y \
  "robotpkg-pinocchio=${PINOCCHIO_VER}" \
  "robotpkg-hpp-fcl=${HPP_FCL_VER}"

sudo apt-mark hold robotpkg-pinocchio robotpkg-hpp-fcl

echo "==> Verifying"
export PKG_CONFIG_PATH="${PREFIX}/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
GOT="$(pkg-config --modversion pinocchio)"
if [[ "${GOT}" != "${PINOCCHIO_VER}" ]]; then
  echo "pkg-config reported pinocchio ${GOT}, expected ${PINOCCHIO_VER}" >&2
  exit 1
fi
test -f "${PREFIX}/lib/cmake/pinocchio/pinocchioConfig.cmake"

echo
echo "OK: Pinocchio ${GOT} at ${PREFIX}"
echo "In this shell (or add to ~/.bashrc):"
echo "  source ${SCRIPT_DIR}/env.sh"
echo "Then:"
echo "  cmake -S . -B build"
echo "  cmake --build build"
