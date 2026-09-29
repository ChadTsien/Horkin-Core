# Source this after installing deps (or from ~/.bashrc):
#   source scripts/env.sh
#
# robotpkg installs Pinocchio 2.7 under /opt/openrobots.

if [ -d /opt/openrobots ]; then
  export PATH="/opt/openrobots/bin${PATH:+:$PATH}"
  export PKG_CONFIG_PATH="/opt/openrobots/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
  export LD_LIBRARY_PATH="/opt/openrobots/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export CMAKE_PREFIX_PATH="/opt/openrobots${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
fi
