# Horkin Core CMake options — included from root CMakeLists.txt

option(HORKIN_BUILD_SIM   "Build simulation backend and app_sim" ON)
option(HORKIN_BUILD_HAL   "Build real HAL and app_real"         ON)
option(HORKIN_BUILD_TOOLS "Build tools (smoke / identify / replay)" ON)
option(HORKIN_BUILD_TESTS "Build unit tests under kine/dyn"     OFF)

# Include root so every target can use:
#   #include "types/...."
#   #include "ports/...."
set(HORKIN_INCLUDE_ROOT "${CMAKE_SOURCE_DIR}" CACHE PATH "Public include root for Horkin Core")
