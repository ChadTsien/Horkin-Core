cmake_minimum_required(VERSION 3.16)
project(horkin_core
  LANGUAGES CXX
  DESCRIPTION "Horkin Core"
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(HorkinOptions)
include(HorkinPinocchio)

# ---------------------------------------------------------------------------
# 契约层（无实现，先可被任何人 link）
# ---------------------------------------------------------------------------
add_subdirectory(types)
add_subdirectory(ports)

# ---------------------------------------------------------------------------
# 纯逻辑库（禁止依赖 hal / sim / ROS）
# ---------------------------------------------------------------------------
add_subdirectory(kine)
add_subdirectory(dyn)
add_subdirectory(signal)
add_subdirectory(control)
add_subdirectory(runtime)

# ---------------------------------------------------------------------------
# 后端与装配根（按 option 裁剪）
# ---------------------------------------------------------------------------
if(HORKIN_BUILD_HAL)
  add_subdirectory(hal)
  add_subdirectory(app_real)
endif()

if(HORKIN_BUILD_SIM)
  add_subdirectory(sim)
  add_subdirectory(app_sim)
endif()

if(HORKIN_BUILD_TOOLS)
  add_subdirectory(tools)
endif()
