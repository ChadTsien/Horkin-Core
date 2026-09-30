# Kinematics library. Currently a placeholder INTERFACE; switch to STATIC
# when first .cpp lands (fk.cpp, jacobian.cpp, ...).

add_library(horkin_kine STATIC
  robot_model.cpp
  ik_dls.cpp
)
target_link_libraries(horkin_kine
  PUBLIC horkin_types
  PRIVATE ${HORKIN_PINOCCHIO}
)
target_include_directories(horkin_kine PUBLIC "${HORKIN_INCLUDE_ROOT}")

# Example when implementing:
# add_library(horkin_kine STATIC
#   model_params.cpp
#   fk.cpp
#   jacobian.cpp
#   ik_dls.cpp
#   scale.cpp
#   limits.cpp
# )
# target_link_libraries(horkin_kine PUBLIC horkin_types)
# target_include_directories(horkin_kine PUBLIC "${HORKIN_INCLUDE_ROOT}")

target_precompile_headers(horkin_kine PRIVATE
  <pinocchio/fwd.hpp>
  <pinocchio/algorithm/kinematics.hpp>
  <pinocchio/parsers/urdf.hpp>
)

if(HORKIN_BUILD_TESTS)
  add_executable(test_fk test/test_fk.cpp)
  target_link_libraries(test_fk PRIVATE horkin_kine)
  target_compile_definitions(test_fk PRIVATE
    HORKIN_TEST_URDF="${CMAKE_SOURCE_DIR}/config/urdf/v6_mc.urdf")
endif()
