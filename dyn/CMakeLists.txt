# Dynamics library. Placeholder INTERFACE until first .cpp.

add_library(horkin_dyn INTERFACE)
target_link_libraries(horkin_dyn INTERFACE horkin_kine)
target_include_directories(horkin_dyn INTERFACE "${HORKIN_INCLUDE_ROOT}")

# Example when implementing:
# add_library(horkin_dyn STATIC
#   inertial_params.cpp
#   rnea.cpp
#   gravity.cpp
#   mass.cpp
#   friction.cpp
# )
# target_link_libraries(horkin_dyn PUBLIC horkin_kine)
# target_include_directories(horkin_dyn PUBLIC "${HORKIN_INCLUDE_ROOT}")

if(HORKIN_BUILD_TESTS)
  # add_executable(test_rnea test/test_rnea.cpp)
  # target_link_libraries(test_rnea PRIVATE horkin_dyn)
endif()
