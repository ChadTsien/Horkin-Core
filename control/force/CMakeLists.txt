# Hybrid force/position control. Placeholder INTERFACE until first .cpp.

add_library(horkin_control_force INTERFACE)
target_link_libraries(horkin_control_force INTERFACE horkin_types)
target_include_directories(horkin_control_force INTERFACE "${HORKIN_INCLUDE_ROOT}")

# Example when implementing:
# add_library(horkin_control_force STATIC
#   admittance_axis.cpp
#   admittance_6d.cpp
#   selection.cpp
#   contact_fsm.cpp
#   safety_limit.cpp
# )
# target_link_libraries(horkin_control_force PUBLIC horkin_types)
# # Prefer calling kine/dyn as free functions; link when needed:
# # target_link_libraries(horkin_control_force PUBLIC horkin_kine horkin_dyn)
# target_include_directories(horkin_control_force PUBLIC "${HORKIN_INCLUDE_ROOT}")
