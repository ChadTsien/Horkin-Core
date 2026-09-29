# Simulation backend. Implements ports; may use kine/dyn for plant.
# Must NOT link hal.

add_library(horkin_sim INTERFACE)
target_link_libraries(horkin_sim INTERFACE horkin_ports horkin_kine horkin_dyn)
target_include_directories(horkin_sim INTERFACE "${HORKIN_INCLUDE_ROOT}")

# Example when implementing:
# add_library(horkin_sim STATIC
#   plant/...
#   contact/...
#   joint_sim.cpp
#   sensor_sim.cpp
#   servo_sim.cpp
#   world.cpp
# )
# target_link_libraries(horkin_sim PUBLIC horkin_ports horkin_kine horkin_dyn)
# target_include_directories(horkin_sim PUBLIC "${HORKIN_INCLUDE_ROOT}")
