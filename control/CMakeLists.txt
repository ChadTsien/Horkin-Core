# Control umbrella: force (now), motion/traj (later).

add_subdirectory(force)

# Optional later:
# add_subdirectory(motion)
# add_subdirectory(traj)

# Convenience INTERFACE aggregating control libs currently present.
add_library(horkin_control INTERFACE)
target_link_libraries(horkin_control INTERFACE horkin_control_force)
