# Backend ports (header-only INTERFACE). Depends only on horkin_types.
# Add headers later: force_sensor.hpp, joint_driver.hpp, clock.hpp

add_library(horkin_ports INTERFACE)
target_link_libraries(horkin_ports INTERFACE horkin_types)
target_include_directories(horkin_ports INTERFACE "${HORKIN_INCLUDE_ROOT}")
