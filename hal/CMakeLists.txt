# Real hardware adapters. Implements ports. Must NOT link sim or control formulas.

add_library(horkin_hal INTERFACE)
target_link_libraries(horkin_hal INTERFACE horkin_ports)
target_include_directories(horkin_hal INTERFACE "${HORKIN_INCLUDE_ROOT}")

# Example when implementing:
# add_library(horkin_hal STATIC
#   can_socket.cpp
#   motor_mit.cpp
#   motor_j3_trans.cpp
#   sensor_hkv.cpp
#   clock_realtime.cpp
# )
# target_link_libraries(horkin_hal PUBLIC horkin_ports)
# target_include_directories(horkin_hal PUBLIC "${HORKIN_INCLUDE_ROOT}")
