# Early smoke for force types / IForceSensor stub.
# Becomes useful after types/wrench.hpp and ports/force_sensor.hpp exist.

add_executable(smoke_wrench main_placeholder.cpp)
target_link_libraries(smoke_wrench PRIVATE horkin_ports)
target_include_directories(smoke_wrench PRIVATE "${HORKIN_INCLUDE_ROOT}")
