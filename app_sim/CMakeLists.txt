# Simulation assembly root. Only place that wires control + sim.

add_executable(app_sim main_placeholder.cpp)
target_link_libraries(app_sim PRIVATE horkin_runtime horkin_sim)
target_include_directories(app_sim PRIVATE "${HORKIN_INCLUDE_ROOT}")
