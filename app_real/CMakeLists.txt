# Real-robot assembly root. Only place that wires control + hal.

# Placeholder executable so the target graph configures cleanly.
# Replace with real main.cpp / wiring.cpp / config.cpp later.

add_executable(app_real main_placeholder.cpp)
target_link_libraries(app_real PRIVATE horkin_runtime horkin_hal)
target_include_directories(app_real PRIVATE "${HORKIN_INCLUDE_ROOT}")
