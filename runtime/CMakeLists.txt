# RT loop, SPSC, watchdog. Placeholder INTERFACE until first .cpp.
# May depend on ports + control + signal + kine + dyn once implemented.

add_library(horkin_runtime INTERFACE)
target_link_libraries(horkin_runtime INTERFACE
  horkin_ports
  horkin_control
  horkin_signal
  horkin_kine
  horkin_dyn
)
target_include_directories(horkin_runtime INTERFACE "${HORKIN_INCLUDE_ROOT}")

# Example when implementing:
# add_library(horkin_runtime STATIC
#   rt_loop.cpp
#   watchdog.cpp
# )
# target_link_libraries(horkin_runtime PUBLIC
#   horkin_ports
#   horkin_control
#   horkin_signal
#   horkin_kine
#   horkin_dyn
# )
# target_include_directories(horkin_runtime PUBLIC "${HORKIN_INCLUDE_ROOT}")
