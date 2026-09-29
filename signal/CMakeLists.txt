# Signal processing: wrench transform, gravity comp, filters.
# Placeholder INTERFACE until first .cpp.

add_library(horkin_signal INTERFACE)
target_link_libraries(horkin_signal INTERFACE horkin_types)
target_include_directories(horkin_signal INTERFACE "${HORKIN_INCLUDE_ROOT}")

# Example when implementing:
# add_library(horkin_signal STATIC
#   wrench_transform.cpp
#   gravity_comp.cpp
#   identify_ls.cpp
#   filter_median.cpp
#   filter_butterworth.cpp
#   diff_lpf.cpp
# )
# target_link_libraries(horkin_signal PUBLIC horkin_types)
# target_include_directories(horkin_signal PUBLIC "${HORKIN_INCLUDE_ROOT}")
