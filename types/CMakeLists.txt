# Zero-dependency POD types (header-only INTERFACE library).
# Add headers here later: time.hpp, wrench.hpp, joint.hpp, ...

add_library(horkin_types INTERFACE)
target_include_directories(horkin_types INTERFACE "${HORKIN_INCLUDE_ROOT}")
