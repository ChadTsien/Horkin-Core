# Pinocchio 2.7 (robotpkg prefix /opt/openrobots). Reject 3.x / 4.x.

if(EXISTS "/opt/openrobots")
  list(PREPEND CMAKE_PREFIX_PATH "/opt/openrobots")
endif()

find_package(pinocchio 2.7 QUIET)

if(NOT pinocchio_FOUND)
  message(FATAL_ERROR
    "Pinocchio 2.7 not found.\n"
    "On Ubuntu x86_64:\n"
    "  ./scripts/setup_deps.sh\n"
    "  source scripts/env.sh\n"
    "Then re-run cmake. Prefix is /opt/openrobots "
    "(or pass -DCMAKE_PREFIX_PATH=/opt/openrobots)."
  )
endif()

if(pinocchio_VERSION VERSION_GREATER_EQUAL 3)
  message(FATAL_ERROR
    "Need Pinocchio 2.7.x, found ${pinocchio_VERSION}. "
    "Do not install robotpkg-pinocchio without a version pin "
    "(candidate is often 3.x/4.x). Re-run ./scripts/setup_deps.sh"
  )
endif()

if(TARGET pinocchio::pinocchio)
  set(HORKIN_PINOCCHIO pinocchio::pinocchio)
else()
  set(HORKIN_PINOCCHIO ${pinocchio_LIBRARIES})
endif()

message(STATUS "Pinocchio ${pinocchio_VERSION}")
