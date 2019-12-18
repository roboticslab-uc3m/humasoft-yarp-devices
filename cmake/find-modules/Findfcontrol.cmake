# Find the fcontrol library and headers.
#
# Sets the following variables:
#
# fcontrol_FOUND        - system has fcontrol
# fcontrol_INCLUDE_DIRS - fcontrol include directories
# fcontrol_LIBRARIES    - fcontrol libraries
# fcontrol_CXX_FLAGS    - fcontrol C++ compiler flags
#
# ...and the following imported targets:
#
# FControl::fcontrol    - fcontrol library
#
# Hints: fcontrol_ROOT environment variable

find_path(fcontrol_INCLUDE_DIR NAMES fcontrol.h
                               HINTS $ENV{fcontrol_ROOT}
                               PATH_SUFFIXES fcontrol)

find_library(fcontrol_LIBRARY NAMES fcontrol
                              HINTS $ENV{fcontrol_ROOT}
                              PATH_SUFFIXES fcontrol)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(fcontrol DEFAULT_MSG fcontrol_INCLUDE_DIR fcontrol_LIBRARY)

if(fcontrol_FOUND)
    set(fcontrol_INCLUDE_DIRS ${fcontrol_INCLUDE_DIR})
    set(fcontrol_LIBRARIES ${fcontrol_LIBRARY})
    set(fcontrol_CXX_FLAGS "-std=c++11")

    if(NOT TARGET FControl::fcontrol)
        add_library(FControl::fcontrol UNKNOWN IMPORTED)

        set_target_properties(FControl::fcontrol PROPERTIES IMPORTED_LOCATION "${fcontrol_LIBRARY}"
                                                            INTERFACE_INCLUDE_DIRECTORIES "${fcontrol_INCLUDE_DIR}"
                                                            INTERFACE_COMPILE_OPTIONS "${fcontrol_CXX_FLAGS}")
    endif()
endif()

mark_as_advanced(fcontrol_INCLUDE_DIR fcontrol_LIBRARY)
