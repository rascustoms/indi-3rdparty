# - Find the FLIR Spinnaker SDK includes and library
#
# This module defines
#  SPINNAKER_INCLUDE_DIR, where to find Spinnaker SDK header files
#  SPINNAKER_LIBRARIES, the libraries to link against to use Spinnaker SDK
#  SPINNAKER_FOUND, If false, do not try to use Spinnaker SDK
#  SPINNAKER_VERSION_STRING, e.g. 1.27.0.48
#  SPINNAKER_VERSION_MAJOR, e.g. 1
#  SPINNAKER_VERSION_MINOR, e.g. 27
#  SPINNAKER_VERSION_PATCH, e.g. 48


#=============================================================================
# Copyright 2020 Kieren Rasmussen
#=============================================================================

find_path(SPINNAKER_INCLUDE_DIR spinnaker/Spinnaker.h)
set(SPINNAKER_INCLUDE_DIR "${SPINNAKER_INCLUDE_DIR}/spinnaker")
mark_as_advanced(SPINNAKER_INCLUDE_DIR)

set(SPINNAKER_NAMES ${SPINNAKER_NAMES} Spinnaker)
find_library(SPINNAKER_LIBRARIES NAMES ${SPINNAKER_NAMES} )
mark_as_advanced(SPINNAKER_LIBRARIES)

# TODO Remove hardcode, should be CMAKE functions
set(SPINNAKER_VERSION_MAJOR "1")
set(SPINNAKER_VERSION_MINOR "27")
set(SPINNAKER_VERSION_STRING "${SPINNAKER_VERSION_MAJOR}.${SPINNAKER_VERSION_MINOR}")

# handle the QUIETLY and REQUIRED arguments and set SPINNAKER_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SPINNAKER DEFAULT_MSG SPINNAKER_LIBRARIES SPINNAKER_INCLUDE_DIR)
