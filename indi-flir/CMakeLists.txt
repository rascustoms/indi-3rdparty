cmake_minimum_required(VERSION 2.4.7)
PROJECT(indi_flir CXX C)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")
include(GNUInstallDirs)

find_package(CFITSIO REQUIRED)
find_package(INDI REQUIRED)
find_package(ZLIB REQUIRED)
find_package(SPINNAKER REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${CFITSIO_INCLUDE_DIR})
include_directories( ${SPINNAKER_INCLUDE_DIR})

include(CMakeCommon)

############# GENERIC CCD ###############
set(FLIR_SRCS
	${CMAKE_CURRENT_SOURCE_DIR}/src/flir_ccd.cpp
    )

IF (UNITY_BUILD)
    ENABLE_UNITY_BUILD(indiflir FLIR_SRCS 10 cpp)
ENDIF ()

add_executable(indi_flir_ccd ${FLIR_SRCS})

target_link_libraries(indi_flir_ccd ${INDI_LIBRARIES} ${SPINNAKER_LIBRARIES} ${CFITSIO_LIBRARIES} m ${ZLIB_LIBRARY})

install(TARGETS indi_flir_ccd RUNTIME DESTINATION bin)

install(FILES indi_flir_ccd.xml DESTINATION ${INDI_DATA_DIR})
