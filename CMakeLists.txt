cmake_minimum_required(VERSION 2.4.7)
PROJECT(indi_cam90_ccd C CXX)
SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread" )
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake_modules/")

include(GNUInstallDirs)

set (CAM90_VERSION_MAJOR 1)
set (CAM90_VERSION_MINOR 0)

add_library(cam90 libcam90.c)

find_package(CFITSIO REQUIRED)
find_package(INDI REQUIRED)
find_package(ZLIB REQUIRED)
find_package ( LibFTDI1 NO_MODULE REQUIRED )
include ( ${LIBFTDI_USE_FILE} )

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_cam90_ccd.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_cam90_ccd.xml )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
#include_directories ( ${LIBFTDI_USE_FILE} )
#include_directories(${LIBFTD2XX_INCLUDE_DIR})

include_directories( ${CFITSIO_INCLUDE_DIR})

############# CAM90 CCD ###############
if (CFITSIO_FOUND)

set(cam90ccd_SRCS
	${CMAKE_CURRENT_SOURCE_DIR}/cam90_ccd.cpp
)

add_executable(indi_cam90_ccd ${cam90ccd_SRCS})

target_link_libraries(indi_cam90_ccd cam90 ${LIBFTDI_LIBRARIES} ${INDI_DRIVER_LIBRARIES} ${CFITSIO_LIBRARIES} m ${ZLIB_LIBRARY})

install(TARGETS indi_cam90_ccd RUNTIME DESTINATION bin)

endif (CFITSIO_FOUND)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_cam90_ccd.xml DESTINATION ${INDI_DATA_DIR})
