cmake_minimum_required(VERSION 3.12.0)
project(
   GalileoSDK
   VERSION 1.2.3
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpic")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include(GNUInstallDirs)

find_package(Boost REQUIRED COMPONENTS thread filesystem regex)
find_package(PkgConfig REQUIRED)
pkg_search_module(ROSCPP REQUIRED roscpp)
pkg_search_module(GEOMETRY_MSGS REQUIRED geometry_msgs)


include_directories(
   ${PROJECT_SOURCE_DIR}/include
   ${ROSCPP_INCLUDE_DIRS}
   ${GEOMETRY_MSGS_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIRS}
   ${PROJECT_SOURCE_DIR}/../include
   ${PROJECT_SOURCE_DIR}/../iot
   ${PROJECT_SOURCE_DIR}/../IotWrapper
)

if("${BUILD_SHARED_LIBS}" STREQUAL OFF)
   set(SDK_NAME "libGalileoSDK.a")
else()
   set(BUILD_SHARED_LIBS ON)
   set(SDK_NAME "libGalileoSDK.so")
endif()

if(ANDROID)
add_library(GalileoSDK
   ${PROJECT_SOURCE_DIR}/GalileoSDK.cpp
   ${PROJECT_SOURCE_DIR}/ifaddrs1.c
   ${PROJECT_SOURCE_DIR}/HttpConnection.cpp
   ${PROJECT_SOURCE_DIR}/iot.cpp
)
else()
add_library(GalileoSDK
   ${PROJECT_SOURCE_DIR}/GalileoSDK.cpp
   ${PROJECT_SOURCE_DIR}/HttpConnection.cpp
   ${PROJECT_SOURCE_DIR}/iot.cpp
)
endif()

set_target_properties(GalileoSDK PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(GalileoSDK PROPERTIES SOVERSION 1)
set_target_properties(GalileoSDK PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(GalileoSDK
   ${ROSCPP_LINK_LIBRARIES}
   ${GEOMETRY_MSGS_LINK_LIBRARIES}
   ${Boost_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/../iot/build/libiot.a
   ${PROJECT_SOURCE_DIR}/../IotWrapper/build/libIotWrapper.a
)

install(
   TARGETS GalileoSDK
   EXPORT GalileoSDK
   RUNTIME DESTINATION bin
   LIBRARY DESTINATION lib
   ARCHIVE DESTINATION lib
)

install(
   DIRECTORY ${PROJECT_SOURCE_DIR}/include/ 
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.h"
)

install(
   DIRECTORY ${PROJECT_SOURCE_DIR}/include/ 
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.hpp"
)

install(
   DIRECTORY ${PROJECT_SOURCE_DIR}/../include/galileo_serial_server
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.h"
)

install(
   DIRECTORY ${PROJECT_SOURCE_DIR}/../iot/
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.h"
)

install(
   DIRECTORY ${PROJECT_SOURCE_DIR}/../IotWrapper/
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.h"
)

# install pkg-config file
configure_file(
   "pkg-config/galileo-sdk.pc.in"
   "pkg-config/galileo-sdk.pc"
   @ONLY
)

install(
   FILES "${CMAKE_CURRENT_BINARY_DIR}/pkg-config/galileo-sdk.pc"
   DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
)

# install cmake files
set(INCLUDE_INSTALL_DIR "include")

include(CMakePackageConfigHelpers)
configure_package_config_file(GalileoSDKConfig.cmake.in
   ${CMAKE_CURRENT_BINARY_DIR}/GalileoSDKConfig.cmake
   INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/GalileoSDK
   PATH_VARS INCLUDE_INSTALL_DIR
)

write_basic_package_version_file(
   ${CMAKE_CURRENT_BINARY_DIR}/GalileoSDKConfigVersion.cmake
   VERSION ${PROJECT_VERSION}
   COMPATIBILITY SameMajorVersion 
)

install(
   FILES ${CMAKE_CURRENT_BINARY_DIR}/GalileoSDKConfig.cmake
   ${CMAKE_CURRENT_BINARY_DIR}/GalileoSDKConfigVersion.cmake
   DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/GalileoSDK
)