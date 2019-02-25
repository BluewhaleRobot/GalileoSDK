cmake_minimum_required(VERSION 3.1.0)
project(
   GalileoSDK
   VERSION 1.0.0
   LANGUAGES CXX
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include(GNUInstallDirs)

find_package(PkgConfig REQUIRED)
pkg_search_module(JSONCPP REQUIRED jsoncpp)
pkg_search_module(ROSCPP REQUIRED roscpp)

include_directories(
   ${PROJECT_SOURCE_DIR}/include
   ${JSONCPP_INCLUDE_DIRS}
   ${ROSCPP_INCLUDE_DIRS}
   ${PROJECT_SOURCE_DIR}/../include
)
set(BUILD_SHARED_LIBS ON)
add_library(GalileoSDK
   ${PROJECT_SOURCE_DIR}/GalileoSDK.cpp
)

set_target_properties(GalileoSDK PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(GalileoSDK PROPERTIES SOVERSION 0)

target_link_libraries(GalileoSDK
   ${ROSCPP_LIBRARIES}
   ${JSONCPP_LIBRARIES}
)

install(
   TARGETS GalileoSDK
   EXPORT GalileoSDK
   RUNTIME DESTINATION bin
   LIBRARY DESTINATION lib
   ARCHIVE DESTINATION lib/static
)

install(
   DIRECTORY ${PROJECT_SOURCE_DIR}/include/ 
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.h"
)

install(
   DIRECTORY ${PROJECT_SOURCE_DIR}/../include/galileo_serial_server
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