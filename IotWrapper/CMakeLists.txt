cmake_minimum_required(VERSION 3.1.0)
project(
   IotWrapper
   VERSION 1.0.0
   LANGUAGES CXX
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# build static link lib by default
if("${BUILD_SHARED_LIBS}" STREQUAL ON)
   set(BUILD_SHARED_LIBS ON)
else()
   set(BUILD_SHARED_LIBS OFF)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include(GNUInstallDirs)


include_directories(
   ${PROJECT_SOURCE_DIR}
)

add_library(IotWrapper
   ${PROJECT_SOURCE_DIR}/iot_wrapper.cpp
)