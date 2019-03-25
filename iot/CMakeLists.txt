cmake_minimum_required(VERSION 3.1.0)
project(
   iot
   VERSION 1.0.0
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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include(GNUInstallDirs)

include_directories(
   ${PROJECT_SOURCE_DIR}
   ${PROJECT_SOURCE_DIR}/../IotWrapper
)

FILE(GLOB IOT_SOURCES ${PROJECT_SOURCE_DIR}/*.c)

add_library(iot
   ${IOT_SOURCES}
)

set_target_properties(iot PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(iot
   ${PROJECT_SOURCE_DIR}/../IotWrapper/build/libIotWrapper.a
)