cmake_minimum_required(VERSION 3.1.0)
project(
   GalileoSDKTest
   VERSION 1.0.0
   LANGUAGES CXX
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -std=c++11 -fpic")


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(PkgConfig REQUIRED)
pkg_search_module(ROSCPP REQUIRED roscpp)
find_package(Boost REQUIRED COMPONENTS thread filesystem regex system)

include_directories(
   ${PROJECT_SOURCE_DIR}/include
   ${ROSCPP_INCLUDE_DIRS}
   ${PROJECT_SOURCE_DIR}/../include
   ${PROJECT_SOURCE_DIR}/../GalileoSDK/include
   ${PROJECT_SOURCE_DIR}/../iot
   ${PROJECT_SOURCE_DIR}/../IotWrapper
)

add_executable(GalileoTest
    ${PROJECT_SOURCE_DIR}/GalileoSDKTest.cpp
)

target_link_libraries(GalileoTest
    ${ROSCPP_LINK_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/../GalileoSDK/build/libGalileoSDK.so
    ${Boost_LIBRARIES}
)

