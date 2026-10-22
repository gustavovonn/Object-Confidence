cmake_minimum_required(VERSION 3.16)
project(ObjectConfidence LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Static executables for the VisionFive 2 initramfs (vf2-setup/vf2_setup.sh)
option(STATIC_BUILD "Link executables statically" OFF)
if(STATIC_BUILD)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")
endif()

# Hungarian algorithm (BSD), libs/hungarian-algorithm-cpp
add_library(hungarian STATIC libs/hungarian-algorithm-cpp/Hungarian.cpp)
target_include_directories(hungarian SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/libs/hungarian-algorithm-cpp)

add_library(ukf STATIC libs/ukf/ukf.cpp)
target_include_directories(ukf SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/libs/ukf ${PROJECT_SOURCE_DIR}/libs/eigen)

add_library(object_confidence STATIC
    src/ObjectConfidenceEngine.cc
    src/Tracker.cc
)
target_include_directories(object_confidence PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/libs/v2x-misc/include
)
target_include_directories(object_confidence SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/libs/eigen)
target_compile_options(object_confidence PRIVATE -Wall -Wextra)
target_link_libraries(object_confidence PUBLIC ukf PRIVATE hungarian)

add_executable(replay tools/replay.cc)
target_link_libraries(replay PRIVATE object_confidence)
