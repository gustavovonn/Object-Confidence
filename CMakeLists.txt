cmake_minimum_required(VERSION 3.16)
project(ObjectConfidence LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

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

enable_testing()
add_executable(test_engine tests/test_engine.cc)
target_link_libraries(test_engine PRIVATE object_confidence)
add_test(NAME engine COMMAND test_engine)

add_executable(replay tools/replay.cc)
target_link_libraries(replay PRIVATE object_confidence)

add_executable(test_adapter tests/test_adapter.cc)
target_link_libraries(test_adapter PRIVATE object_confidence)
add_test(NAME adapter COMMAND test_adapter)
