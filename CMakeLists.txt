cmake_minimum_required(VERSION 3.0)

enable_language(CXX)

set(SERIALLOOPINCLUDE target/generated-sources-cpp/serialLoop)
set(SIMPLELOOPINCLUDE target/generated-sources-cpp/simpleLoop)
set(OSCILLATIONINCLUDE target/generated-sources-cpp/oscillation)
set(PARALLELLOOPINCLUDE target/generated-sources-cpp/parallelLoop)
add_subdirectory(${SERIALLOOPINCLUDE})
add_subdirectory(${SIMPLELOOPINCLUDE})
add_subdirectory(${OSCILLATIONINCLUDE})
add_subdirectory(${PARALLELLOOPINCLUDE})
add_subdirectory(looptest)