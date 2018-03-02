cmake_minimum_required(VERSION 3.5)
project (Coordinator_${compName} CXX)

set (CMAKE_CXX_STANDARD 11)
set (THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(Coordinator_${compName} Coordinator_${compName}.cpp)
set_target_properties(Coordinator_${compName} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(Coordinator_${compName} ${targets} Threads::Threads)
target_include_directories(Coordinator_${compName} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

export(TARGETS Coordinator_${compName} FILE Coordinator_${compName}.cmake)