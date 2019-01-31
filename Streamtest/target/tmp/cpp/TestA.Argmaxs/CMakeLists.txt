cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)

project(TestA_argmaxs LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# add dependencies
find_package(Armadillo REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Armadillo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Armadillo_LIBRARIES})

# additional commands

# create static library
include_directories(${INCLUDE_DIRS})
add_library(TestA_argmaxs TestA_argmaxs.h)
target_include_directories(TestA_argmaxs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(TestA_argmaxs PUBLIC ${LIBS})
set_target_properties(TestA_argmaxs PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS TestA_argmaxs FILE TestA_argmaxs.cmake)

# additional commands end
include_directories(test)
add_executable(StreamTests test/tests_main.cpp)
target_compile_definitions(StreamTests PRIVATE CATCH_CONFIG_MAIN=1 ARMA_DONT_USE_WRAPPER)
