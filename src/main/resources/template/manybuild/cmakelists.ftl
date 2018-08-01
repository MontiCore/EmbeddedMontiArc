cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)

project(combined_test LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${'$'}{CMAKE_MODULE_PATH} ${'$'}{CMAKE_CURRENT_SOURCE_DIR}/cmake)

# add dependencies
find_package(Armadillo REQUIRED)
set(INCLUDE_DIRS ${'$'}{INCLUDE_DIRS} ${'$'}{Armadillo_INCLUDE_DIRS})
set(LIBS ${'$'}{LIBS} ${'$'}{Armadillo_LIBRARIES})

# additional commands

# create static library
include_directories(${'$'}{INCLUDE_DIRS})

# additional commands end
include_directories(test)
add_executable(StreamTests combined_tests_main.cpp)
target_compile_definitions(StreamTests PRIVATE CATCH_CONFIG_MAIN=1 ARMA_DONT_USE_WRAPPER)
