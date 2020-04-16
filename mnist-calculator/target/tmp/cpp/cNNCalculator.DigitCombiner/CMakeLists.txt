cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(cNNCalculator_digitCombiner LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# add dependencies
find_package(Armadillo REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Armadillo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Armadillo_LIBRARIES})


# additional commands

# create static library
include_directories(${INCLUDE_DIRS})
add_library(cNNCalculator_digitCombiner cNNCalculator_digitCombiner.cpp)
target_include_directories(cNNCalculator_digitCombiner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(cNNCalculator_digitCombiner PUBLIC ${LIBS})
set_target_properties(cNNCalculator_digitCombiner PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS cNNCalculator_digitCombiner FILE cNNCalculator_digitCombiner.cmake)

# additional commands end
include_directories(test)
add_executable(cNNCalculator_digitCombiner_StreamTests  test/tests_main.cpp)
target_compile_definitions(cNNCalculator_digitCombiner_StreamTests PRIVATE CATCH_CONFIG_MAIN=1 ARMA_DONT_USE_WRAPPER)
target_link_libraries(cNNCalculator_digitCombiner_StreamTests  PUBLIC cNNCalculator_digitCombiner)
set_target_properties(cNNCalculator_digitCombiner_StreamTests  PROPERTIES LINKER_LANGUAGE CXX)

# execute tests
add_custom_target(run_cNNCalculator_digitCombiner_StreamTests ALL
                  COMMAND cNNCalculator_digitCombiner_StreamTests
                  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
