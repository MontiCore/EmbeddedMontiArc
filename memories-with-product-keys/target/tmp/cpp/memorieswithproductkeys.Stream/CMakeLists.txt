cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(memorieswithproductkeys_stream LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# add dependencies
find_package(Armadillo REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Armadillo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Armadillo_LIBRARIES})


# additional commands

# create static library
include_directories(${INCLUDE_DIRS})
add_library(memorieswithproductkeys_stream memorieswithproductkeys_stream.cpp)
target_include_directories(memorieswithproductkeys_stream PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(memorieswithproductkeys_stream PUBLIC ${LIBS})
set_target_properties(memorieswithproductkeys_stream PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS memorieswithproductkeys_stream FILE memorieswithproductkeys_stream.cmake)

# additional commands end
include_directories(test)
add_executable(memorieswithproductkeys_stream_StreamTests  test/tests_main.cpp)
target_compile_definitions(memorieswithproductkeys_stream_StreamTests PRIVATE CATCH_CONFIG_MAIN=1 ARMA_DONT_USE_WRAPPER)
target_link_libraries(memorieswithproductkeys_stream_StreamTests  PUBLIC memorieswithproductkeys_stream)
set_target_properties(memorieswithproductkeys_stream_StreamTests  PROPERTIES LINKER_LANGUAGE CXX)

# execute tests
add_custom_target(run_memorieswithproductkeys_stream_StreamTests ALL
                  COMMAND memorieswithproductkeys_stream_StreamTests
                  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
