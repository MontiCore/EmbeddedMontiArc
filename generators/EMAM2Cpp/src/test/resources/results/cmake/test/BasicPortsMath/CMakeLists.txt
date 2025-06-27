cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(test_basicPortsMath LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# add dependencies
find_package(Armadillo REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Armadillo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Armadillo_LIBRARIES})

# additional library linkage

# additional commands

# create static library
include_directories(${INCLUDE_DIRS})
add_library(test_basicPortsMath test_basicPortsMath.cpp)
target_include_directories(test_basicPortsMath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(test_basicPortsMath PUBLIC ${LIBS})
set_target_properties(test_basicPortsMath PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS test_basicPortsMath FILE test_basicPortsMath.cmake)

# additional commands end
include_directories(test)
add_executable(test_basicPortsMath_StreamTests  test/tests_main.cpp)
target_compile_definitions(test_basicPortsMath_StreamTests PRIVATE CATCH_CONFIG_MAIN=1 ARMA_DONT_USE_WRAPPER)
target_link_libraries(test_basicPortsMath_StreamTests  PUBLIC test_basicPortsMath)
set_target_properties(test_basicPortsMath_StreamTests  PROPERTIES LINKER_LANGUAGE CXX)

# execute tests
add_custom_target(run_test_basicPortsMath_StreamTests ALL
                  COMMAND test_basicPortsMath_StreamTests
                  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
