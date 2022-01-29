cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(vae_connector LANGUAGES CXX)

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
add_library(vae_connector vae_connector.cpp)
target_include_directories(vae_connector PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(vae_connector PUBLIC ${LIBS})
set_target_properties(vae_connector PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS vae_connector FILE vae_connector.cmake)

# additional commands end
include_directories(test)
add_executable(vae_connector_StreamTests  test/tests_main.cpp)
target_compile_definitions(vae_connector_StreamTests PRIVATE CATCH_CONFIG_MAIN=1 ARMA_DONT_USE_WRAPPER)
target_link_libraries(vae_connector_StreamTests  PUBLIC vae_connector)
set_target_properties(vae_connector_StreamTests  PROPERTIES LINKER_LANGUAGE CXX)

# execute tests
add_custom_target(run_vae_connector_StreamTests ALL
                  COMMAND vae_connector_StreamTests
                  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
