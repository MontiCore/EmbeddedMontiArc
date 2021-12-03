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
set(LIBS ${LIBS} mxnet)

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
