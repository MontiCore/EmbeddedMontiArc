cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(de_rwth_montisim_agent_master LANGUAGES CXX)

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
add_library(de_rwth_montisim_agent_master de_rwth_montisim_agent_master.cpp)
target_include_directories(de_rwth_montisim_agent_master PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(de_rwth_montisim_agent_master PUBLIC ${LIBS})
set_target_properties(de_rwth_montisim_agent_master PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS de_rwth_montisim_agent_master FILE de_rwth_montisim_agent_master.cmake)

# additional commands end
