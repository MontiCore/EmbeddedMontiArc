cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(torcs_agent_dqn_reward LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# add dependencies
find_package(Armadillo REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Armadillo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Armadillo_LIBRARIES})

# additional commands

# create static library
include_directories(${INCLUDE_DIRS})
add_library(torcs_agent_dqn_reward torcs_agent_dqn_reward.cpp)
target_include_directories(torcs_agent_dqn_reward PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(torcs_agent_dqn_reward PUBLIC ${LIBS})
set_target_properties(torcs_agent_dqn_reward PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS torcs_agent_dqn_reward FILE torcs_agent_dqn_reward.cmake)

# additional commands end
