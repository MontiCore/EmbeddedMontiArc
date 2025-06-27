cmake_minimum_required (VERSION 2.8)

set (CMAKE_CXX_FLAGS "-g -std=c++0x")

find_package (vsomeip 2.10.0 REQUIRED)
find_package( Boost 1.55 COMPONENTS system thread log REQUIRED )

include_directories (
    ${Boost_INCLUDE_DIR}
    ${VSOMEIP_INCLUDE_DIRS}
)

add_executable(publisher ../src/publisher.cpp)
target_link_libraries(publisher vsomeip ${Boost_LIBRARIES})
add_executable(subscriber ../src/subscriber.cpp)
target_link_libraries(subscriber vsomeip ${Boost_LIBRARIES})

