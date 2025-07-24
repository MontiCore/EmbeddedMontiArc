cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(load_test LANGUAGES CXX)


add_library(load_test_lib SHARED src/lib.cpp)
target_link_libraries(load_test_lib PUBLIC -static-libgcc -static-libstdc++)
set_target_properties(load_test_lib PROPERTIES LINKER_LANGUAGE CXX POSITION_INDEPENDENT_CODE ON)
set_target_properties(load_test_lib PROPERTIES PREFIX "")
target_compile_features(load_test_lib PUBLIC cxx_std_14)



add_executable(exe_test src/main.cpp)
target_link_libraries(exe_test PUBLIC dl)