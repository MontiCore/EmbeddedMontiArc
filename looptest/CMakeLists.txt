cmake_minimum_required(VERSION 3.5)
project(LoopTest CXX)
set(CMAKE_CXX_STANDARD 11)

add_executable(Oscillation mainOscillation.cpp)
target_include_directories(Oscillation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Oscillation de_monticore_lang_monticar_semantics_loops_oscillation ${LIBS})
export(TARGETS Oscillation FILE Oscillation.cmake)

add_executable(SerialLoop mainSerialLoop.cpp)
target_include_directories(SerialLoop PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(SerialLoop de_monticore_lang_monticar_semantics_loops_serialLoop ${LIBS})
export(TARGETS SerialLoop FILE SerialLoop.cmake)

add_executable(SimpleLoop mainSimpleLoop.cpp)
target_include_directories(SimpleLoop PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(SimpleLoop de_monticore_lang_monticar_semantics_loops_simpleLoop ${LIBS})
export(TARGETS SimpleLoop FILE SimpleLoop.cmake)

add_executable(ParallelLoop mainParallelLoop.cpp)
target_include_directories(ParallelLoop PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ParallelLoop de_monticore_lang_monticar_semantics_loops_parallelLoop ${LIBS})
export(TARGETS ParallelLoop FILE ParallelLoop.cmake)
