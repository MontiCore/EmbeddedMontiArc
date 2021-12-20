cmake_minimum_required(VERSION 3.7)
project(pe-parser-library)

# List all files explicitly; this will make IDEs happy (i.e. QtCreator, CLion, ...)
list(APPEND PEPARSERLIB_SOURCEFILES
  include/parser-library/parse.h
  include/parser-library/nt-headers.h
  include/parser-library/to_string.h

  src/buffer.cpp
  src/parse.cpp
)

if ( CMAKE_COMPILER_IS_GNUCC )
  add_definitions(
    -static-libstdc++
  )
endif()
add_library(pe-parser-library ${PEPARSERLIB_SOURCEFILES})
target_include_directories(pe-parser-library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(pe-parser-library PRIVATE ${GLOBAL_CXXFLAGS})

if ( CMAKE_COMPILER_IS_GNUCC )
  target_compile_options(pe-parser-library PRIVATE
      -fPIC
  )
endif()

# install(TARGETS pe-parser-library
#   RUNTIME DESTINATION "bin"
#   LIBRARY DESTINATION "lib"
#   ARCHIVE DESTINATION "lib"
# )
# install(FILES "cmake/peparse-config.cmake" DESTINATION "lib/cmake/peparse")
# install(DIRECTORY "include/parser-library" DESTINATION "include")