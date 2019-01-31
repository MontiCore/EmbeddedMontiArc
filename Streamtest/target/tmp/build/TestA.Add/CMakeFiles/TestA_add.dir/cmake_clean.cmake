file(REMOVE_RECURSE
  "libTestA_add.pdb"
  "libTestA_add.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/TestA_add.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
