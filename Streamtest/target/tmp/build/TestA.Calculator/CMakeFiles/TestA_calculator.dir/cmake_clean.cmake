file(REMOVE_RECURSE
  "libTestA_calculator.pdb"
  "libTestA_calculator.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/TestA_calculator.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
