g++ -std=c++11 ^
   -I"%JAVA_HOME%\include" ^
   -I"%JAVA_HOME%\include\win32" ^
   -I"%ARMADILLO_HOME%\include" ^
   -L"%HOME%\lib" ^
   -o "%TEST_EXEC_DIR%\TestsForCurrentModel.exe" ^
   "%TESTS_CPP_DIR%\test\catch_tests_main.o"  ^
   "%TESTS_CPP_DIR%\HelperA.o"  ^
   "%TESTS_CPP_DIR%\test\tests_main.cpp" ^
   -include %ARMADILLO_HOME%\include\armadillo.h ^
   -DARMA_DONT_USE_WRAPPER -lgdi32 -lopenblas -llibarpack-2
