call variables.bat
set STREAM_TESTING=%PROJECT_ROOT%\target\generated-sources-cpp\streamtest
set AUTOPILOT_TESTS=%STREAM_TESTING%\target\generated-sources-cpp\streamtest\autopilot
set STREAM_TEST_EXEC_DIR=%STREAM_TESTING%\exec

g++ -std=c++11 ^
   -I"%JAVA_HOME%\include" ^
   -I"%JAVA_HOME%\include\win32" ^
   -I"%ARMADILLO_HOME%\include" ^
   -L"%HOME%\lib" ^
   -o "%2\TestsForCurrentModel.exe" ^
   "%ARMADILLO_HOME%\include\catch_tests_main.o"  ^
   "%1/test/tests_main.cpp" ^
   -include %ARMADILLO_HOME%\include\armadillo.h ^
   -DARMA_DONT_USE_WRAPPER -lgdi32 -lopenblas -llibarpack-2
