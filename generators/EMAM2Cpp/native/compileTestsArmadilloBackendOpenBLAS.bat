@rem (c) https://github.com/MontiCore/monticore  


call variables.bat
set STREAM_TESTING=%PROJECT_ROOT%\target\generated-sources-cpp\streamtest
set AUTOPILOT_TESTS=%STREAM_TESTING%\target\generated-sources-cpp\streamtest\autopilot
set STREAM_TEST_EXEC_DIR=%STREAM_TESTING%\exec
g++ --version

g++ -std=c++11 ^
   -I"%JAVA_HOME%\include_win" ^
   -I"%JAVA_HOME%\include_win\win32" ^
   -I"%ARMADILLO_HOME%\include" ^
   -L"%HOME%\lib\win" ^
   -o "%2\TestsForCurrentModel.exe" ^
   "%1/test/tests_main.cpp" ^
   -include %ARMADILLO_HOME%\include\armadillo ^
   -DARMA_DONT_USE_WRAPPER -lgdi32 -lopenblas -llibarpack-2
