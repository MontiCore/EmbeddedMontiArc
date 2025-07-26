@rem (c) https://github.com/MontiCore/monticore  
g++ -fPIC ^
   -I"%JAVA_HOME%\include" ^
   -I"%JAVA_HOME%\include\win32" ^
   -I"%ARMADILLO_HOME%\include" ^
   -L"%HOME%\lib" ^
   -o "%TEST_EXEC_DIR%\TestsForCurrentModel.exe" ^
   "%TESTS_CPP_DIR%\test\catch_tests_main.o"  ^
   "%TESTS_CPP_DIR%\HelperA.o"  ^
   "%TESTS_CPP_DIR%\test\tests_main.cpp" ^
   -DARMA_DONT_USE_WRAPPER -lblas -llapack -lgdi32 -luser32 -lshell32
