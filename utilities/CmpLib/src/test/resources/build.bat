@rem (c) https://github.com/MontiCore/monticore  
@echo off

set HOME=%CD%

set ARMADILLO_HOME=%HOME%\etc\armadillo
set LIB_DIR=%HOME%\etc\lib
set MINGW_HOME=%HOME%\etc\mingw64

set PATH=%LIB_DIR%;%MINGW_HOME%\bin;ARMADILLO_HOME;%PATH%

g++ -O0 ^
    -I"%ARMADILLO_HOME%\include" ^
    -L"%LIB_DIR%" ^
    -o "%1\test\test_exec.exe" ^
    "%1\test\tests_main.cpp" ^
    -DCATCH_CONFIG_MAIN=1 ^
    -DARMA_DONT_USE_WRAPPER -lopenblas

%1\test\test_exec.exe > %1\test\out.txt
