REM @echo off
popd
pushd %~dp0
cd "..\..\common"
call setup
popd
if exist "%WASM_HOME%\%CURRENT_PROJECT%\wasm" rmdir "%WASM_HOME%\%CURRENT_PROJECT%\wasm" /s /q
mkdir "%WASM_HOME%\%CURRENT_PROJECT%\wasm"
call %EMSCRIPTEN_HOME%\emcc.bat ^
     %WASM_HOME%\%CURRENT_PROJECT%\cpp\%Current_MAIN_COMPONENT%.cpp ^
     -o %WASM_HOME%\%CURRENT_PROJECT%\wasm\%Current_MAIN_COMPONENT%.js ^
     -I"%WASM_HOME%\armadillo-code-8.400.x\include" ^
     -L"%WASM_HOME%\lib" ^
     -s WASM=1 -s FORCE_FILESYSTEM=1 -s ALLOW_MEMORY_GROWTH=1 -std=c++11 -DARMA_DONT_USE_WRAPPER -llapack_WIN64 -llibblas_WIN64 -llibf2c -ltmglib_WIN64 -llibfblaswr -O3 --bind