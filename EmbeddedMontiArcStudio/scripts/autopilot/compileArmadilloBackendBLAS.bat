@rem (c) https://github.com/MontiCore/monticore  
if exist "%DLL_DIR%" rmdir "%DLL_DIR%" /s /q
mkdir "%DLL_DIR%"
g++ -shared -fPIC ^
   -I"%JAVA_HOME%\include" ^
   -I"%JAVA_HOME%\include\win32" ^
   -I"%ARMADILLO_HOME%\include" ^
   -L"%HOME%\lib" ^
   -o "%DLL_DIR%\AutopilotAdapter.dll" ^
   "%CPP_DIR%\AutopilotAdapter.cpp" ^
   -DARMA_DONT_USE_WRAPPER -lblas -llapack -lgdi32 -luser32 -lshell32
