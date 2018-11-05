call "..\..\common\variables"

if exist "%CLUSTERER_EXEC_DIR%" rmdir "%CLUSTERER_EXEC_DIR%" /s /q
mkdir "%CLUSTERER_EXEC_DIR%"

copy %HOME%\lib\mainClusterer.cpp %CPP_DIR%

g++ -fPIC ^
   -I"%JAVA_HOME%\include" ^
   -I"%JAVA_HOME%\include\win32" ^
   -I"%ARMADILLO_HOME%\include" ^
   -L"%HOME%\lib" ^
   -o "%CLUSTERER_EXEC_DIR%\Clusterer.exe" ^
   "%CPP_DIR%\mainClusterer.cpp" ^
   -DARMA_DONT_USE_WRAPPER -lgdi32 -lopenblas -llibarpack-2