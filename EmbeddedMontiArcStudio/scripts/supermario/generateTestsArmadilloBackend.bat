call "..\shared\variables"
if exist "%TEST_EXEC_DIR%" rmdir "%TEST_EXEC_DIR%" /s /q
mkdir "%TEST_EXEC_DIR%"
if exist "%TESTS_CPP_DIR%" rmdir "%TESTS_CPP_DIR%" /s /q
mkdir "%TESTS_CPP_DIR%"
"%JAVA_HOME%\bin\java.exe" -jar "%STREAM_TESTING%\emam2cpp.jar" ^
   --models-dir="%HOME%\model\supermario" ^
   --output-dir="%TESTS_CPP_DIR%" ^
   --root-model=%1 ^
   --flag-generate-tests ^
   --flag-use-armadillo-backend
xcopy %HOME%\precompiled %TESTS_CPP_DIR%
xcopy %HOME%\precompiled\test %TESTS_CPP_DIR%\test