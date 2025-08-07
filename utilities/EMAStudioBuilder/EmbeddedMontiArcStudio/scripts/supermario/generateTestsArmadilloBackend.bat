@rem (c) https://github.com/MontiCore/monticore  
call "..\shared\variables"
if exist "%STREAM_TEST_EXEC_DIR%" rmdir "%STREAM_TEST_EXEC_DIR%" /s /q
mkdir "%STREAM_TEST_EXEC_DIR%"
"%JAVA_HOME%\bin\java.exe" -jar "%STREAM_TESTING%\emam2cpp.jar" ^
   --models-dir="%HOME%\model\supermario" ^
   --output-dir="%TESTS_CPP_DIR%" ^
   --root-model=%1 ^
   --flag-generate-tests ^
   --flag-use-armadillo-backend
xcopy /s /y %HOME%\precompiled %TESTS_CPP_DIR%
xcopy /s /y %HOME%\precompiled\test %TESTS_CPP_DIR%\test
