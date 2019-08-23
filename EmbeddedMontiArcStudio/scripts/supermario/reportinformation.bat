@rem (c) https://github.com/MontiCore/monticore  
call "..\shared\variables"
"%JAVA_HOME%\bin\java.exe" -jar "%STREAM_TESTING%\emam2cpp.jar" ^
   --models-dir="%HOME%\model\supermario" ^
   --output-dir="%TESTS_CPP_DIR%" ^
   --root-model=de.rwth.supermario.superMarioWrapper ^
   --check-model-dir
if exist "%STREAM_TESTING%\testInfo" rmdir "%STREAM_TESTING%\testInfo" /s /q
mkdir %STREAM_TESTING%\testInfo
xcopy /s /y %TESTS_CPP_DIR%\reporting %STREAM_TESTING%\testInfo
