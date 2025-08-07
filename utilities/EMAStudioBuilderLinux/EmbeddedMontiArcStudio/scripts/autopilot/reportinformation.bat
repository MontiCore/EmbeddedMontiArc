@rem (c) https://github.com/MontiCore/monticore  
call "..\shared\variables"
"%JAVA_HOME%\bin\java.exe" -jar "%HOME%\emam2cpp.jar" ^
   --models-dir="%HOME%\model\autopilot" ^
   --output-dir="%TESTS_CPP_DIR%" ^
   --root-model=a ^
   --check-model-dir
if exist "%HOME%\testInfo" rmdir "%HOME%\testInfo" /s /q
mkdir %HOME%\testInfo
xcopy /s %TESTS_CPP_DIR%\reporting %HOME%\testInfo
