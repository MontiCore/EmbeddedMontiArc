@rem (c) https://github.com/MontiCore/monticore  
if exist "%CPP_DIR%" rmdir "%CPP_DIR%" /s /q
mkdir "%CPP_DIR%"
"%JAVA_HOME%\bin\java.exe" -jar "%HOME%\emam2cpp.jar" ^
   --models-dir="%HOME%\model" ^
   --root-model=%1 ^
   --output-dir="%CPP_DIR%" ^
   --flag-generate-autopilot-adapter ^
   --flag-use-armadillo-backend
xcopy %HOME%\precompiled %CPP_DIR%
