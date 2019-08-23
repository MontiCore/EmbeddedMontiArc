@rem (c) https://github.com/MontiCore/monticore  
call "..\shared\variables"
if exist "%CPP_DIR%" rmdir "%CPP_DIR%" /s /q
mkdir "%CPP_DIR%"
"%JAVA_HOME%\bin\java.exe" -jar "%HOME%\emam2cpp.jar" ^
   --models-dir="%HOME%\model\clustering" ^
   --root-model=detection.objectDetector1 ^
   --output-dir="%CPP_DIR%" ^
   --flag-use-armadillo-backend
