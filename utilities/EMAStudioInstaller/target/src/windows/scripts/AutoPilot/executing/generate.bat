if exist "%CPP_DIR%" rmdir "%CPP_DIR%" /s /q
mkdir "%CPP_DIR%"
"%JAVA_HOME%\bin\java.exe" -jar "%HOME%\emam2cpp.jar" ^
   --models-dir="%HOME%\models\autopilot" ^
   --root-model=de.rwth.armin.modeling.autopilot.autopilot ^
   --output-dir="%CPP_DIR%" ^
   --flag-generate-autopilot-adapter ^
   --flag-use-armadillo-backend
xcopy %HOME%\precompiled %CPP_DIR%