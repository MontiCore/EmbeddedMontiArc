REM @echo off
if exist "%WASM_HOME%\%CURRENT_PROJECT%\cpp" rmdir "%WASM_HOME%\%CURRENT_PROJECT%\cpp" /s /q
mkdir "%WASM_HOME%\%CURRENT_PROJECT%\cpp"
"%JAVA_HOME%\bin\java.exe" -jar "%WASM_HOME%\emam2cppLogging.jar" ^
   --models-dir="%HOME2%\models\%CURRENT_PROJECT%" ^
   --root-model=%CURRENT_ROOT_MODEL% ^
   --output-dir="%WASM_HOME%\%CURRENT_PROJECT%\cpp" ^
   --flag-use-armadillo-backend