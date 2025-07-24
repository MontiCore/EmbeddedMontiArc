@echo off
setlocal
if exist "%INTERACTIVE_SIMULATOR_HOME%\SVG" rmdir "%INTERACTIVE_SIMULATOR_HOME%\SVG" /s /q
mkdir "%INTERACTIVE_SIMULATOR_HOME%\SVG"
"%JAVA_HOME%\bin\java.exe" -jar "%SVG_HOME%\embeddedmontiarc-svggenerator.jar" ^
   --input "%CURRENT_ROOT_MODEL%" ^
   --modelPath "%HOME2%\models\%CURRENT_PROJECT%\\" ^
   --recursiveDrawing "true" ^
   --outputPath "%INTERACTIVE_SIMULATOR_HOME%\SVG\\"

"%JAVA_HOME%\bin\java.exe" -jar "%INTERACTIVE_SIMULATOR_HOME%\fixSvg.jar" ^
   -o "%INTERACTIVE_SIMULATOR_HOME%\SVG" ^
   "%INTERACTIVE_SIMULATOR_HOME%\SVG"
   
endlocal
   
