call "..\shared\variables.bat"
"%JAVA_HOME%\bin\java.exe" -jar "%SVG_HOME%\emam2ema.jar" ^
   %HOME%\model ^
   %SVG_HOME%\modelEMA
if exist "%SVG_HOME%\SVG" rmdir "%SVG_HOME%\SVG" /s /q
mkdir "%SVG_HOME%\SVG"
"%JAVA_HOME%\bin\java.exe" -jar "%SVG_HOME%\embeddedmontiarc-svggenerator.jar" ^
   --input "pumpStationExample.pumpStation" ^
   --modelPath "%SVG_HOME%\modelEMA\pump\\" ^
   --recursiveDrawing "true" ^
   --outputPath "%SVG_HOME%\SVG\\"