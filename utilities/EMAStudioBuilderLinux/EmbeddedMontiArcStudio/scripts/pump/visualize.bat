@rem (c) https://github.com/MontiCore/monticore  
call "..\shared\variables.bat"
if exist "%SVG_HOME%\SVG" rmdir "%SVG_HOME%\SVG" /s /q
mkdir "%SVG_HOME%\SVG"
"%JAVA_HOME%\bin\java.exe" -jar "%SVG_HOME%\embeddedmontiarc-svggenerator.jar" ^
   --input "pumpStationExample.pumpStation" ^
   --modelPath "%HOME%\model\pump\model\\" ^
   --recursiveDrawing "true" ^
   --outputPath "%SVG_HOME%\SVG\\"
