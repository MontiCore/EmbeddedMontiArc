@rem (c) https://github.com/MontiCore/monticore  
call "..\shared\variables.bat"
if exist "%SVG_HOME%\SVG" rmdir "%SVG_HOME%\SVG" /s /q
mkdir "%SVG_HOME%\SVG"
"%JAVA_HOME%\bin\java.exe" -jar "%SVG_HOME%\embeddedmontiarc-svggenerator.jar" ^
   --input "de.rwth.armin.modeling.autopilot.motion.calculatePidError" ^
   --modelPath "%HOME%\model\autopilot\\" ^
   --recursiveDrawing "false" ^
   --outputPath "%SVG_HOME%\SVG\\" ^
   --resource 
