call "..\shared\variables.bat"
if exist "%SVG_HOME%\SVG" rmdir "%SVG_HOME%\SVG" /s /q
mkdir "%SVG_HOME%\SVG"
"%JAVA_HOME%\bin\java.exe" -jar "%SVG_HOME%\embeddedmontiarc-svggenerator.jar" ^
   --model "de.rwth.supermario.superMarioWrapper" ^
   --modelPath "%HOME%\model\supermario\\" ^
   --outputPath "%SVG_HOME%\SVG\\"