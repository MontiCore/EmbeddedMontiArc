@echo off

call "..\..\common\variables"

if exist "%SVG_HOME%\SVG" rmdir "%SVG_HOME%\SVG" /s /q
mkdir "%SVG_HOME%\SVG"

"%JAVA_HOME%\bin\java.exe" -jar "%VISUALIZATION_EMAM_HOME%\visualization-emam.jar" ^
   --model "de.rwth.supermario.superMarioWrapper" ^
   --modelPath "%HOME%\models\SuperMario\\" ^
   --outputPath "%SVG_HOME%\SVG\\"