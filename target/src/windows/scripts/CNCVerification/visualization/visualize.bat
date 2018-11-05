@echo off

call "..\..\common\variables"

if exist "%SVG_HOME%\SVG" rmdir "%SVG_HOME%\SVG" /s /q
mkdir "%SVG_HOME%\SVG"

"%JAVA_HOME%\bin\java.exe" -jar "%VISUALIZATION_EMAM_HOME%\visualization-emam.jar" ^
   --model "fas.demo_fas_Fkt_m.fAS" ^
   --modelPath "%HOME%\models\CNCVerification\\" ^
   --outputPath "%SVG_HOME%\SVG\\"