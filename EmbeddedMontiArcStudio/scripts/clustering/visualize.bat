call "..\shared\variables.bat"
if exist "%SVG_HOME%\SVG" rmdir "%SVG_HOME%\SVG" /s /q
mkdir "%SVG_HOME%\SVG"
"%JAVA_HOME%\bin\java.exe" -jar "%VISUALIZATION_EMAM_HOME%\visualization-emam.jar" ^
   --model "detection.spectralClusterer" ^
   --modelPath "%HOME%\model\clustering\\" ^
   --outputPath "%SVG_HOME%\SVG\\"