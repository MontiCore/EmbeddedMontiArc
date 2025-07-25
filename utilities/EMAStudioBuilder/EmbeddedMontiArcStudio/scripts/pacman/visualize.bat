@rem (c) https://github.com/MontiCore/monticore  
call "..\shared\variables.bat"
if exist "%SVG_HOME%\SVG" rmdir "%SVG_HOME%\SVG" /s /q
mkdir "%SVG_HOME%\SVG"
"%JAVA_HOME%\bin\java.exe" -jar "%VISUALIZATION_EMAM_HOME%\visualization-emam.jar" ^
   --model "de.rwth.pacman.pacManWrapper" ^
   --modelPath "%HOME%\model\pacman\\" ^
   --outputPath "%SVG_HOME%\SVG\\"
