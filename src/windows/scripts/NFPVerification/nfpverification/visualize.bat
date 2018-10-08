set file=%1 
set file=%file:~0,-5%
echo %file%
REM PAUSE
"%JAVA_HOME%\bin\java.exe" -jar "%SVG_HOME%\embeddedmontiarc-svggenerator.jar" ^
   --input "%file%" ^
   --modelPath "%HOME%\models\NFPVerification\target\witnesses_%2_example.model.Sensors" ^
   --recursiveDrawing "false" ^
   --outputPath "%NFPVERIFICATION_HOME%\results\\" 