@ECHO OFF
call "..\..\common\variables"
call generate.bat
call compile.bat
call start-simulation.bat