@ECHO OFF
call "..\..\common\variables"
call stop-simulation.bat

FOR /F "tokens=5 delims= " %%P IN ('netstat -a -n -o ^| findstr 0.0.0.0:8080') DO TaskKill.exe /F /PID %%P