@ECHO OFF
FOR /F "tokens=5 delims= " %%P IN ('netstat -a -n -o ^| findstr 0.0.0.0:80') DO TaskKill.exe /F /PID %%P
FOR /F "tokens=5 delims= " %%P IN ('netstat -a -n -o ^| findstr [::1]:5432') DO TaskKill.exe /F /PID %%P
FOR /F "tokens=5 delims= " %%P IN ('netstat -a -n -o ^| findstr 0.0.0.0:10101') DO TaskKill.exe /F /PID %%P

TaskKill.exe /IM PostgreSQLPortable.exe /F
TaskKill.exe /IM psql.exe /F