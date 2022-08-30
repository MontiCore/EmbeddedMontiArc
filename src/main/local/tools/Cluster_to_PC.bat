@echo off
set TRIES=25
set INTERVAL=5

:retry

winscp.com /ini=nul /log=transferC2P.log /script=Cluster_to_PC.txt /parameter // %1 %2 %3 %4 %5

if %ERRORLEVEL% neq 0 (
   set /A TRIES=%TRIES%-1
   if %TRIES% gtr 1 (
       echo Cluster has not finished, retrying in %INTERVAL% seconds...
       timeout /t %INTERVAL%
       goto retry
   ) else (
       echo Failed, aborting
       exit /b 1
   )
)

echo Received new file!
exit /b 0