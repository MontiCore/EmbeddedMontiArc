@rem (c) https://github.com/MontiCore/monticore  
cd ..\shared
call variables
cd ..\pacman
if exist "%STREAM_TESTING%\testResults\" rmdir "%STREAM_TESTING%\testResults\" /s /q
if exist "%HOME%\testResults\" rmdir "%HOME%\testResults\" /s /q
mkdir %STREAM_TESTING%\testResults\
mkdir %HOME%\testResults\
call generateTestsArmadilloBackend %1
call compileTestsArmadilloBackendOpenBLAS
REM start /min runCurrentExecutable %1
call runCurrentExecutable %1
xcopy /s /y %STREAM_TESTING%\testResults %HOME%\testResults
REM :start
REM  tasklist /FI "windowtitle eq runCurrentExecutable.bat" | findstr "cmd.exe" >nul
REM if %ERRORLEVEL% == 0 goto loop
REM goto start
REM :loop
REM sleep 1

REM taskkill /fi "status eq not responding" /f
REM if %ERRORLEVEL% == 1 goto killit
REM goto loop2
REM :loop2
REM tasklist /FI "windowtitle eq runCurrentExecutable.bat" | findstr "cmd.exe" >nul
REM if %ERRORLEVEL% == 1 goto continue


REM goto loop
REM :killit

REM taskkill /fi "windowtitle eq runCurrentExecutable.bat" /f /t
REM goto loop2
REM :continue


