@ECHO OFF
call "..\..\common\variables"

if exist "%SFS%\extensions\AllInOne" del /f /q "%SFS%\extensions\AllInOne"

copy "%DISTR_SIM%\SmartFoxExtensions\distributed" "%SFS%\extensions\AllInOne"

call generate.bat
call compile.bat

pushd %~dp0
cd %RMI%
start /b java -Djava.library.path=%RMI_PATH% -Djava.rmi.server.codebase=file:.\rmi-model-server-1.0.1.jar -Djava.rmi.server.hostname=127.0.0.1 -cp .\rmi-model-server-1.0.1.jar rwth.rmi.model.server.RMIServer
popd

start /b %PSQL%\PostgreSQLPortable.exe
ping -n 10 127.0.0.1>nul
start /b %SFS%\sfs2x-standalone.exe
