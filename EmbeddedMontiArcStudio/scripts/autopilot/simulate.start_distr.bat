call ..\shared\variables.bat

call generate.bat
call compile.bat

pushd %~dp0
cd %RMI%
call mvn clean install
timeout 3 > NUL
start java -Djava.library.path=%RMI_PATH% -Djava.rmi.server.codebase=file:.\target\rmi-model-server-1.0.1.jar -Djava.rmi.server.hostname=127.0.0.1 -cp .\target\rmi-model-server-1.0.1.jar rwth.rmi.model.server.RMIServer
popd

start /b %PSQL%\PostgreSQLPortable.exe
timeout 3 > NUL
start /b %SFS%\sfs2x-standalone.exe
