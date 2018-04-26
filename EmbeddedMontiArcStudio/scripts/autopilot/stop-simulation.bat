pushd %~dp0
cd "%TOMCAT_HOME%\bin"
call shutdown.bat
popd
