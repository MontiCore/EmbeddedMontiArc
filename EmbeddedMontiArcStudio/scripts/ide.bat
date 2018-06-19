@ECHO OFF

TITLE EmbeddedMontiArcStudio
cd shared
call setup.bat
cd ..
cmd.exe /c ..\nodejs\node.exe ..\ide\server\server.js
PAUSE