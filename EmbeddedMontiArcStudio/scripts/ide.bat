@rem (c) https://github.com/MontiCore/monticore  
@ECHO OFF

TITLE EmbeddedMontiArcStudio
cd shared
call setup.bat
call variables.bat
call visualization.prepare.bat
cd ..
cmd.exe /c ..\nodejs\node.exe ..\ide\server\server.js
PAUSE
