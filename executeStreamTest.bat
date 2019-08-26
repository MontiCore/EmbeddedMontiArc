@rem (c) https://github.com/MontiCore/monticore  

subst N: .
cd /D N:
cd native
call variables.bat
cd %PROJECT_ROOT%%1
xcopy /s /y "N:\native\lib\win" "%PROJECT_ROOT%%1"
call TestsForCurrentModel.exe
