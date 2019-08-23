@rem (c) https://github.com/MontiCore/monticore  
REM call reportinformation 
call "..\shared\variables.bat"
if exist "%CNCVERIFICATION_HOME%\results\" rmdir "%CNCVERIFICATION_HOME%\results\" /s /q
mkdir %CNCVERIFICATION_HOME%\results\

if exist "%CNCVERIFICATION_HOME%WitnessSVG\" rmdir "%CNCVERIFICATION_HOME%WitnessSVG\" /s /q
mkdir %CNCVERIFICATION_HOME%WitnessSVG\

call verifyDesign2 %1

