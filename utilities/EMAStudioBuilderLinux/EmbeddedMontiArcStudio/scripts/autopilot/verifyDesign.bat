@rem (c) https://github.com/MontiCore/monticore  
REM call reportinformation 
call "..\shared\variables.bat"
if exist "%HOME%\viewverification\results\" rmdir "%HOME%\viewverification\results\" /s /q
mkdir %HOME%\viewverification\results\

if exist "%HOME%\viewverification\modelEMA\" rmdir "%HOME%\viewverification\modelEMA\" /s /q
mkdir %HOME%\viewverification\modelEMA\

if exist "%HOME%\viewverification\WitnessSVG\" rmdir "%HOME%\viewverification\WitnessSVG\" /s /q
mkdir %HOME%\viewverification\WitnessSVG\

call verifyDesign2 %1

