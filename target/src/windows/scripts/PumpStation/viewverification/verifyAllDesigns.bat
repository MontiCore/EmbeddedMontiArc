REM call reportinformation 
call "..\shared\variables.bat"
if exist "%HOME%\viewverification\results\" rmdir "%HOME%\viewverification\results\" /s /q
mkdir %HOME%\viewverification\results\

if exist "%HOME%\viewverification\WitnessSVG\" rmdir "%HOME%\viewverification\WitnessSVG\" /s /q
mkdir %HOME%\viewverification\WitnessSVG\


for /f "tokens=*" %%a in (%HOME%\scripts\pump\helper\viewListPump.txt) do (
  call verifyDesign2 %%a
)
