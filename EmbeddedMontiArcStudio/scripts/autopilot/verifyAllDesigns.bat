REM call reportinformation 
call "..\shared\variables.bat"
if exist "%HOME%\viewverification\results\" rmdir "%HOME%\viewverification\results\" /s /q
mkdir %HOME%\viewverification\results\


for /f "tokens=*" %%a in (%HOME%\viewverification\viewList.txt) do (
  call verifyDesign %%a
)
