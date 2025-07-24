REM call reportinformation 
call "..\..\common\variables.bat"
if exist "%CNCVERIFICATION_HOME%\results\" rmdir "%CNCVERIFICATION_HOME%\results\" /s /q
mkdir %CNCVERIFICATION_HOME%\results\

if exist "%CNCVERIFICATION_HOME%WitnessSVG\" rmdir "%CNCVERIFICATION_HOME%WitnessSVG\" /s /q
mkdir %CNCVERIFICATION_HOME%WitnessSVG\

for /r "%HOME%\models\CNCVerification\fas_views" %%a in (*.emv) do (
  call verifyDesign2 %%a
)

PAUSE