@ECHO OFF

call reportinformation

if exist "%HOME%\testResults\" rmdir "%HOME%\testResults\" /s /q
mkdir %HOME%\testResults\

REM call runAllTestsForComponent de.rwth.armin.modeling.autopilot.motion.decideEngineOrBrakes
for /f "tokens=*" %%a in (%HOME%\testInfo\testComponents.txt) do (
  call test %%a
)