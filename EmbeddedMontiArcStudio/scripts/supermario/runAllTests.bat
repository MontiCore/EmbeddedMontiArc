call reportinformation
if exist "%STREAM_TESTING%\testResults\" rmdir "%STREAM_TESTING%\testResults\" /s /q
mkdir %STREAM_TESTING%\testResults\


for /f "tokens=*" %%a in (%STREAM_TESTING%\testInfo\testComponents.txt) do (
  call runAllTestsForComponent %%a
)
REM call runAllTestsForComponent de.rwth.armin.modeling.autopilot.motion.decideEngineOrBrakes