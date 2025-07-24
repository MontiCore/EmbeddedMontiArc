@rem (c) https://github.com/MontiCore/monticore  
call reportinformation
if exist "%STREAM_TESTING%\testResults\" rmdir "%STREAM_TESTING%\testResults\" /s /q
if exist "%HOME%\testResults\" rmdir "%HOME%\testResults\" /s /q
mkdir %STREAM_TESTING%\testResults\
mkdir %HOME%\testResults\

for /f "tokens=*" %%a in (%STREAM_TESTING%\testInfo\testComponents.txt) do (
  call runAllTestsForComponent %%a
)
xcopy /s /y %STREAM_TESTING%\testResults %HOME%\testResults
REM call runAllTestsForComponent de.rwth.armin.modeling.autopilot.motion.decideEngineOrBrakes
