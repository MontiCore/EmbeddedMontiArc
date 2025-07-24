@rem (c) https://github.com/MontiCore/monticore  
call reportinformation
if exist "%HOME%\testResults\" rmdir "%HOME%\testResults\" /s /q
mkdir %HOME%\testResults\


for /f "tokens=*" %%a in (%HOME%\testInfo\testComponents.txt) do (
  call runAllTestsForComponent %%a
)
REM call runAllTestsForComponent de.rwth.armin.modeling.autopilot.motion.decideEngineOrBrakes
