@rem (c) https://github.com/MontiCore/monticore  
cd "..\shared"
call variables.bat
cd "..\supermario"
if exist "%WASM_HOME%\supermario" xcopy /s /y "%WASM_HOME%\supermario" "%SUPERMARIO_HOME%" 
