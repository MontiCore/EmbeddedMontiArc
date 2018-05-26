cd "..\shared"
call variables.bat
cd "..\supermario"
if exist "%WASM_HOME%\supermario" xcopy /s /y "%WASM_HOME%\supermario" "%PACMAN_HOME%" 