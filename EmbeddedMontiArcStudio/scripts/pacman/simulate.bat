cd "..\shared"
call variables.bat
cd "..\pacman"
if exist "%WASM_HOME%\pacman" xcopy /s /y "%WASM_HOME%\pacman" "%PACMAN_HOME%" 