@ECHO OFF

call "..\..\common\variables"

if exist "%WASM_HOME%\pacman" xcopy /s /y "%WASM_HOME%\pacman" "%PACMAN_HOME%" 