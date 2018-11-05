@echo off

call "..\..\common\variables"

if exist "%WASM_HOME%\supermario" xcopy /s /y "%WASM_HOME%\supermario" "%SUPERMARIO_HOME%" 