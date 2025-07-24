@echo off
setlocal
call setup.bat
call emam2wasm.bat
xcopy "%WASM_HOME%\%CURRENT_PROJECT%\wasm" "%INTERACTIVE_SIMULATOR_HOME%\html\%CURRENT_PROJECT%" /s /Y /i
endlocal