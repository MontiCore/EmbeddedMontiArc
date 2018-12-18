@echo off
setlocal
call setup.bat
call emam2wasm.bat
xcopy "%WASM_HOME%\%CURRENT_PROJECT%\wasm" "%INTERACTIVE_SIMULATOR_HOME%\html\%CURRENT_PROJECT%" /s /Y /i
call generateSVG.bat
xcopy "%INTERACTIVE_SIMULATOR_HOME%\SVG" "%INTERACTIVE_SIMULATOR_HOME%\html\%CURRENT_SVGMAIN%" /s /Y /i

endlocal