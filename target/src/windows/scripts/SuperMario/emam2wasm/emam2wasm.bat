@echo off
setlocal
call setup.bat
call tocpp.bat
xcopy "%WASM_HOME%\%CURRENT_PROJECT%\toCopy\HelperA.h" "%WASM_HOME%\%CURRENT_PROJECT%\cpp\HelperA.h*" /s /Y
xcopy "%WASM_HOME%\%CURRENT_PROJECT%\toCopy\%Current_MAIN_COMPONENT%.cpp" "%WASM_HOME%\%CURRENT_PROJECT%\cpp\%Current_MAIN_COMPONENT%.cpp*" /s /Y
call towasm.bat
xcopy "%WASM_HOME%\%CURRENT_PROJECT%\toCopy\%Current_MAIN_COMPONENT%_wrapper.js" "%WASM_HOME%\%CURRENT_PROJECT%\wasm\%Current_MAIN_COMPONENT%_wrapper.js*" /s /Y
xcopy "%WASM_HOME%\%CURRENT_PROJECT%\wasm" "%HOME2%\%CURRENT_PROJECT%" /s /Y

endlocal