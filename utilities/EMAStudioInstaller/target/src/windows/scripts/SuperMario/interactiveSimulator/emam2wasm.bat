@echo off
setlocal
call tocpp.bat
xcopy "%WASM_HOME%\%CURRENT_PROJECT%\toCopy\HelperA.h" "%WASM_HOME%\%CURRENT_PROJECT%\cpp\HelperA.h*" /s /Y
xcopy "%WASM_HOME%\%CURRENT_PROJECT%\toCopy\%CURRENT_MAIN_COMPONENT%.cpp" "%WASM_HOME%\%CURRENT_PROJECT%\cpp\%CURRENT_MAIN_COMPONENT%.cpp*" /s /Y
call towasm.bat
xcopy "%WASM_HOME%\%CURRENT_PROJECT%\toCopy\%CURRENT_MAIN_COMPONENT%_wrapper.js" "%WASM_HOME%\%CURRENT_PROJECT%\wasm\%CURRENT_MAIN_COMPONENT%_wrapper.js*" /s /Y
endlocal