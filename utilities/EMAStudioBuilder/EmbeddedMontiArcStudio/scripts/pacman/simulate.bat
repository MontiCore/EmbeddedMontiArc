@rem (c) https://github.com/MontiCore/monticore  
cd "..\shared"
call variables.bat
cd "..\pacman"
if exist "%WASM_HOME%\pacman" xcopy /s /y "%WASM_HOME%\pacman" "%PACMAN_HOME%" 
