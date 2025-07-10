@rem (c) https://github.com/MontiCore/monticore  
setlocal
pushd %~dp0
cd ..
cd ..
set HOME2=%CD%
popd
set WASM_HOME=%HOME2%\emam2wasm
set JAVA_HOME=%HOME2%\jdk

set PROJECTNAME=%1
set QUALIFIEDNAME=%2

echo "Projectname: %PROJECTNAME% \n"
echo "Modelname  : %QUALIFIEDNAME%"

if exist "%WASM_HOME%\%PROJECTNAME%" rmdir "%WASM_HOME%\%PROJECTNAME%" /s /q
mkdir "%WASM_HOME%\%PROJECTNAME%"
cd %WASM_HOME%
"%JAVA_HOME%\bin\java.exe" -jar emam2wasm.jar ^
    --cpp-dir=target ^
    --wasm-dir=%PROJECTNAME% ^
    --web-dir=%PROJECTNAME% ^
    --model-path=%HOME2%/model/%PROJECTNAME% ^
    --model=%QUALIFIEDNAME% ^
    --include=armadillo-code-8.400.x/include ^
    --library=lib --flag=std=c++11,llibblas_WIN64,O3 ^
    --option=WASM=1,NO_FILESYSTEM=1,ALLOW_MEMORY_GROWTH=1 ^
    --bind=true ^
    --algebraic-optimization=false ^
    --spring.profiles.active=compiler ^
    -Dspring.config.location=.\application.properties
endlocal
