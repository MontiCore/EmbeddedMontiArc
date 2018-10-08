@ECHO OFF
setlocal
echo Checking OCL types
pushd %~dp0
cd ..\..\..
set HOME=%CD%
popd

cd ..\..\..\oclverification

set PROJECT_PATH="%HOME%\models\OCLVerification"
set OCL_MODEL=%1

"%JAVA_HOME%\bin\java.exe" -jar ocl-1.2.3-cli.jar ^
    -path ^
    %PROJECT_PATH% ^
    -ocl ^
    %OCL_MODEL%

endlocal
