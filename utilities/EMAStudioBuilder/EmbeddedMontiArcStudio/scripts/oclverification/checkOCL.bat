@rem (c) https://github.com/MontiCore/monticore  
setlocal
echo "Checking OCL types"
pushd %~dp0
cd ..
cd ..
set HOME=%CD%
popd

cd ..\..\oclverification
del %CD%\data\result.txt

set PROJECT_PATH="%HOME%\model\oclverification"
set OCL_MODEL=%1


"%JAVA_HOME%\bin\java.exe" -jar ocl-1.2.3-cli.jar ^
    -path ^
    %PROJECT_PATH% ^
    -ocl ^
    %OCL_MODEL% > data/result.txt 2>&1

endlocal
