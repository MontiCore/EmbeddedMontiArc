@rem (c) https://github.com/MontiCore/monticore  
setlocal
echo "Running CD Visualization"

cd ..\..\oclverification


set CDSTRING=%1

echo "CDSTRING: %CDSTRING%"


"%JAVA_HOME%\bin\java.exe" -jar ocl-1.2.3-cli.jar ^
    --printCDSrc ^
    %CDSTRING% ^
    --printCDTgt ^
    "data/plantUML.txt" ^
    --showAttributes ^
    --showAssociationNames ^
    --showRoleNames

endlocal
