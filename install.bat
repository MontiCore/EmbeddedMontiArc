@REM
@REM (c) https://github.com/MontiCore/monticore
@REM
@REM The license generally applicable for this project
@REM can be found under https://github.com/MontiCore/monticore
@REM


@REM Make sure the script runs from its original location
pushd %~dp0

call mvn clean install -s "settings.xml" %*
popd
