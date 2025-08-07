@REM
@REM (c) https://github.com/MontiCore/monticore
@REM

pushd %~dp0
call mvn clean install -s "settings.xml" %*
popd