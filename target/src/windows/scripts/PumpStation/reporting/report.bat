@ECHO OFF

call "..\..\common\variables.bat"

if exist "%REPORTING_HOME%\report\data" rmdir "%REPORTING_HOME%\report\data" /s /q
mkdir "%REPORTING_HOME%\report\data"

cd %REPORTING_HOME%

"%JAVA_HOME%\bin\java.exe" -jar "reporting.jar" ^
   "%HOME%\models\PumpStation" ^
   -tc "true" ^
   -zn "dummy.zip" ^
   -svg "true"

cd "%HOME%\scripts"