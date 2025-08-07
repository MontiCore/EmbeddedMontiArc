@echo off

call "..\..\common\variables"

cd "..\testing"

call "test.all"

if exist "%REPORTING_HOME%\report\data" rmdir "%REPORTING_HOME%\report\data" /s /q
mkdir "%REPORTING_HOME%\report\data"

cd %REPORTING_HOME%

"%JAVA_HOME%\bin\java.exe" -jar "reporting.jar" ^
   "%HOME%\models\PacMan" ^
   -tc "true" ^
   -zn "dummy.zip" ^
   -svg "true" ^
   -ist "%STREAM_TESTING%\testResults"

cd "%HOME%\scripts"