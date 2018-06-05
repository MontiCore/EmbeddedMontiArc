cd "..\shared"
call variables.bat
cd "..\supermario"
if exist "%REPORTING_HOME%\report\data" rmdir "%REPORTING_HOME%\report\data" /s /q
mkdir "%REPORTING_HOME%\report\data"
cd %REPORTING_HOME%
"%JAVA_HOME%\bin\java.exe" -jar "reporting.jar" ^
   "%HOME%\model\supermario" ^
   -tc "true" ^
   -zn "dummy.zip" ^
   -svg "true"
cd "%HOME%\scripts"