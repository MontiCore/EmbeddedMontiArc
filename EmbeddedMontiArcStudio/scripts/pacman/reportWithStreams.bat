call "..\shared\variables.bat"
call "..\pacman\runAllTests.bat"
if exist "%REPORTING_HOME%\report\data" rmdir "%REPORTING_HOME%\report\data" /s /q
mkdir "%REPORTING_HOME%\report\data"
cd %REPORTING_HOME%
"%JAVA_HOME%\bin\java.exe" -jar "reporting.jar" ^
   "%HOME%\model\pacman" ^
   -tc "true" ^
   -zn "dummy.zip" ^
   -svg "true" ^
   -ist "%HOME%\testResults"
cd "%HOME%\scripts"