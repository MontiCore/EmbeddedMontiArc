setlocal ENABLEDELAYEDEXPANSION

REM get Model
REM "%JAVA_HOME%\bin\java.exe" -jar "%SVG_HOME%\emam2ema.jar" ^
REM   %HOME%\model ^
REM   %VERIFICATION_HOME%\modelEMA 
 
REM XCOPY /E %VERIFICATION_HOME%\modelEMA %HOME%\model
cd %HOME%\models\CNCVerification
set adress2=%~f1
REM set adress2=%adress2:\scripts\pump\model\=\model\% 

REM set count=1
REM set name=""
REM for /f "tokens=*" %%d in (%adress2%) do (
REM if !count! equ 3 (set name=%%d & GOTO NEXT)
REM set /a count+=1
REM )

REM :NEXT
REM set adress=%~dp1
REM set adress1=%adress:\scripts\pump\model\pump\views\=\viewverification\modelEMA\pump\model\%
REM set name=%name:~10,-3%
REM set adress1=%adress1%%name%.ema


REM create Witness
"%JAVA_HOME%\bin\java.exe" -jar "%VERIFICATION_HOME%\view-verification-0.0.2-SNAPSHOT-jar-with-dependencies.jar" ^
	%HOME%\models\CNCVerification\fas\demo_fas_Fkt_m\FAS.ema ^
	%adress2% ^
	%CNCVERIFICATION_HOME%\results
	
RMDIR "target" /s /q

REM mkdir "%VERIFICATION_HOME%\output"


set witnessadress=%CNCVERIFICATION_HOME%\results\witness\

set temp=%~n1
set temp=%temp:~0,1%
set "_UCASE=ABCDEFGHIJKLMNOPQRSTUVWXYZ"
set "_LCASE=abcdefghijklmnopqrstuvwxyz"

for /l %%b in (0,1,25) do (
   call set "_FROM=%%_UCASE:~%%b,1%%
   call set "_TO=%%_LCASE:~%%b,1%%
   call set "temp=%%temp:!_FROM!=!_TO!%%
)
set temp2=%~n1
set temp2=%temp2:~1%
set temp=%temp%%temp2%
set temp3=%~n1

REM set /p temp3=<%adress1
set adress3=%~dp1
set adress3=%adress3:\scripts\PumpStation\models\PumpStation\views\=\viewverification\results\witness\%
call set adress3=%%adress3:%witnessadress%=%%
set adress3=%adress3:\=.%

set FilePath=%CNCVERIFICATION_HOME%\results\inconsistency\%~n1.txt
set /a _Lines=0

For /f %%j in ('Type %FilePath%^|Find "" /v /c ') Do Set /a _Lines=%%j
REM set "number=findstr /R /N "" %VERIFICATION_HOME%\results\inconsistency\%~n1.txt | find /C ":"" 
set /a _Test=0

if %_Lines% neq %_Test% GOTO TRUE
GOTO FALSE

:TRUE
for /l %%C in (1,1,%_Lines%) do (
set "number=%%C"
"%JAVA_HOME%\bin\java.exe" -jar "%SVG_HOME%\embeddedmontiarc-svggenerator.jar" ^
   --input "%adress3%%temp%Witness!number!" ^
   --modelPath "%witnessadress%\" ^
   --recursiveDrawing "true" ^
   --outputPath "%CNCVERIFICATION_HOME%\WitnessSVG\\" 

set count=1
set text=""
for /f "tokens=*" %%d in (%FilePath%) do (
if !count! equ %%C (set text=%%d)
set /a count+=1
)

del %CNCVERIFICATION_HOME%\WitnessSVG\%temp3%Witness!number!.html 

(
echo ^<!DOCTYPE html^>
echo   ^<html^>^<body^>^<header^>Witness %temp%Witness!number! doesn't verify model %name%^</header^>
echo       ^<iframe src="%adress3%%temp%Witness!number!.html" height=400 width=800^>
echo   ^</iframe^>^<p^>!text!^</p^>
echo   ^</body^>^</html^>
) >> %CNCVERIFICATION_HOME%\WitnessSVG\%temp3%Witness!number!.html 

) 
GOTO END

:FALSE
for /l %%C in (1,1,1) do (
set "number=%%C"
"%JAVA_HOME%\bin\java.exe" -jar "%SVG_HOME%\embeddedmontiarc-svggenerator.jar" ^
   --input "%adress3%%temp%Witness!number!" ^
   --modelPath "%witnessadress%\" ^
   --recursiveDrawing "true" ^
   --outputPath "%CNCVERIFICATION_HOME%\WitnessSVG\\" 
   
del %CNCVERIFICATION_HOME%\WitnessSVG\%temp%Witness!number!.html 

(
echo ^<!DOCTYPE html^>
echo   ^<html^>^<body^>^<header^>Witness %temp%Witness!number! verifies model %name%^</header^>
echo       ^<iframe src="%adress3%%temp%Witness!number!.html" height=400 width=800^>
echo   ^</iframe^>^<p^>^</p^>
echo   ^</body^>^</html^>
) >> %CNCVERIFICATION_HOME%\WitnessSVG\%temp3%Witness!number!.html
)
:END