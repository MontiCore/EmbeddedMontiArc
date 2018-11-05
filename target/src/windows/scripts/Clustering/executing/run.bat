@ECHO OFF
call "..\..\common\variables"

cd "%CLUSTERER_EXEC_DIR%"
"Clusterer.exe"

copy /Y "%CLUSTERER_EXEC_DIR%\result.bmp" "%HOME%\cluster-fiddle\img.bmp"