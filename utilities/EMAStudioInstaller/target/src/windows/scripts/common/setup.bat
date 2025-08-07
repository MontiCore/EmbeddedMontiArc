setlocal
If Not Exist "setup" (
    @call "%~dp0..\..\emsdk\emsdk.bat" activate latest
    echo.%CD%>setup
) Else (
    echo.%CD%>setupCompare
    fc setup setupCompare > nul
    if errorlevel 1 (
        @call "%~dp0..\..\emsdk\emsdk.bat" activate latest
        echo.%CD%>setup
    ) 
    del setupCompare
)
endlocal