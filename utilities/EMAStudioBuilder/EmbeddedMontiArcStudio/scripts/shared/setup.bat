@rem (c) https://github.com/MontiCore/monticore  
setlocal
If Not Exist setup (
    call ..\..\emam2wasm\scripts\setup
    echo.%CD%>setup
) Else (
    echo.%CD%>setupCompare
    fc setup setupCompare > nul
    if errorlevel 1 (
        call ..\..\emam2wasm\scripts\setup
        echo.%CD%>setup
    ) 
    del setupCompare
)
endlocal
