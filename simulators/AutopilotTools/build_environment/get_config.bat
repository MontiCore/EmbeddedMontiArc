@echo off

for /F "tokens=*" %%A in (%1) do (
    FOR /f "tokens=1,2 delims==" %%a IN ("%%A") do (
        :: echo Entry %%a is %%b
        set %%a=%%b
    )
)
