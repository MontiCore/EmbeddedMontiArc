@echo off

echo.
echo **************************************
echo            Making pe-parse
echo **************************************
cd pe-parse
call ..\docs\build.bat
echo.
echo **************************************
echo       Building pe-parse (Debug)
echo **************************************
msbuild -verbosity:quiet build\pe-parse.sln /m /p:Configuration=Debug /p:Platform=x64
echo.
echo **************************************
echo      Building pe-parse (Release)
echo **************************************
msbuild -verbosity:quiet build\pe-parse.sln /m /p:Configuration=Release /p:Platform=x64
cd ..


echo.
echo **************************************
echo            Making unicorn
echo **************************************
cd unicorn
call ..\docs\build.bat
echo.
echo **************************************
echo       Building unicorn (Debug)
echo **************************************
msbuild -verbosity:quiet build\unicorn.sln /m /p:Configuration=Debug /p:Platform=x64
echo.
echo **************************************
echo      Building unicorn (Release)
echo **************************************
msbuild -verbosity:quiet build\unicorn.sln /m /p:Configuration=Release /p:Platform=x64
cd ..


echo.
echo **************************************
echo             Making Zydis
echo **************************************
cd zydis
call ..\docs\build.bat
echo.
echo **************************************
echo         Building Zydis (Debug)
echo **************************************
msbuild -verbosity:quiet build\Zydis.sln /m /p:Configuration=Debug /p:Platform=x64
echo.
echo **************************************
echo        Building Zydis (Release)
echo **************************************
msbuild -verbosity:quiet build\Zydis.sln /m /p:Configuration=Release /p:Platform=x64
cd ..

echo.
echo **************************************
echo       Making Hardware Emulator
echo **************************************
cd hardware_emulator
call ..\docs\build.bat
echo.
echo **************************************
echo   Building Hardware Emulator (Debug)
echo **************************************
msbuild -verbosity:quiet build\hardware-emulator.sln /m /p:Configuration=Debug /p:Platform=x64
echo.
echo **************************************
echo  Building Hardware Emulator (Release)
echo **************************************
msbuild -verbosity:quiet build\hardware-emulator.sln /m /p:Configuration=Release /p:Platform=x64
cd ..