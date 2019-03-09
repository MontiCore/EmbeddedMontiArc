@echo off
for %%I in (.) do set CurrDirName=%%~nxI
IF "%CurrDirName%"=="docs" ( cd .. )
echo.
echo **************************************
echo   Building pe-parse (Release/debug)
echo **************************************
cd pe-parse
call ..\docs\build.bat
msbuild -verbosity:quiet build\pe-parse.sln /m /p:Configuration=Release /p:Platform=x64
msbuild -verbosity:quiet build\pe-parse.sln /m /p:Configuration=Debug /p:Platform=x64
cd ..


echo.
echo **************************************
echo    Building unicorn (Release/debug)
echo **************************************
cd unicorn
call ..\docs\build.bat
msbuild -verbosity:quiet build\unicorn.sln /m /p:Configuration=Release /p:Platform=x64
msbuild -verbosity:quiet build\unicorn.sln /m /p:Configuration=Debug /p:Platform=x64
cd ..


echo.
echo **************************************
echo     Building Zydis (Release/debug)
echo **************************************
cd zydis
call ..\docs\build.bat
msbuild -verbosity:quiet build\Zydis.sln /m /p:Configuration=Release /p:Platform=x64
msbuild -verbosity:quiet build\Zydis.sln /m /p:Configuration=Debug /p:Platform=x64
cd ..

mkdir hardware_emulator\libs\Release
mkdir hardware_emulator\libs\Debug

copy "pe-parse\build\pe-parser-library\Release\pe-parser-library.lib" "hardware_emulator\libs\Release\pe-parser-library.lib"
copy "pe-parse\build\pe-parser-library\Debug\pe-parser-library.lib" "hardware_emulator\libs\Debug\pe-parser-library.lib"
copy "pe-parse\build\pe-parser-library\Debug\pe-parser-library.pdb" "hardware_emulator\libs\Debug\pe-parser-library.pdb"

copy "unicorn\build\Release\unicorn.lib" "hardware_emulator\libs\Release\unicorn.lib"
copy "unicorn\build\qemu\Release\x86_64-softmmu.lib" "hardware_emulator\libs\Release\x86_64-softmmu.lib"
copy "unicorn\build\Debug\unicorn.lib" "hardware_emulator\libs\Debug\unicorn.lib"
copy "unicorn\build\qemu\Debug\x86_64-softmmu.lib" "hardware_emulator\libs\Debug\x86_64-softmmu.lib"
copy "unicorn\build\Debug\unicorn.pdb" "hardware_emulator\libs\Debug\unicorn.pdb"

copy "zydis\build\Release\Zydis.lib" "hardware_emulator\libs\Release\Zydis.lib"
copy "zydis\build\Debug\Zydis.lib" "hardware_emulator\libs\Debug\Zydis.lib"
copy "zydis\build\Debug\Zydis.pdb" "hardware_emulator\libs\Debug\Zydis.pdb"