@REM
@REM (c) https://github.com/MontiCore/monticore
@REM
@REM The license generally applicable for this project
@REM can be found under https://github.com/MontiCore/monticore.
@REM


@REM -------------------------------------------------------------------------------------------------------------
@REM This script compiles all the libraries required by the hardware_emulator and places them in the 'libs' folder
@REM -------------------------------------------------------------------------------------------------------------

@echo off
set SCRIPTS_DIR=%~dp0
set ROOT_DIR=%~dp0\..\..
set LIBS_DIR=%ROOT_DIR%\hardware_emulator\libs\vs
if not exist %LIBS_DIR% mkdir %LIBS_DIR%

IF [%1] == [] (SET DEP_TARGET=Release) else (SET DEP_TARGET=%1)

pushd %ROOT_DIR%

echo.
echo [SCRIPT] Building dependency: pe-parse...
pushd pe-parse
call %SCRIPTS_DIR%\build_compile.bat pe-parse %DEP_TARGET%
echo [SCRIPT] Copying pe-parse library...
copy "build\pe-parser-library\%DEP_TARGET%\pe-parser-library.lib" "%LIBS_DIR%\pe-parser-library.lib"
IF ["%DEP_TARGET%"] == ["Debug"] (copy "build\pe-parser-library\%DEP_TARGET%\pe-parser-library.pdb" "%LIBS_DIR%\pe-parser-library.pdb")
popd

echo.
echo [SCRIPT] Building dependency: Zydis...
pushd zydis
call %SCRIPTS_DIR%\build_compile.bat Zydis %DEP_TARGET%
echo [SCRIPT] Copying Zydis library...
copy "build\%DEP_TARGET%\Zydis.lib" "%LIBS_DIR%\Zydis.lib"
IF ["%DEP_TARGET%"] == ["Debug"] (copy "build\%DEP_TARGET%\Zydis.pdb" "%LIBS_DIR%\Zydis.pdb")
popd

echo.
echo [SCRIPT] Building dependency: unicorn...
pushd unicorn
call %SCRIPTS_DIR%\build_compile.bat unicorn %DEP_TARGET%
echo [SCRIPT] Copying unicorn library...
copy "build\%DEP_TARGET%\unicorn.lib" "%LIBS_DIR%\unicorn.lib"
copy "build\qemu\%DEP_TARGET%\x86_64-softmmu.lib" "%LIBS_DIR%\x86_64-softmmu.lib"
IF ["%DEP_TARGET%"] == ["Debug"] (
    copy "build\%DEP_TARGET%\unicorn.pdb" "%LIBS_DIR%\unicorn.pdb";
    copy "build\qemu\%DEP_TARGET%\x86_64-softmmu.pdb" "%LIBS_DIR%\x86_64-softmmu.pdb"
    )

popd

popd
