pushd %~dp0
cd "..\..\common"
call variablesWASM.bat
call setup.bat
popd
set CURRENT_PROJECT=pacman
set CURRENT_ROOT_MODEL=de.rwth.pacman.pacManWrapper
set Current_MAIN_COMPONENT=pacManWrapper