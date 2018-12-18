pushd %~dp0
cd "..\..\common"
call variablesWASM.bat
call setup.bat
popd
set CURRENT_PROJECT=supermario
set CURRENT_ROOT_MODEL=de.rwth.supermario.superMarioWrapper
set Current_MAIN_COMPONENT=superMarioWrapper
set CURRENT_SVGMAIN=SVGSupermario