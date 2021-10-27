set ARMADILLO_PATH=%~dp0%external\armadillo

set CMAKE_ARGS=-DARMADILLO_PATH="%ARMADILLO_PATH%"
set CMAKE_ARGS=%CMAKE_ARGS% -G"MinGW Makefiles"

set COMPONENT_ID=de.rwth.connectedcars.TestAutoPilot

pushd target\tmp
cmake %CMAKE_ARGS% -S cpp/%COMPONENT_ID% -B %COMPONENT_ID%
cmake --build %COMPONENT_ID% --config Release
popd