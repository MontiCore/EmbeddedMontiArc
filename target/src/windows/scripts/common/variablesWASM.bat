pushd %~dp0
cd "..\.."
set HOME2=%CD%
popd
set WASM_HOME=%HOME2%\emam2wasm
set INTERACTIVE_SIMULATOR_HOME=%HOME2%\interactiveSimulator
set EMSCRIPTEN_HOME=%HOME2%\emsdk\emscripten\1.38.12
set JAVA_HOME=%HOME2%\jdk
set SVG_HOME=%HOME2%\visualisation