@ECHO Off
:: add *_HOME to PATH temporarily
IF NOT [%cmake_HOME%] == [] (
   set PATH="%cmake_HOME%;%PATH%"
)
IF NOT [%make_HOME%] == [] (
   set PATH="%make_HOME%;%PATH%"
)
IF NOT [%g++_HOME%] == [] (
   set PATH="%g++_HOME%;%PATH%"
)

:: source additional environment variables


:: check if needed programs are in PATH
where cmake
IF NOT %ERRORLEVEL% EQU 0 (
   echo "Can not find cmake in PATH! Aborting."
   exit /B 1
)
where make
IF NOT %ERRORLEVEL% EQU 0 (
   echo "Can not find make in PATH! Aborting."
   exit /B 1
)
where g++
IF NOT %ERRORLEVEL% EQU 0 (
   echo "Can not find g++ in PATH! Aborting."
   exit /B 1
)

:: cmake
cmake -B./build/ -G "MinGW Makefiles" %* ./src

:: make
cd .\build
make
cd ..