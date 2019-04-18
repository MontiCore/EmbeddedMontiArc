@REM
@REM
@REM  ******************************************************************************
@REM  MontiCAR Modeling Family, www.se-rwth.de
@REM  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
@REM  All rights reserved.
@REM
@REM  This project is free software; you can redistribute it and/or
@REM  modify it under the terms of the GNU Lesser General Public
@REM  License as published by the Free Software Foundation; either
@REM  version 3.0 of the License, or (at your option) any later version.
@REM  This library is distributed in the hope that it will be useful,
@REM  but WITHOUT ANY WARRANTY; without even the implied warranty of
@REM  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
@REM  Lesser General Public License for more details.
@REM
@REM  You should have received a copy of the GNU Lesser General Public
@REM  License along with this project. If not, see <http://www.gnu.org/licenses/>.
@REM *******************************************************************************
@REM


call variables.bat
set STREAM_TESTING=%PROJECT_ROOT%\target\generated-sources-cpp\streamtest
set AUTOPILOT_TESTS=%STREAM_TESTING%\target\generated-sources-cpp\streamtest\autopilot
set STREAM_TEST_EXEC_DIR=%STREAM_TESTING%\exec
g++ -version

g++ -std=c++11 ^
   -I"%JAVA_HOME%\include_win" ^
   -I"%JAVA_HOME%\include_win\win32" ^
   -I"%ARMADILLO_HOME%\include" ^
   -L"%HOME%\lib\win" ^
   -o "%2\TestsForCurrentModel.exe" ^
   "%1/test/tests_main.cpp" ^
   -include %ARMADILLO_HOME%\include\armadillo ^
   -DARMA_DONT_USE_WRAPPER -lgdi32 -lopenblas -llibarpack-2
