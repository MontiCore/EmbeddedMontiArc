call variables.bat
echo Test Results for model %1: >> %TEST_EXEC_DIR%\result.txt
%TEST_EXEC_DIR%\TestsForCurrentModel.exe >> %TEST_EXEC_DIR%\result.txt