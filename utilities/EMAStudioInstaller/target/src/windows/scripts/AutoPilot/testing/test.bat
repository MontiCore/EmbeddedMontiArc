@ECHO OFF

if not exist "%HOME%\testResults\" mkdir %HOME%\testResults\

call generateTestsArmadilloBackend %1
call compileTestsArmadilloBackendOpenBLAS
call runCurrentExecutable %1