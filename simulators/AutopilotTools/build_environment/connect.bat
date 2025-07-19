@echo off

set Help=0
if "%1"=="--help" set Help=1
if [%1] == [] set Help=1
if [%Help%] == [1] (
    echo Usage:"
    echo     connect.sh ^<port^>
    echo Description:
    echo     Connects to the Lab Raspberry Pi and enables Port Frowarding on the specified port.
    echo     Use the same script on the Raspberry to tunnel from it to the specified VCG.
    echo     The specified PORT is then locally available on 'localhost' (::1 for ipv6^).
    echo Configuration:
    echo     Change the SSH_CONFIG variable in this script to set your login.
    echo     (default: student@lablogin.se.rwth-aachen.de^)
    exit /B 0
)

pushd %~dp0

CALL get_config.bat config

ssh -L %1:localhost:%1 %SSH_CONFIG%

popd