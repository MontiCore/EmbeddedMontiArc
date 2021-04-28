@echo off

if "%1"=="--help" (
    echo Description:
    echo     Starts the DSA image with the current folder (this script's folder^)
    echo     exposed to the container under '/home/wd' (by default, specified in 'config'^).
    echo     All files in the current folder are then accessible to the container for building.
    echo     Also attaches the 'externals' folder to the image.
    echo Configuration:
    echo     Change the IMAGE_NAME entry in 'config' if you have another image name.
    exit /B 0
)

pushd %~dp0

CALL get_config.bat config

set EXTERNALS_DIRECTORY=%~dp0%EXTERNALS_DIRECTORY%
set ARMADILLO_PATH=%IMAGE_EXTERNALS_DIRECTORY%/armadillo
set LIBDDC_PATH=%IMAGE_EXTERNALS_DIRECTORY%/libddc
set SHARED_CPP_PATH=%IMAGE_EXTERNALS_DIRECTORY%/shared_cpp

set WORKING_DIRECTORY=%~dp0

docker run -it --rm ^
  --mount type=bind,src=%EXTERNALS_DIRECTORY%,dst=%IMAGE_EXTERNALS_DIRECTORY% ^
  -e ARMADILLO_PATH=%ARMADILLO_PATH% ^
  -e LIBDDC_PATH=%LIBDDC_PATH% ^
  -e SHARED_CPP_PATH=%SHARED_CPP_PATH% ^
  --mount type=bind,src=%WORKING_DIRECTORY%,dst=%IMAGE_WORKING_DIRECTORY% ^
  -w %IMAGE_WORKING_DIRECTORY% ^
  %IMAGE_NAME%

popd