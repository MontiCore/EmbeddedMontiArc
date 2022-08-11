#!/bin/bash

directory=$(pwd)
. $directory/config.sh
cp -ar $directory/ ${SAVE_FOLDER}
cd ${SAVE_FOLDER}
echo Please enter the name of the new folder:
read copyfile_name

if [ -e "$copyfile_name" ]
then
    echo The file already exists, please enter a new name!
    read new_name
    mv "${DIR_NAME}" "$new_name"
else
    mv "${DIR_NAME}" "$copyfile_name"
fi
