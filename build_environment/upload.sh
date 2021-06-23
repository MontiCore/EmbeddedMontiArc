#!/bin/bash

if [ -z "$1" ] || [ -z "$2" ] || [ "$1" == "--help" ]; then
    echo "Usage:"
    echo "    ./upload.sh <folder> <vcg>"
    echo "    Where <vcg> is vcg1, vcg2 or vcg3."
    echo "Description:"
    echo "    Uploads the program in <folder>/build/ with name <folder> to the specified vcg."
    echo "    After the upload, the program can be found under '/data/pdata/programs/' in the APPCONT container."
    echo "Note:"
    echo "    <folder> must match the folder name AND the program name (CMake target name)."
    echo "Configuration:"
    echo "    Change the SSH_CONFIG variable in this script to set your login."
    echo "    (default: student@lablogin.se.rwth-aachen.de)"
    exit 0
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $SCRIPT_DIR

. get_config.sh config

PROGRAM_NAME=$1
FOLDER=$1
VCG=$2

scp $FOLDER/build/$PROGRAM_NAME $SSH_CONFIG:/home/student/upload/$PROGRAM_NAME
ssh $SSH_CONFIG "sh -c \"chmod u+x /home/student/upload/$PROGRAM_NAME && scp /home/student/upload/$PROGRAM_NAME $VCG:/data/pdata/lxc/appcont/rootfs/data/pdata/programs/$PROGRAM_NAME\""

popd