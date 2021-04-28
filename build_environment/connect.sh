#!/bin/bash

if [ -z "$1" ] || [ "$1" == "--help" ]; then
    echo "Usage:"
    echo "    ./connect.sh <port>"
    echo "Description:"
    echo "    Connects to the Lab Raspberry Pi and enables Port Frowarding on the specified port."
    echo "    Use the same script on the Raspberry to tunnel from it to the specified VCG."
    echo "    The specified PORT is then locally available on 'localhost' (::1 for ipv6)."
    echo "Configuration:"
    echo "    Change the SSH_CONFIG variable in this script to set your login."
    echo "    (default: student@lablogin.se.rwth-aachen.de)"
    exit 0
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

pushd $SCRIPT_DIR

. get_config.sh config

ssh -L $1:localhost:$1 $SSH_CONFIG

popd