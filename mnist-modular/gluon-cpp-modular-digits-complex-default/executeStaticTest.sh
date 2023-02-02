#!/bin/bash

SMALL=""

if [ "$2" == "s" ]; then
    SMALL=-small
    echo "using small test runs"
fi

if [ "$1" == "d" ]; then
    RUN="python3 ../TestRunner.py ../digits-static-run$SMALL.txt digits"
    echo running:
    echo $RUN
    #python3 ../TestRunner.py ../digits-static-run$SMALL.txt digits
    $RUN
fi

if [ "$1" == "l" ]; then
    RUN="python3 ../TestRunner.py ../letters-static-run$SMALL.txt letters"
    echo running:
    echo $RUN
    #python3 ../TestRunner.py ../letters-static-run$SMALL.txt letters
    $RUN
fi
