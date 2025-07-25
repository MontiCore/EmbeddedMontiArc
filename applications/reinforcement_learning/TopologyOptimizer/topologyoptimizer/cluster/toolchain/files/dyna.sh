#!/bin/bash

SCRIPT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
rm $SCRIPT_DIR/Lattice_Structures/signalfile
cd $SCRIPT_DIR/Lattice_Structures

$MPIEXEC $FLAGS_MPI_BATCH ls-dyna_mpp_s i=main.key
