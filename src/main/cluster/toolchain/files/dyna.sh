#!/bin/bash

rm $HOME/Dokumente/topologyoptimizer/toolchain/files/Lattice_Structures/signalfile
cd $HOME/Dokumente/topologyoptimizer/toolchain/files/Lattice_Structures

$MPIEXEC $FLAGS_MPI_BATCH ls-dyna_mpp_s i=main.key
