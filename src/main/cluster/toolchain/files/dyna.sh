#!/bin/bash

mv $HOME/Dokumente/topologyoptimizer/toolchain/files/Input.csv $HOME/Dokumente/topologyoptimizer/toolchain/preprocessing/raw/
cd $HOME/Dokumente/topologyoptimizer/toolchain/files/Lattice_Structures

$MPIEXEC $FLAGS_MPI_BATCH ls-dyna_mpp_s i=main.key