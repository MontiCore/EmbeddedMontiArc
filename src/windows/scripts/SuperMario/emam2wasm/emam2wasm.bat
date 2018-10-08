@echo off
cd "..\..\common"
call setup
cd "..\SuperMario\emam2wasm"
call emam2wasmGen_.bat "SuperMario" de.rwth.supermario.superMarioWrapper