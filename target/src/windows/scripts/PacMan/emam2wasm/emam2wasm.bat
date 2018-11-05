@echo off
cd "..\..\common"
call setup
cd "..\PacMan\emam2wasm"
call emam2wasmGen_.bat "PacMan" de.rwth.pacman.pacManWrapper