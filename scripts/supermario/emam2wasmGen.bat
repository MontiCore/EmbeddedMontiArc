if not exist SetupExecuted.txt (
  pushd %~dp0
  cd ../../emam2wasm/scripts
  call setup.bat
  popd
  echo. 2>SetupExecuted.txt
)
call emam2wasmGen_.bat "supermario" de.rwth.supermario.superMarioWrapper