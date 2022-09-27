echo "Copying library_code to target directory.."
cp -r src/main/other_code/library_code/* target
echo "Copying pipeline_code to target directory.."
cp -r src/main/other_code/pipeline_code/* target
cd target
echo "Executing pipeline executor script"
python -c'import Pipeline_Executor; Pipeline_Executor.execute()'
echo "Training done"
cd ..
/bin/bash

