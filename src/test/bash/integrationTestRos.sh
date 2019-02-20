#!/bin/bash
set -e
baseDir=$(readlink -f `dirname $0`/../../..)

for f in `find $baseDir/target/generated-sources-cmake/ -name compile.sh`
do
	chmod +x $f
	bash -H $f
done
