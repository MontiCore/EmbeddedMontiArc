#!/bin/bash
set -e

for f in `find . -name compile.sh`
do
	chmod +x $f
	bash -H $f
done
