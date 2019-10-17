#!/bin/bash
set -o nounset                              # Treat unset variables as an error
set -e
curDir=$(readlink -f `dirname $0`)

if [ ! `command -v vsce` ]
then
	npm install -g vsce
fi

npm install --unsafe-perm
echo "y" | vsce package -o "ema-linter.vsix"