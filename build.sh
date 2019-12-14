#!/bin/bash
set -o nounset                              # Treat unset variables as an error
set -e
curDir=$(readlink -f `dirname $0`)

if [ ! `command -v vsce` ]
then
	npm install -g vsce
fi

npm install --unsafe-perm
echo "y" | vsce package --baseContentUrl "https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/vscode-ema-linter/blob/master" -o "ema-linter.vsix"
