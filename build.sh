#!/bin/bash
set -o nounset                              # Treat unset variables as an error
set -e
curDir=$(readlink -f `dirname $0`)

if [ ! `command -v vsce` ]
then
	npm install -g vsce
fi

npm install --unsafe-perm
echo "y" | vsce package --baseContentUrl "https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/ema-lsp/tree/treiber/dl" -o "ema-linter.vsix"