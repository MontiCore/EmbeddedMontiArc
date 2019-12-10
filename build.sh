#!/bin/bash
set -o nounset                              # Treat unset variables as an error
set -e
curDir=$(readlink -f `dirname $0`)

if [ ! `command -v vsce` ]
then
	npm install -g vsce
fi

mvn install:install-file \
 -Dfile="ema-lsp-1.3-SNAPSHOT.jar" \
 -DgroupId=de.monticore.lang.monticar \
 -DartifactId=ema-lsp \
 -Dversion=1.3-SNAPSHOT \

npm install --unsafe-perm
echo "y" | vsce package --baseContentUrl "https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/vscode-ema-linter/blob/emadl" -o "ema-linter.vsix"