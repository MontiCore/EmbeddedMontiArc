# (c) https://github.com/MontiCore/monticore
curDir=$(readlink -f `dirname $0`)

npm install --unsafe-perm
if [ ! $? ]
then
	echo "Error running npm install"
	exit 1
fi
if [ ! `command -v vsce` ]
then
	npm install -g vsce
fi

if [ `command -v vsce` ]
then
	vsce package -o "${curDir}/emam-debug.vsix" --baseImagesUrl "https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/emam-debugger-vscode/raw/master/"
	exit $?
else
	echo "Can not find vsce. Aborting!"
	exit 1
fi
