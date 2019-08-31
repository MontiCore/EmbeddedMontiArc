# (c) https://github.com/MontiCore/monticore
curDir=$(readlink -f `dirname $0`)
fileName="${curDir}/resources/emam-generator.jar"
curl -o "$fileName" "https://nexus.se.rwth-aachen.de/repository/public/de/monticore/lang/monticar/embedded-montiarc-math-generator/0.1.15-SNAPSHOT/embedded-montiarc-math-generator-0.1.15-20190830.195219-1-jar-with-dependencies.jar"
if [ ! -f "${fileName}" ]
then
	echo "Can not find generator jar"
	exit 1
fi

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
