curDir=$(readlink -f `dirname $0`)
fileName="${curDir}/resources/emam-generator.jar"
curl -o "$fileName" "https://nexus.se.rwth-aachen.de/repository/public/de/monticore/lang/monticar/embedded-montiarc-math-generator/0.1.10/embedded-montiarc-math-generator-0.1.10-jar-with-dependencies.jar"
if [ ! -f "${fileName}" ]
then
	echo "Can not find generator jar"
	exit 1
fi

npm install
npm install -g vsce

if [ `command -v vsce` ]
then
	vsce package -o "${curDir}/emam-debug.vsix"
	exit $?
else
	echo "Can not find vsce. Aborting!"
	exit 1
fi