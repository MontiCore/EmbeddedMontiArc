echo "Loading config!"
# (c) https://github.com/MontiCore/monticore  
echo ""
source ./config.sh

echo -e "date\t\t=\t"${date}
echo -e "targetDir\t=\t"${targetDir}
echo -e "targetName\t=\t"${targetName}
echo -e "packingItems\t=\t"${packingItems}
echo -e "cacheDownload\t=\t"${cacheDownload}

echo -e "dl\t\t=\t"${dl}
echo -e "ext\t\t=\t"${ext}
echo -e "build\t\t=\t"${build}
echo ""

source ./tools.sh
if [ "$foundAll" = true ]; then
	source ./download.sh
	source ./extract.sh
	source ./pack.sh
fi
