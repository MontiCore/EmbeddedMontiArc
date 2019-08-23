source config.sh
# (c) https://github.com/MontiCore/monticore  

if [ -z $ext ] || [ -z $build ] || [ -z $targetDir ] || [ -z $targetName ] || [ -z $dl ]
then
	echo "ERROR: Variables are unset. Aborting!"
	exit 1
fi

rm -rf ${ext}
rm -rf ${build}"/"${targetDir}*
rm -rf ${build}"/"${targetName}

if [ ! "$cacheDownload" == 1 ]; then
	echo rm -rf ${dl}
fi
