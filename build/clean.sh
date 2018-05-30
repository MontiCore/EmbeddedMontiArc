source config.sh

rm -rf ${ext}
rm -rf ${build}"/"${targetDir}*
rm -rf ${build}"/"${targetName}

if [ ! "$cacheDownload" == 1 ]; then
	echo rm -rf ${dl}
fi
