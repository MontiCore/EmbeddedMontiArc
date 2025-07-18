mkdir -p ${targetDir}
# (c) https://github.com/MontiCore/monticore  

echo "Copy extracted dependencies to build"
mv -v ${ext}* ${targetDir}

echo "Copy project items to build"
for item in ${packingItems}; do
	item="../"${item}
	echo "copying ${item} to ${targetDir}"
	cp -r -a ${item} ${targetDir}
done

#echo "Packing to self-extracting exe"
echo "Packing to zip"
rm ${targetName}
zip -rq $targetName $targetDir
echo "Done"
