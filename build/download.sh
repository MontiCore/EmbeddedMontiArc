mkdir -p ${dl}
# (c) https://github.com/MontiCore/monticore  

cd ${dl}
echo "Downloading dependencies"

while read line; do
	# if line beginns with ** extract link
	if [[ ${line:0:2} = '**' ]]; then
		link=${line:3}
		#echo ${line}

		# get filename
		filename=${link##*/}
		#fix sciebo format
		if [[ $filename = *"files="* ]]; then
			filename=$(echo $filename | sed -e 's/.*files=\(.*\)/\1/')
		fi

		# if dependencies.txt does not contain name, use curl to get it
		if [[ ! $filename = *".zip" ]]; then
			filename=$(curl -sI  $link | grep -o -E 'filename=.*$' | sed -e 's/filename=//' | tr -d "\"\r")
		fi

		if [ ! -z "$filename" ]; then
			# if file does not exist, download
			if [ ! -f ${filename} ]; then
				echo "Downloading "${filename}" from: "$link
				curl -LJO ${link}
			fi
		fi
	fi
done <../../dependencies.txt

cd ${build}
