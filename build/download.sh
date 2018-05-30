mkdir -p ${dl}

cd ${dl}
echo "Downloading dependencies"

while read line; do
	# if line beginns with ** extract link
	if [[ ${line:0:2} = '**' ]]; then
		link=${line:3}
		#echo ${line}
		# extract filename
		filename=${link##*/}
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
