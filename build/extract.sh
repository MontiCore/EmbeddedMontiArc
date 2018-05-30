dir=${ext}

mkdir -p ${ext}

echo "Extracting depencencies."

while read line; do
	# if line beginns with * it is a extraction
	if [[ ${line:0:2} = '* ' ]]; then
		dir=${ext}${line:2}
		mkdir -p ${dir}
	fi

	if [[ ${line:0:2} = '**' ]]; then
		link=${line:3}
		# extract filename
		filename=${link##*/}
		if [ ! -z "$filename" ]; then
			filename=${dl}${filename}
		    # if file does exist, extract
			if [ -f ${filename} ]; then
				echo "Extracting "${filename}" to "${dir}
				unzip -q -o -d ${dir} ${filename} 
			fi
		fi
	fi
done <../dependencies.txt
