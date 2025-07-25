dir=${ext}
# (c) https://github.com/MontiCore/monticore  

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

		#get filename
		filename=${link##*/}
		#fix sciebo format
		if [[ $filename = *"files="* ]]; then
			filename=$(echo $filename | sed -e 's/.*files=\(.*\)/\1/')
		fi
		# if dependencies.txt does not contain name, use curl to get it
		if [[ ! $filename = *".zip" ]]; then
			filename=$(curl -sI  $link | grep -o -E 'filename=.*$' | sed -e 's/filename=//' | sed -e 's/\"//g' | tr -d "\r")
		fi

		if [ ! -z "$filename" ]; then
			filename=${dl}${filename}
		    # if file does exist, extract
			if [ -f ${filename} ]; then
				echo "Extracting "${filename}" to "${dir}
				unzip -q -o -d ${dir} ${filename}
			else
				echo "File not found! ${filename}"
			fi
		fi
	fi
done <../dependencies.txt

echo "Fixing execute bit"
find $ext -name "*.sh" -print0 | xargs -0 chmod -v +x

echo "Fixing armadillo include name"
find $ext -name "armadillo" -exec cp -v "{}" "{}.h" \;
find $ext -name "armadillo.h" | sed 's/\.h$//' | xargs -i{} cp -v "{}.h" "{}" 
