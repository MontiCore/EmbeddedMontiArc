echo "checking if all build programs are installed"
# (c) https://github.com/MontiCore/monticore  
foundAll=true
for p in ${buildPrograms}; do
	tmpPresent=`dpkg -l | grep ${p}`
	if [ ! -z "$tmpPresent" ]; then
		echo "${p} found!"
	else
		echo "Please install ${p}!"
		foundAll=false
	fi
done
