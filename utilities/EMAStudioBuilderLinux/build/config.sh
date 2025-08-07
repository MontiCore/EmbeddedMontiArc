#date="{0:yyyy-MM-dd HH.mm}" -f (get-date)
# (c) https://github.com/MontiCore/monticore  
date=`date '+%Y-%m-%d_%H:%M'`
# The Folder which will be present after self-extracting
targetDir="EMAStudio"
# The name of the self-extracting archive
targetName="EmbeddedMontiArcStudio.zip"
# Project items or dirs that need to be included
packingItems=("ide.sh setup.sh EmbeddedMontiArcStudio")
# Set to 0 to remove downloads in clean phase
cacheDownload=1

buildPrograms="curl unzip zip"  
build="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

dl=${build}"/download/"  
ext=${build}"/extracted/"
