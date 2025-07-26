#!/bin/bash
#
# (c) https://github.com/MontiCore/monticore
#
# The license generally applicable for this project
# can be found under https://github.com/MontiCore/monticore.
#



# Get the folder containing this script to locate the project correclty
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
# Make script folder the current directory
pushd $DIR


# Make sure OpenModelica is installed:
if ! hash omc 2>/dev/null; then
    echo "Could not find OMC, trying to insall..."
    for deb in deb deb-src; do echo "$deb http://build.openmodelica.org/apt `lsb_release -cs` nightly"; done | sudo tee /etc/apt/sources.list.d/openmodelica.list
    wget -q http://build.openmodelica.org/apt/openmodelica.asc -O- | sudo apt-key add - 
    sudo apt update
    sudo apt install openmodelica
fi

# Configuration:
ext=".fmu"
PATH_TO_RES="$DIR/src/main/resources/linux/lib/"

# Locate FMI includes
FMI_LOC=`find /usr/include/ -name "fmi2Functions.h" -print -quit`
if [ -z "$FMI_LOC" ]; then 
    FMI_LOC=`find / -name "fmi2Functions.h" -print -quit` 
fi
if [ -z "$FMI_LOC" ]; then 
    echo "Could not find FMI include headers"
    exit
fi

PATH_TO_FMI="${FMI_LOC%/fmi2Functions.h}"
echo "Found FMI includes: $PATH_TO_FMI"


pushd modelica

# For all .mo files in the folder
for i in *.mo; do
    [ -f "$i" ] || break
    filename="${i%.*}"
    echo "Exporting and building $filename FMU..."
    
    # Creating build script for OpenModelica
    > build.mos
    echo "loadModel(Modelica);" >> build.mos
    echo "loadFile(\"$filename.mo\"); getErrorString(true);" >> build.mos
    echo "translateModelFMU($filename, fmuType=\"cs\"); getErrorString(true);" >> build.mos
    # echo "Build.mos content:"
    # cat build.mos
    
    # Exporting FMU
    omc build.mos
    
    # Unzipping and compiling the FMU
    unzip "$filename$ext" -d "$filename"
    rm "$filename$ext"
    pushd $filename/sources/
    ./configure CPPFLAGS="-I$HOME/local/include -I$PATH_TO_FMI"
    make
    popd

    #Zip FMU
    pushd $filename
    zip -r "../$filename$ext" *
    popd

    #Copy FMU to resources directory
    rm "$PATH_TO_RES$filename$ext"
    cp "$filename$ext" "$PATH_TO_RES"
    
    # Clean-up
    rm -r "$filename"
done

# Clean-up
rm *.libs
rm *.log
rm *.makefile
rm build.mos
rm *.fmu

popd

popd