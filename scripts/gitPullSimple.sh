#!/bin/bash

declare -a emaRepos=( "Documentation" "EMA2Kotlin" "EMAM2Cpp" "EMAM2PythonROS" "EMAM2RosCpp" "EmbeddedMontiArc" "EmbeddedMontiArcMath" \
                "MontiMath" "OCL" "OCL2Java" "Tagging" "Tagging-Examples" "ViewVerification" "visualisation" )
declare -a msRepos=( "CarLang" "server" "visualization" "SimLang" "simulation" "commons" "application" "example-autopilot-ema" \
                     "controller" "Documentation" )

# clone EmbeddedMontiArc
if [ ! -d "EmbeddedMontiArc" ]
then
  mkdir "EmbeddedMontiArc"
fi
cd EmbeddedMontiArc
for repo in "${emaRepos[@]}"
do
  if [ -d $repo ]
  then
    cd $repo && git pull && cd ..
  else
    git clone --depth 1 "https://github.com/EmbeddedMontiArc/$repo"
  fi
done
cd ..

# clone MontiSim
if [ ! -d "MontiSim" ]
then
  mkdir "MontiSim"
fi
cd MontiSim
for repo in "${msRepos[@]}"
do
  if [ -d $repo ]
  then
    cd $repo && git pull && cd ..
  else
    git clone --depth 1 "https://github.com/MontiSim/$repo"
  fi
done
cd ..