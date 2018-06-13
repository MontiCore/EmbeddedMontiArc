#!/bin/bash

echo "Installing dependencies!"
read -p "This script will download multiple GB of packages. Press [CTRL-C] to stop or  [ENTER] to continue." tmp

sudo EmbeddedMontiArcStudio/scripts/classifier/installMxNet.sh
sudo EmbeddedMontiArcStudio/scripts/intersection/setup.sh

