#!/bin/bash

echo "Installing dependencies!"
if [ "$EUID" -ne 0 ]
  then echo "Please run as root to install dependencies."
  exit
fi

read -p "This script will download multiple GB of packages. Press [CTRL-C] to stop or  [ENTER] to continue." tmp

EmbeddedMontiArcStudio/scripts/intersection/setup.sh
