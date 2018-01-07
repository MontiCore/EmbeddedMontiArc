#!/bin/bash
wget https://s3.amazonaws.com/mozilla-games/emscripten/releases/emsdk-portable.tar.gz
tar -xvf emsdk-portable.tar.gz
cd emsdk-portable
emsdk update
emsdk install latest
emsdk activate latest
pwd
ls -al