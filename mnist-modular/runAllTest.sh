#!/bin/bash


BUILD="echo Running without rebuilding"

if [ "$1" == "b" ]; then
    #BUILD=./build.sh  
    #echo "using small test runs"
fi

#### Digits

# Mono

cd gluon-cpp-modular-digits-mono-small
$BUILD && ./executeStaticTest.sh d s
cd ..

cd gluon-cpp-modular-digits-mono-default
$BUILD && ./executeStaticTest.sh d s
cd ..

cd gluon-cpp-modular-digits-mono-big
$BUILD ./executeStaticTest.sh d s
cd ..


# Simple

cd gluon-cpp-modular-digits-simple-small
$BUILD && ./executeStaticTest.sh d s
cd ..

cd gluon-cpp-modular-digits-simple-default
$BUILD && ./executeStaticTest.sh d s
cd ..

cd gluon-cpp-modular-digits-simple-big
$BUILD && ./executeStaticTest.sh d s
cd ..


# Complex

cd gluon-cpp-modular-digits-complex-small
$BUILD && ./executeStaticTest.sh d s
cd ..

cd gluon-cpp-modular-digits-complex-default
$BUILD && ./executeStaticTest.sh d s
cd ..

cd gluon-cpp-modular-digits-complex-big
$BUILD && ./executeStaticTest.sh d s
cd ..



#### Letters

cd gluon-cpp-modular-letters-mono
$BUILD && ./executeStaticTest.sh l s
cd ..

cd gluon-cpp-modular-letters-simple
$BUILD && ./executeStaticTest.sh l s
cd ..

cd gluon-cpp-modular-letters-complex
$BUILD && ./executeStaticTest.sh l s
cd ..

