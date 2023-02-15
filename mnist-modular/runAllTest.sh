#!/bin/bash


BUILD="echo Running without rebuilding"

if [ "$1" == "b" ]; then
    #BUILD=./build.sh  
    echo "trying to recompile. Disabled currently. Adapt script."
fi

#### Digits

# Mono

cd digits-mono-small
$BUILD && ./executeStaticTest.sh d s
cd ..

cd digits-mono-default
$BUILD && ./executeStaticTest.sh d s
cd ..

cd digits-mono-big
$BUILD ./executeStaticTest.sh d s
cd ..


# Simple

cd digits-simple-small
$BUILD && ./executeStaticTest.sh d s
cd ..

cd digits-simple-default
$BUILD && ./executeStaticTest.sh d s
cd ..

cd digits-simple-big
$BUILD && ./executeStaticTest.sh d s
cd ..


# Complex

cd digits-complex-small
$BUILD && ./executeStaticTest.sh d s
cd ..

cd digits-complex-default
$BUILD && ./executeStaticTest.sh d s
cd ..

cd digits-complex-big
$BUILD && ./executeStaticTest.sh d s
cd ..



#### Letters

cd letters-mono
$BUILD && ./executeStaticTest.sh l s
cd ..

cd letters-simple
$BUILD && ./executeStaticTest.sh l s
cd ..

cd letters-complex
$BUILD && ./executeStaticTest.sh l s
cd ..

