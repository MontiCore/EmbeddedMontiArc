#!/bin/bash

#### Digits

# Mono

cd gluon-cpp-modular-digits-mono-small
./build.sh && ./executeStaticTest.sh d s
cd ..

cd gluon-cpp-modular-digits-mono-default
./build.sh && ./executeStaticTest.sh d s
cd ..

cd gluon-cpp-modular-digits-mono-big
./build.sh && ./executeStaticTest.sh d s
cd ..


# Simple

cd gluon-cpp-modular-digits-simple-small
./build.sh && ./executeStaticTest.sh d s
cd ..

cd gluon-cpp-modular-digits-simple-default
./build.sh && ./executeStaticTest.sh d s
cd ..

cd gluon-cpp-modular-digits-simple-big
./build.sh && ./executeStaticTest.sh d s
cd ..


# Complex

cd gluon-cpp-modular-digits-complex-small
./build.sh && ./executeStaticTest.sh d s
cd ..

cd gluon-cpp-modular-digits-complex-default
./build.sh && ./executeStaticTest.sh d s
cd ..

cd gluon-cpp-modular-digits-complex-big
./build.sh && ./executeStaticTest.sh d s
cd ..



#### Letters

cd gluon-cpp-modular-letters-mono
./build.sh && ./executeStaticTest.sh l s
cd ..

cd gluon-cpp-modular-letters-simple
./build.sh && ./executeStaticTest.sh l s
cd ..

cd gluon-cpp-modular-letters-complex
./build.sh && ./executeStaticTest.sh l s
cd ..

