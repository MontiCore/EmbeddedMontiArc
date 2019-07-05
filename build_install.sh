#!/bin/bash

mvn clean install -s settings.xml $*
cd target
for x in basic-simulator-*.jar;do mv $x ../install/basic-simulator.jar;done
cd ..