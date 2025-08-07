#!/bin/bash

cd ./*environment
mvn clean
mvn package -s settings.xml
