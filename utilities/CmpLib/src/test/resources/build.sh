#!/bin/sh
# (c) https://github.com/MontiCore/monticore  


g++ -DCATCH_CONFIG_MAIN=1 -I"$1" -o tests_main.exec tests_main.cpp -DARMA_DONT_USE_WRAPPER -lopenblas
