#!/bin/sh
# (c) https://github.com/MontiCore/monticore  
cd resources
python download_data.py
python create_data_h5.py
mv test.h5 training_data/
mv train.h5 training_data/
cd ..
