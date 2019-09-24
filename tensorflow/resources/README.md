# Udacity Data generation

This readme describes the geration of the test and train sets we used. 
Predefined datasets can be downloaded (here).

## Udacity data download 

The raw data, which needs to be first prerpocessed and then turned into an hdf5 file format, such that it can be used in training,  can be downloaded here https://github.com/udacity/self-driving-car/tree/master/datasets . In our experiments we used the Challenge 2 Driving Data. 
Follow the download instructions and use the Udacity Reader tool to extract the .bag files.

## Usage of jpg2hdf5.py script 

|  | Predict | Visualise | Command |
|:---|:---|:---|:---|
| 1 | &#9745; | | `python3 v-tool/cli.py -i test.h5 -p` |
| 2 | &#9745; | &#9745; | `python3 v-tool/cli.py -i test.h5 -p -v` |













