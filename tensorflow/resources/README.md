# Udacity Data generation

This readme describes the geration of the test and train sets we used. 
Predefined datasets can be downloaded (here).

## Udacity data download 

The raw data, which needs to be first prerpocessed and then turned into an hdf5 file format, such that it can be used in training,  can be downloaded here https://github.com/udacity/self-driving-car/tree/master/datasets . In our experiments we used the Challenge 2 Driving Data. 
Follow the download instructions and use the Udacity Reader tool to extract the .bag files.

## Usage of jpg2hdf5.py script 

The script has 3 stages:
 - The first step parses image ids to steering ids. The id  specifies the time when the measurment was done. However, images and steering are measured at different times. Thereby, requring nearest neighbour matching.
 - The second step creates based on the specified flag a selection of images
 - The thrid step creates a hdf5 file called train.h5.  Because of high frame rates we select only each third image to avoid to similar images and therefore reduce overfitting. 

To achive these steps a number of csv files are generated and can be reviewed. 

|  | Flags | Description 
|:---|:---|:---|
| 1 | -h | help | 
| 2 | -i path | path = path to folder in which there is a folder named center containing all images and a csv file named steering.csv containing associated angles. (udacity extraction format) | 
| 3 | -a | Set this flag in order to generate hdf5 file based on all images | 
| 4 | -c | Set this flag in order to generate hdf5 file containing only images represeting the beginning and end of curves | 
| 5 | -l | Set this flag in order to generate hdf5 file containing only images represeting straight driving | 
| 6 | -s float | Set this flag in order to generate hdf5 file containing  in which the specified float specifies the percentage of how much the straigth driving the data contains. The rest of the data is curve driving. Make sure that the percentage does not exceed 0.5. | 
| 7 | -n int | specifies number of training images used (default all images in center)| 


The -i flag is mandatory scince it specifies the path to the folder containg the images (needs to be named center) and the steering csv (needs to be named steering.csv).
In addition exactly one of the following flags has to be set (-a,-c,-l,-s)
Optional -n to set 

## Examples 

- python jpg2hdf5.py -i ./ -s 0.2
    - assumtion: script is inside a folder containing  the center/ folder with images and  steering.csv 
    - generates: train.h5 containing 20% straight driving and 80% curves and saves it to working directory
    


- python jpg2hdf5.py -i ./test_set -n 1000 -c
    - generates: train.h5 containg 100% curves and 1000/3 images.

To generate test.h5 simply use this script and rename the associated file. 





