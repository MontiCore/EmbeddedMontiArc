/* (c) https://github.com/MontiCore/monticore */
#include <opencv2/core.hpp>
#include <opencv2/hdf.hpp>
#include <iostream>
#include <fstream>
#include "CNNPredictor_endtoend_nvidia.h"

using namespace std;
using namespace cv;

int main(int argc,char* argv[])
{
    //Parse arguments
    if(argc<3){
        cout << "Please give name of h5 file and output filename as arguments." << endl;
        return 1;
    }
    String filename = argv[1];
    String output_filename = argv[2];
    cout << "Reads from " << filename << endl;

    //Initialize variables
    Ptr<hdf::HDF5> h5io =hdf::open( filename );
    std::vector<int> dims = h5io->dsgetsize("data", hdf::HDF5::H5_GETDIMS);
    int num_pictures=dims.at(0);
    vector<float> prediction;
    int picture_offset=dims.at(1)*dims.at(2)*dims.at(3);
    vector<float> data(picture_offset);
    CNNPredictor_endtoend_nvidia_0 predictor;

    //Read data from h5 file
    Mat M;
    h5io->dsread(M, "data");

    //Open output file
    ofstream output_file;
    output_file.open (output_filename);

    for(int n=0;n<num_pictures;n++){
        //Convert picture to flat vector
        for(int i=0;i<picture_offset;i++)
            data[i]=M.at<float>(n*picture_offset+i);
        //Prediction
        predictor.predict(data,prediction);
        cout << prediction.at(0) <<endl;
        if(n<num_pictures-1)
            output_file << prediction.at(0) << ",";
    }

    //Close everything
    h5io->close();
    output_file << prediction.at(0);
    output_file.close();

    return 0;
}
