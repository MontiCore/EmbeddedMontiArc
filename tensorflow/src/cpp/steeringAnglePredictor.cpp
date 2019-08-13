#include <opencv2/core.hpp>
#include <opencv2/hdf.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include "CNNPredictor_endtoend_nvidia.h"


using namespace std;
using namespace cv;


inline const char * const BoolToString(bool b);

int main(int argc,char* argv[])
{
    //Get information
    if(argc<2){
        cout << "Please give name of h5 file as an argument." << endl;
        return 1;
    }
    
    String filename = argv[1];
    
    //Debug information
    cout << "Reads from "+filename << endl;
    Ptr<hdf::HDF5> h5io =hdf::open( filename );

    cout << "Data exists: "<< h5io->hlexists("data") << endl;   
    
    
    //Dimensions for debugging
    std::vector<int> dims = h5io->dsgetsize("data", hdf::HDF5::H5_GETDIMS);
    int num_pictures=dims.at(0);
    int width=dims.at(3);
    int height=dims.at(2);
    int num_channels=dims.at(1);
    
    cout << "Dimension of data is: "; 
    cout << "N" << num_pictures;
    cout << "C" << num_channels;
    cout << "H" << height;
    cout << "W" << width<< endl;
    
    
    //Read data from h5 file
    Mat M;
    h5io->dsread(M, "data");
    
    //Open file
    ofstream myfile;
    myfile.open ("example.csv");
    
    //Convert to vector and call predictor
    vector<float> prediction[num_pictures];
    vector<float> data(num_channels*height*width);
    CNNPredictor_endtoend_nvidia_0 predictor;
    
    for(int n=0;n<num_pictures;n++){    
        for(int i=0;i<width*height*num_channels;i++){
            data[i]=M.at<float>(n*height*width*num_channels+i);
        }
        
        //Prediction
        prediction[n]=vector<float> (1,42);
        predictor.predict(data,prediction[n]);
        cout << prediction[n].at(0) <<endl;
        if(n<num_pictures-1){
            myfile << prediction[n].at(0) << ",";
        }
    }    
    //Close everything
    h5io->close();
    myfile << prediction[num_pictures-1].at(0);
    myfile.close();
    
    return 0;
}
