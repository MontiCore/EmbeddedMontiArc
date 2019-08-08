#include <opencv2/core.hpp>
#include <opencv2/hdf.hpp>
#include <iostream>
#include "CNNPredictor_endtoend_nvidia.h"


using namespace std;
using namespace cv;


inline const char * const BoolToString(bool b);

int main(int argc,char* argv[])
{
    if(argc<2){
        cout << "Please give name of h5 file as an argument." << endl;
        return 1;
    }
    
    String filename = argv[1];
    
    cout << "Reads from "+filename << endl;
    Ptr<hdf::HDF5> h5io =hdf::open( filename );

    cout << "Data exists: "<< BoolToString(h5io->hlexists("data")) << endl;
    
    cout << "Softmax_label exists: " << BoolToString(h5io->hlexists("softmax_label")) << endl;

    
    std::vector<int> dims;
    dims = h5io->dsgetsize("data", hdf::HDF5::H5_GETDIMS);    
    cout << "Dimension of data is: ";
    cout << dims.at(0) << ",";
    cout << dims.at(1) << ",";
    cout << dims.at(2) << ",";
    cout << dims.at(3) << endl;

    Mat M;
    h5io->dsread(M, "data");
   
    int num_pictures=dims.at(0);
    int width=dims.at(3);
    int height=dims.at(2);
    int num_channels=dims.at(1);
    vector<float> prediction[num_pictures];
    
    for(int n=0;n<num_pictures;n++){
        vector<float> data(num_channels*height*width);
        prediction[n]=vector<float> (1,42);
        Mat img(height,width,CV_32F, M.ptr<int>(n));
        for(int i=0;i<height;i++){
            for(int j=0;j<width;j++){
                for(int k=0;k<num_channels;k++){
                    data[k*height*width + i*height + j] = (float) img.at<int>(i,j);
                }
            }
        }
        vector<float> *prediction_address = &(prediction[n]);
        vector<float> *data_address = &(data);
        
        
    }
    
    
    
    
    
    h5io->close();
    return 0;
}

inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}
