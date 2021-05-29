/* (c) https://github.com/MontiCore/monticore */
#include <armadillo>
#include "flowNetS_network.h"
#include "CNNTranslator.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <map>
#include <math.h>
#include <stdlib.h>


#define FLOW_THRESH 1e7

vector<vector<int>> makeMiddelburryColorWheel(){
    
    int RY = 15;
    int YG = 6;
    int GC = 4;
    int CB = 11;
    int BM = 13;
    int MR = 6;

    int ncols = RY + YG + GC + CB + BM + MR;

    vector<vector<int>> colorwheel(ncols, vector<int>(3, 0));

    int col = 0;

    //RY
    for(int i=0; i < RY; i++){
        colorwheel[i][0] = 255;
        colorwheel[i][1] = 255*i / RY;
    }
    col += RY;

    //YG
    for(int i=col; i < col+YG; i++){
        colorwheel[i][0] = 255 - (255*(i-col) / YG);
        colorwheel[i][1] = 255;
    }
    col += YG;

    //GC
    for(int i=col; i < col+GC; i++){
        colorwheel[i][1] = 255;
        colorwheel[i][2] = (255*(i-col) / GC);
    }
    col += GC;

    //CB
    for(int i=col; i < col+CB; i++){
        colorwheel[i][1] = 255 - (255*(i-col) / CB);
        colorwheel[i][2] = 255;
    }
    col += CB;

    //BM
    for(int i=col; i < col+BM; i++){
        colorwheel[i][2] = 255;
        colorwheel[i][0] = (255*(i-col) / BM);
    }
    col += BM;

    //MR
    for(int i=col; i < col+MR; i++){
        colorwheel[i][2] = 255 - (255*(i-col) / MR);
        colorwheel[i][0] = 255;
    }
    col += MR;

    return colorwheel;
}

cv::Mat computeFlowImg(vector<vector<double>> u, vector<vector<double>> v){

    int h = u.size();
    int w = u[0].size();
    
    cv::Mat img(h, w, CV_8UC3, cv::Scalar(0,0,0));

    vector<vector<int>> colorwheel = makeMiddelburryColorWheel();
    int ncols = colorwheel.size();

    vector<vector<float>> rad(h, vector<float>(w, 0));
    float fk;
    vector<vector<int>> k0(h, vector<int>(w, 0));
    vector<vector<int>> k1(h, vector<int>(w, 0));
    vector<vector<bool>> kNan(h, vector<bool>(w, false));
    vector<vector<float>> f(h, vector<float>(w, 0));
    
    for(int i=0; i < h; i++){
        for(int j=0; j < w; j++){
            
            if(isnan(u[i][j]) || isnan(v[i][j])){
                u[i][j] = 0;
                v[i][j] = 0;
                kNan[i][j] = true;
            }
            
            rad[i][j] = sqrt(pow(u[i][j], 2) + pow(v[i][j], 2));
            fk = (atan2(-v[i][j], -u[i][j]) / M_PI  + 1) / 2 * (ncols - 1) + 1;    
           
            k0[i][j] = int(fk);    
            k1[i][j] = k0[i][j] + 1;
            
            if(k1[i][j] == ncols+1){
                k1[i][j] = 1;
            }
            
            f[i][j] = fk - k0[i][j];
        }
    }
    
    
    float col0;
    float col1;
    float col;
    for(int i=0; i < h; i++){
        for(int j=0; j < w; j++){
            for(uint k=0; k < colorwheel[0].size(); k++){

                col0 = colorwheel[k0[i][j]-1][k] / 255.0; //255.0 so that 255 is not a int
                col1 = colorwheel[k1[i][j]-1][k] / 255.0;
                col = (1-f[i][j]) * col0 + f[i][j] * col1;
               
                if(rad[i][j] <= 1){
                    
                    col = 1-rad[i][j] * (1-col);
                }else{
                    
                    col *= 0.75;
                }
                    
                if(kNan[i][j]){
                        
                    img.at<cv::Vec3b>(i, j)[k] = uchar(0);
                }else{
                    
                    img.at<cv::Vec3b>(i, j)[k] = uchar(std::floor(255 * col)); 
                }
            } 
        }
    }
    
    return img;
}
        
cv::Mat flowFieldToImage(cube flowField){
 
    double maxu = -999;
    double maxv = -999;
    double minu = 999;
    double minv = 999;
    int h = flowField.n_rows;
    int w = flowField.n_cols;
    
    vector<vector<double>> u(h, vector<double>(w, 0));
    vector<vector<double>> v(h, vector<double>(w, 0));
    vector<vector<double>> rad(h, vector<double>(w, 0));
    double maxrad = -1;  
    
    
    for(int i=0; i < h; i++){
        for(int j=0; j < w; j++){
            
            u[i][j] = flowField(i,j,0);
            v[i][j] = flowField(i,j,1); 
            
            if((abs(u[i][j]) > FLOW_THRESH) || (abs(v[i][j]) > FLOW_THRESH)){
                u[i][j] = 0;
                v[i][j] = 0;
            }
            
            if(u[i][j] > maxu){
                maxu = u[i][j];
            }
            
            if(u[i][j] < minu){
                minu = u[i][j];
            }
            
            if(v[i][j] > maxv){
                maxv = v[i][j];
            }
            if(v[i][j] < minv){
                minv = v[i][j];
            }
            
            rad[i][j] = sqrt(pow(u[i][j], 2) + pow(v[i][j], 2)); 
            if(rad[i][j] > maxrad){
                maxrad = rad[i][j];
            }
        }
    }
    
    
    for(int i=0; i < h; i++){
        for(int j=0; j < w; j++){
            
            u[i][j] = u[i][j] / (maxrad + std::numeric_limits<float>::epsilon());
            v[i][j] = v[i][j] / (maxrad + std::numeric_limits<float>::epsilon());
        }
    }
    

    cv::Mat img = computeFlowImg(u, v);
    
    for(int i=0; i < h; i++){
        for(int j=0; j < w; j++){
            if((abs(u[i][j]) > FLOW_THRESH) || (abs(v[i][j]) > FLOW_THRESH)){
                img.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
            }    
        }
    }
    
    return img;
}
             

int main(int argc, char* argv[]) {
    
    if(argc < 2){ //Note: argc=1 if no arguments are provided
        std::cout << "Missing argument: Path to 2 images must be provided " << std::endl;
        exit(1);
    }
    

    flowNetS_network network;
    network.init();

    
    std::string filePath = argv[1];

    if (!std::ifstream(filePath).good()) {
        std::cerr << "Image 1 loading failure, 1st test image '" << filePath << "' does not exist." << std::endl;
        exit(1);
    }

    cv::Mat img1 = cv::imread(filePath);
    cv::cvtColor(img1, img1, CV_BGR2RGB);
    
    std::cout << "Image 1 size: " << img1.size() << std::endl;

    size_t channels = 3;
    size_t height = img1.rows;
    size_t width = img1.cols;
    vector<float> data1(channels*height*width);

    for(size_t j=0; j<height; j++){
        for(size_t k=0; k<width; k++){
            cv::Vec3b intensity = img1.at<cv::Vec3b>(j, k);
            for(size_t i=0; i<channels; i++){
                data1[i*height*width + j*width + k] = (float) intensity[i];
            }
        }
    }
    
    
    filePath = argv[2];

    if (!std::ifstream(filePath).good()) {
        std::cerr << "Image 2 loading failure, 2nd test image '" << filePath << "' does not exist." << std::endl;
        exit(1);
    }

    cv::Mat img2 = cv::imread(filePath);
    cv::cvtColor(img2, img2, CV_BGR2RGB);
        
    std::cout << "Image 2 size: " << img2.size() << std::endl;

    channels = 3;
    height = img2.rows;
    width = img2.cols;
    vector<float> data2(channels*height*width);

    for(size_t j=0; j<height; j++){
        for(size_t k=0; k<width; k++){
            cv::Vec3b intensity = img2.at<cv::Vec3b>(j, k);
            for(size_t i=0; i<channels; i++){
                data2[i*height*width + j*width + k] = (float)intensity[i];
            }
        }
    }

    
    network.data_0 = conv_to< cube >::from( CNNTranslator::translateToCube(data1, vector<size_t> {channels,height,width}) );
    network.data_1 = conv_to< cube >::from( CNNTranslator::translateToCube(data2, vector<size_t> {channels,height,width}) );
     
    network.execute();

    
    cube flowFieldCube = network.target_0;
    
    cv::Mat flowFieldImg;
    cv::resize(flowFieldToImage(flowFieldCube), flowFieldImg, cv::Size(512, 384));
    
    cv::cvtColor(img1, img1, CV_BGR2RGB);
    cv::cvtColor(img2, img2, CV_BGR2RGB);
        
    cv::imshow("Image 1", img1);
    cv::imshow("Image 2", img2);
    cv::imshow("Predicted Flow Field", flowFieldImg);
    cv::waitKey();

    return 0;
}
