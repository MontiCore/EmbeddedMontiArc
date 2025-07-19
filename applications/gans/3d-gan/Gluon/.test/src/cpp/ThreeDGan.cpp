/* (c) https://github.com/MontiCore/monticore */
#include "CNNTranslator.h"
#include "threedgan_connector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz/viz3d.hpp>

#include <armadillo>
#include <string> //Check if it can be removed
#include <iostream>
#include <stdlib.h>  
#include <map>
#include <random>
#include <utility>
#include <fstream>
#include <boost/format.hpp>

std::pair<arma::mat,arma::umat> voxel2mesh(arma::cube voxels){
	float threshold = 0.9;
	arma::umat cube_verts{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {0, 1, 1}, {1, 0, 0}, {1, 0, 1}, {1, 1, 0},
                  {1, 1, 1}};
	arma::umat cube_faces{{1, 2, 3}, {2, 4, 3}, {3, 4, 7}, {4, 8, 7}, {1, 3, 7}, {1, 7, 5}, {1, 6, 2},
                  {1, 5, 6}, {7,8,6}, {7,6,5}, {2,8,4}, {2,6,8}};
	int l = voxels.n_rows;
	int m = voxels.n_cols;
	int n = voxels.n_slices;

	double scale = 0.01;
	double cube_dist_scale = 1.1;
	int curr_vert = 0;
	arma::mat verts(0,3);
	arma::umat faces(0,3);

	//Positions of voxels
	arma::uvec positions = arma::find(voxels > threshold);
	arma::uvec offpositions = arma::find(voxels <= threshold);
	arma::vec voxelsVec = arma::vec(voxels.n_elem);

	arma::umat pos = arma::ind2sub(size(voxels), positions); //Make a subscriptable index notation
	arma::umat offpos = arma::ind2sub(size(voxels), offpositions);

	arma::icube iVoxels(voxels.n_rows, voxels.n_cols, voxels.n_slices);

	iVoxels.elem(positions) = arma::ones<ivec>(positions.n_elem);
	iVoxels.elem(offpositions) = arma::zeros<ivec>(offpositions.n_elem);

	int result;
	int insertPos = 0;
	int insertPos2 = 0;
	int ist;
	int jst;
	int kst;
	for (arma::uword r=0; r < pos.n_cols; ++r){
		int i = pos(2,r);
		int j = pos(1,r);
		int k = pos(0,r);
		result = 0;
		for (int is = i-1; is < i+3; ++is){
			if (is < 0){
				ist = -1;
			}
			else{
				ist = is;
			}
			if (is >= voxels.n_slices && is != -1){
				continue;
			}
			for (int js = j-1; js < j+3; ++js){
				if (js < 0){
					jst = -1;
				}
				else{
					jst = js;
				}
				if (js >= voxels.n_cols && js != -1){
					continue;
				}
				for (int ks = k-1; ks < k+3; ++ks){
					if (ks < 0){
						kst = -1;
					}
					else {
						kst = ks;
					}
					if (ks >= voxels.n_rows && ks != -1){
						continue;
					}
					if (kst == -1 || jst == -1 || ist == -1){
						result += 0;
					}
					else {
						result += iVoxels(kst,jst,ist);
					}
					
				}
			}
		}
		if (result < 27){
			arma::mat toInsertM(0,3);
			arma::rowvec toInsert {k,i,j}; //OBJ FILE FORMAT IS X Y Z = COL ROW SLICE
			toInsert *= cube_dist_scale;
			for (int f = 0; f<8; f++){
				toInsertM.insert_rows(f, toInsert);
			}
			for (int f = 0; f<8; f++){
				toInsertM(f,0) += cube_verts(f,0);
				toInsertM(f,1) += cube_verts(f,1);
				toInsertM(f,2) += cube_verts(f,2);
			}
			toInsertM *= scale;
			verts.insert_rows(insertPos, toInsertM);
			arma::umat insertFaces = cube_faces + curr_vert;
			faces.insert_rows(insertPos2, insertFaces);
			curr_vert += 8;
			insertPos+= 8;
			insertPos2 += 12;
		}
	}
	std::pair<arma::mat,arma::umat> toreturn {verts, faces};
	return toreturn;
}

void write_obj(std::string filename, arma::mat verts, arma::umat faces){
	std::ofstream objFile;
	objFile.open(filename);
	objFile << boost::format("g\n# %1% vertex\n") % verts.n_rows;
	for (int i = 0; i < verts.n_rows; i++){
		objFile << boost::format("v %1% %2% %3%\n") % verts(i,0) % verts(i,1) % verts(i,2);
		//std::cout << verts(i,0);
	}
	objFile << boost::format("# %1% faces\n") % faces.n_rows;
	for (int i = 0; i < faces.n_rows; i++){
		objFile << boost::format("f %1% %2% %3%\n") % faces(i,0) % faces(i,1) % faces(i,2);
	}
}

int main(int argc, char* argv[]) {
    
	std::cout << argc << std::endl;

    threedgan_connector connector;
    connector.init();

	int genSeed = std::stoi(argv[1]);
	srand(genSeed);
	std::default_random_engine generator;
	generator.seed(genSeed);
	std::normal_distribution<double> distribution(0.0,1.0);

    vector<float> data(200);

	for(size_t i=0; i < 200; i++){
		data[i] = distribution(generator);
	}

	connector.noise = conv_to< arma::Cube<double> >::from( CNNTranslator::translateToCube(data, 
				vector<size_t> {200, 1, 1}) );

    connector.execute();

	arma::cube img_result_cube = connector.aerial;

	std::pair<arma::mat,arma::umat> results = voxel2mesh(img_result_cube);
	arma::mat verts = std::get<0>(results);
	arma::umat faces = std::get<1>(results);

	write_obj("newObject.obj", verts, faces);

    return 0;
}

void print_mat(arma::cube my_matrix) {
    std::cout.precision(4);
    uint cols = my_matrix.n_cols;
    uint rows = my_matrix.n_rows;
	uint slices = my_matrix.n_slices;
    
    std::cout << "--------\n";
    for(uint rX = 0; rX < rows; rX++) {
        std::cout << " " << rX << ": ";
        for(uint cX = 0; cX < cols; cX++) {
            std::cout << " " << cX << ": ";
			for(uint sX = 0; sX < slices; sX++) {
            	std::cout << std::fixed << my_matrix(rX, cX, sX) << " ";
        	}
        }
		
        std::cout << "\n";
    }
    std::cout << "--------\n";
}


