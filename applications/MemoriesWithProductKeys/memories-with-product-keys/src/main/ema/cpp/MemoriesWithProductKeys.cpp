/* (c) https://github.com/MontiCore/monticore */
#include "CNNTranslator.h"
#include "memorieswithproductkeys_connector.h"
#include <armadillo>
#include <string> //Check if it can be removed
#include <iostream>
#include <fstream>

std::vector<float> intSplit(string str){
    string elem;
    std::vector<float> intSeq;
    for (char const &c: str) {
        if(c == ','){
            intSeq.push_back(std::stof(elem));
            elem = "";
        }else{
            elem.append(1, c);
        }
    }
    intSeq.push_back(std::stoi(elem));
    return intSeq;
}

int main(int argc, char* argv[]) {

    if(argc < 4 || argc > 4){
        std::cout << "Missing or to many arguments: Path to 1 file with sentence samples to perform prediction onto has to be provided." << std::endl;
        exit(1);
    }

    memorieswithproductkeys_connector connector;
    connector.init();

    std::string samplesPath = argv[1];
    std::string sequencesPath = argv[2];
    std::string lengthsPath = argv[3];
    fstream samplesFile;
    fstream sequencesFile;
    fstream lengthsFile;

    samplesFile.open(samplesPath, ios::in);
    sequencesFile.open(sequencesPath, ios::in);
    lengthsFile.open(lengthsPath, ios::in);
    if (samplesFile.is_open() && sequencesFile.is_open() && lengthsFile.is_open()){
        string sample;
        string sequence;
        string length;

        while(getline(samplesFile, sample)){
            getline(sequencesFile, sequence);
            getline(lengthsFile, length);

            if(!samplesFile.eof() && (sequencesFile.eof() || lengthsFile.eof())){
                std::cout << "The input files have a differing number of rows." << std::endl;
                exit(1);
            }
        }
        if(!sequencesFile.eof() || !lengthsFile.eof()){
            std::cout << "The input files have a differing number of rows." << std::endl;
            exit(1);
        }

        samplesFile.close();
        sequencesFile.close();
        lengthsFile.close();
    }

    samplesFile.open(samplesPath, ios::in);
    sequencesFile.open(sequencesPath, ios::in);
    lengthsFile.open(lengthsPath, ios::in);
    if (samplesFile.is_open() && sequencesFile.is_open() && lengthsFile.is_open()){
        string sample;
        string sequence;
        string length;
        int classIndex;
        int sampleNum = 0;

        while(getline(samplesFile, sample)){
            getline(sequencesFile, sequence);
            getline(lengthsFile, length);
            std::vector<float> sampleSeq;
            std::vector<float> sequenceSeq;
            std::vector<float> lengthSeq;
            sampleSeq = intSplit(sample);
            sequenceSeq = intSplit(sequence);
            lengthSeq = intSplit(length);

            size_t seqLen = sampleSeq.size();

            connector.data_0 = CNNTranslator::translateToIntCol(sampleSeq, vector<size_t>{seqLen});
            connector.data_1 = CNNTranslator::translateToIntCol(sequenceSeq, vector<size_t>{seqLen});
            connector.data_2 = CNNTranslator::translateToIntCol(lengthSeq, vector<size_t>{1});

            connector.execute();

            classIndex = (int)connector.res;
            std::cout << std::endl << "Prediction for sample " << std::to_string(sampleNum) << ": " << classIndex << std::endl;
            sampleNum++;
        }

        samplesFile.close();
        sequencesFile.close();
        lengthsFile.close();
    }

    return 0;
}
