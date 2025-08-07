/* (c) https://github.com/MontiCore/monticore */
#include "CNNTranslator.h"
#include "translator_rNNsearch.h"

#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

// 28 + 1 + 1 (<sos>, <eos> token) = 30 
const int MAX_LENGTH = 28;

std::pair<std::map<std::string, int>, std::vector<std::string>> readVocabulary(const std::string& filename) {
    std::ifstream file(filename);

    if (!file) {
        throw std::runtime_error("File " + filename + " is invalid");
    }

    int i = 0;
    std::string buffer;

    std::map<std::string, int> map;
    std::vector<std::string> vector;

    while (std::getline(file, buffer)) {
        map[buffer] = i++;
        vector.push_back(buffer);
    }

    if (map.count("<pad>") == 0) {
        throw std::runtime_error("Vocab " + filename + " does not contain <pad>");
    }

    if (map.count("<unk>") == 0) {
        throw std::runtime_error("Vocab " + filename + " does not contain <unk>");
    }

    if (map.count("<s>") == 0) {
        throw std::runtime_error("Vocab " + filename + " does not contain <s>");
    }

    if (map.count("</s>") == 0) {
        throw std::runtime_error("Vocab " + filename + " does not contain </s>");
    }

    return std::make_pair(map, vector);
}

std::vector<int> sequenceToIndices(std::string sequence, const std::map<std::string, int>& vocab, bool padFront = true) {
    std::size_t pos = 0;
    std::vector<std::string> words;

    while ((pos = sequence.find(' ')) != std::string::npos) {
        words.push_back(sequence.substr(0, pos));
        sequence.erase(0, pos + 1);
    }

    if (words.size() > MAX_LENGTH) {
        throw std::runtime_error("Only sequences with " + std::to_string(MAX_LENGTH) + " words or less are supported");
    }

    std::vector<int> indices{vocab.at("<s>")};

    for (const std::string& word : words) {
        if (vocab.count(word) > 0) {
            indices.push_back(vocab.at(word));
        } else {
            indices.push_back(vocab.at("<unk>"));
        }
    }

    indices.push_back(vocab.at("</s>"));

    while (indices.size() < MAX_LENGTH + 2) {
        if (padFront) {
            indices.insert(indices.begin(), vocab.at("<pad>"));
        } else {
            indices.push_back(vocab.at("<pad>"));
        }
    }

    return indices;
}

std::string indicesToSequence(const std::vector<int>& indices, const std::vector<std::string>& vocab) {
    std::string sequence;

    for (int index : indices) {
        if (index < vocab.size()) {
            sequence += vocab.at(index) + " ";
        } else {
            throw std::runtime_error("Invalid target vocab size");
        }
    }

    return sequence;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: Translator <source vocab filename> <target vocab filename> '<source sequence>'" << std::endl;
        return 1;
    }

    try {
        std::pair<std::map<std::string, int>, std::vector<std::string>> sourceVocab = readVocabulary(argv[1]);
        std::pair<std::map<std::string, int>, std::vector<std::string>> targetVocab = readVocabulary(argv[2]);

        std::vector<int> sourceIndices = sequenceToIndices(argv[3], sourceVocab.first);
        std::vector<float> source(sourceIndices.begin(), sourceIndices.end());

        // do stuff
        translator_rNNsearch rnnsearch;
        rnnsearch.init();

        rnnsearch.source = conv_to<ivec>::from(CNNTranslator::translateToCol(source, std::vector<std::size_t>{source.size()}));

        rnnsearch.execute();

        std::vector<int> targetIndices;

        for (std::size_t i = 0; i < MAX_LENGTH + 2; ++i) {
            targetIndices.push_back(rnnsearch.target[i](0));
        }

        std::string targetSequence = indicesToSequence(targetIndices, targetVocab.second);

        std::cout << targetSequence << std::endl;
    } catch (const std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
