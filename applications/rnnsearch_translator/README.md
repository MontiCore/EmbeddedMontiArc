<!-- (c) https://github.com/MontiCore/monticore -->
# RNNsearch Translator
This project uses the RNNsearch architecture by [Bahdanau et al.](https://arxiv.org/abs/1409.0473) to translate a sentence from English to Vietnamese using the IWSLT'15 dataset for training and the tst2013 dataset for testing preprocessed by [The Stanford Natural Language Processing Group](https://nlp.stanford.edu/projects/nmt/).

## Prerequisites
1. [Ubuntu 16.04 LTS](http://releases.ubuntu.com/16.04/)
2. [Apache MXNet](https://mxnet.apache.org/get_started/ubuntu_setup) 1.4 or later
3. [Armadillo](http://arma.sourceforge.net/download.html) 6.600 or later

## Instructions
1. Run `./create_dataset.sh` to download the complete dataset and prepare it for usage. The HDF5 files within the repository only contain a small subset of the dataset used for testing the application.
2. Update the settings in `src/emadl/translator/RNNsearch.cnnt` if wanted.
3. Run `./build.sh` to generate, train and compile the model.
4. Run `build/src/cpp/Translator resources/vocabularies/vocab.en resources/vocabularies/vocab.en '<source sequence>'` to translate a sentence from English to Vietnamese.
