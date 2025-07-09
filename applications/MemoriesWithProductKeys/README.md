<!-- (c) https://github.com/MontiCore/monticore -->
# MemoriesWithProductKeys

## Prerequisites
1. Ubuntu Linux 16.04 LTS
3. Deep Learning Framework **Gluon**. Note that you need mxnet version 1.7.0 for the C++ part and at least versions 1.7.0b20200417 (CPU) or 1.7.0b20200416 (GPU) for python. These version further more only work with python 3 and not python 2. 
The docker script given [here](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMADL2CPP/-/tree/master/src/test/resources/docker) for mxnet 1.7.0 installs the right versions for CPU usage.

## Introduction
This application implements the LargeMemory layer presented [here](https://papers.nips.cc/paper/9061-large-memory-layers-with-product-keys.pdf). 
We do not implement the model presented in the paper with this layer, but rather a simpler one similar to the [EpisodicMemoryInLifelongLanguageLearning]https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/lifelonglanguagelearning/-/tree/master model.
For this it uses thre new layers we added to the EMADL framework: The LargeMemory layer, the DotProductSelfAttention layer and the LoadNetwork layer that allows to load a pretrained network as a component, in our case this is the BERT-small model bert_12_768_12 trained on the book_corpus_wiki_en_uncased dataset provided by [GluonNLP](https://nlp.gluon.ai/model_zoo/bert/index.html). 
Note that we do not include the pretrained BERT model, the training data nor a trained checkpoint of the complete model in this repository, due to their big size. Though we provide the folder structure in which to place the files and two scripts for obtaining the pretrained BERT model and for processing the training data [here](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/memorieswithproductkeys/-/blob/master/additional_material). The script for obtaining the pretrained BERT model differs from the one in the [MemoriesWithProductKeys](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/lifelonglanguagelearning) application, although having the same name and being similar. The resulting network is slightly different. 
Further be aware that for the training data you need to first execute the steps detailed in steps 2 and 3 in [here](https://github.com/h3lio5/episodic-lifelong-learning). Here it is important to check the preprocessing.py file to generate the correct ordering for the test data, which has to be manually set and set to ordering 1 per default. 

## Execution
There are two ways to train this application, the first is the traditional one using a bash script givin in the [gluon](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/lifelonglanguagelearning/-/blob/master/gluon) folder. You just have to run:
```
bash build.sh
```
in this folder, once you provided the pretrained BERT and the training data. Note that the model needs a long time to train and it is advisable to use the cluster. This application utilizes all available GPU's of a machine, a functionality also implemented as standard by us into the EMADL framework. This is of advantage when using the cluster, since it provides two GPU's. 
After training one can perform prediciton by executing:
```
build/src/cpp/MemoriesWithProductKeys <samples_path> <sequence_type_path> <length_path>
```
Where samples_path one or more sentence sequences of tokens, sequence_type_path contains in our classification sequences of 0 for every sample and length_path contains the true lengths of each sequences, hence up to which position of a token sequence include tokens into the calculations. These inputs follow the inputs the pretrained BERT presented above expects. 
Note that this way to train uses the parameter we used for evaluating the model and that are also the ones used in the original paper. 

The second way to train the network is to execute the streamtest in the [memories-with-product-keys](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/memorieswithproductkeys/-/blob/main/memories-with-product-keys) folder. This is done by executing:
```
mvn streamtest:streamtest-build -s settings.xml
```
Note that this way only uses a simpler pretrained net and a smaller set of training dat with a sequence length of only 5 instead of 128.
This can be changed by changing the path in the memorieswithproductkeys.tag file in [this](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/memorieswithproductkeys/-/blob/master/memories-with-product-keys/src/main/ema/emadl/models/memoriesWithProductKeys) folder. Further one would have to adjust the Connector.emadl and Network.emadl files in the same folder to expect sequences of length 128. 
This way is also used for the CI pipeline of this application.

## Results
This model did not achieve good results and only was able to reach 15% accuracy.
