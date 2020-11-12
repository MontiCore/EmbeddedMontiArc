<!-- (c) https://github.com/MontiCore/monticore -->
# EpisodicMemoryInLifelongLanguageLearning

## Prerequisites
1. Ubuntu Linux 16.04 LTS
3. Deep Learning Framework **Gluon**.

## Introduction
This application implements the EpisodicMemoryInLifelongLanguageLearning classification model presented [here](https://papers.nips.cc/paper/9471-episodic-memory-in-lifelong-language-learning.pdf). 
For this it use two new layers we added to the EMADL framework: The EpisodicMemory layer that provides the replay and local adaptation functionality and the LoadNetwork layer that allows to load a pretrained network as a component, in our case this is the BERT-small model bert_12_768_12 trained on the book_corpus_wiki_en_uncased dataset provided by [GluonNLP](https://nlp.gluon.ai/model_zoo/bert/index.html). 
Note that we do not include the pretrained BERT model, the training data nor a trained checkpoint of the complete model in this repository, due to their big size. Though we provide the folder structure in which to place the files and two scripts for obtaining the pretrained BERT model and for processing the training data [here](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/lifelonglanguagelearning/-/blob/master/additional_material). The script for obtaining the pretrained BERT model differs from the one in the [MemoriesWithProductKeys](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/memorieswithproductkeys) application, although having the same name and being similar. The resulting network is slightly different. Further be aware that for the training data you need to first execute the steps detailed in steps 2 and 3 in [this](https://github.com/h3lio5/episodic-lifelong-learning) reference implementation. Here it is important to check the preprocessing.py file to generate the correct ordering for the test data, which has to be manually set and set to ordering 1 per default. 

## Execution
There are two ways to train this application, the first is the traditional one using a bash script givin in the [gluon](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/lifelonglanguagelearning/-/blob/master/gluon) folder. You just have to run:
```
bash build.sh
```
in this folder, once you provided the pretrained BERT and the training data. Note that the model needs a long time to train and it is advisable to use the cluster. This application utilizes all available GPU's of a machine, a functionality also implemented as standard by us into the EMADL framework. This is of advantage when using the cluster, since it provides two GPU's. 
After training one can perform prediction by executing:
```
build/src/cpp/ LifelongLanguageLearning <samples_path> <sequence_type_path> <length_path>
```
Where samples_path one or more sentence sequences of tokens, sequence_type_path contains in our classification sequences of 0 for every sample and length_path contains the true lengths of each sequences, hence up to which position of a token sequence include tokens into the calculations. These inputs follow the inputs the pretrained BERT presented above expects. 
Note that this way to train uses the parameters we used for evaluating the model, though for the CI we replaced the values of localAdaptationK, localAdaptationGradientSteps by 2 and 4 respectively for efficiency reasons, instead of the 32 and 30 used in actual training. These are also the ones used in the original paper. 
Furthermore the saved memory of the pretrained model (model_0_newest_episodic_memory_sub_net_1-0000) is only a subset, of the one originally saved during training, due to its size. This is sufficient for the CI. 

The second way to train the network is to execute the streamtest in the [lifelong-language-learning](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/lifelonglanguagelearning/-/blob/main/lifelong-language-learning) folder. This is done by executing:
```
mvn streamtest:streamtest-build -s settings.xml
```
Note that this way only uses a simpler pretrained net and a smaller set of training data with a sequence length of only 5 instead of 128.
This can be changed by changing the path in the lieflongLanguageLearning.tag file in [this](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/lifelonglanguagelearning/-/blob/master/lifelong-language-learning/src/main/ema/emadl/models/lifelongLanguageLearning) folder. Further one would have to adjust the Connector.emadl and Network.emadl files in the same folder to expect sequences of length 128. 
This way is also used for the CI pipeline of this application.

## Results
We achieved comparable results to the ones in the original paper, by achieving an accuracy of just above 70% after the second batch of training. The training log can also be found [here](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/lifelonglanguagelearning/-/blob/master/additional_material).
