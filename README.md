# BERTSentimentAnalyzer

Sentiment Analysis is the task of idenfifying opinions, sentiments and subjectivity from a sentence or text. In this project we use a BERT 
model for the sentiment analysis task.
<div align="center">
    <img src="/resources/bert.png" width="500"/>
</div>

To *train the neural network* in this project, first the package with the pre-trained model must be downloaded locally. To do this, the command 
```bash
mvn dependency:resolve -s settings.xml
```
can be run.  
After that, the neural network can be trained using the [emadl-maven-plugin](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/emadl-maven-plugin):
```bash
mvn emadl:train -s settings.xml
```

## Training the neural network with the BERT Base model
To be able to get a quick responds from the CI, it is currently configured to use a small pre-trained model for training. To use the BERT Base model, which was originally used, the following modifcations must be made:
1. *Update Dependency*  
    In the [pom.xml file](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/bertsentimentanalyzer/-/blob/master/pom.xml), use the BERT Base model as dependency:  
    Instead of 
    ```xml
    <dependency>
        <groupId>de.monticore.lang.monticar.pretrained</groupId>
        <artifactId>simple-pretrained</artifactId>
        <version>1</version>
        <classifier>pretrained</classifier>
    </dependency>
    ```
    define
    ```xml
    <dependency>
        <groupId>de.monticore.lang.monticar.pretrained</groupId>
        <artifactId>bert-base</artifactId>
        <version>1</version>
        <classifier>pretrained</classifier>
    </dependency>
    ```
    as a dependency.  
    
2. *Modify LayerArtifactParameter Tag*  
    In the [tagging file](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/bertsentimentanalyzer/-/blob/master/src/main/emadl/sentimentanalyzer/sentimentanalyzer.tag), change the artifact of the LayerArtifactParameter tag from `de.monticore.lang.monticar.pretrained:`**`simple-pretrained`**`:1` to `de.monticore.lang.monticar.pretrained:`**`bert-base`**`:1`.

After that, the following commands can be run to train the neural network:
```bash
mvn dependency:resolve -s settings.xml  
mvn emadl:train -s settings.xml
```
