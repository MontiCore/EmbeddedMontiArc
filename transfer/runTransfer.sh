#!/bin/bash

rm -rf target
rm -rf model

# Run the composed model trained on the MNIST dataset
java -jar embedded-montiarc-emadl-generator-0.5.16-SNAPSHOT-jar-with-dependencies.jar -m src/main/emadl/ -r digits.DigitsComposed -o target/digitsComposed -b GLUON -c n -f y -ad -dn "DigitsComposed"
if [ $? -ne 0 ]; then
    echo "Execution of EMADL-Generator for the digit base model failed."
    exit 1
fi

PARAMSFILE=model/digits.DigitsComposed.DigitsInputLayer_decomposed/model_DigitsInputLayer_decomposed-0000.params
SYMBOLFILE=model/digits.DigitsComposed.DigitsInputLayer_decomposed/model_DigitsInputLayer_decomposed-old-symbol.json

if [ ! -f "$PARAMSFILE" ] || [ ! -f "$SYMBOLFILE" ]; then
    echo "Required model files do not exist:"
    [ ! -f "$PARAMSFILE" ] && echo "$PARAMSFILE not found."
    [ ! -f "$SYMBOLFILE" ] && echo "$SYMBOLFILE not found."
    exit 1
fi

# Run the "partially" pretrained model trained on the EMNIST dataset
java -jar embedded-montiarc-emadl-generator-0.5.16-SNAPSHOT-jar-with-dependencies.jar -m src/main/emadl/ -r transfer.Transfer -o target/transferMNIST2EMNIST -b GLUON -f y
if [ $? -ne 0 ]; then
    echo "Execution of EMADL-Generator for the letter transfer model failed."
    exit 1
fi
