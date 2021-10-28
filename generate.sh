rm -rf target
java -jar embedded-montiarc-emadl-generator-0.5.3-SNAPSHOT-jar-with-dependencies.jar -m src/emadl/models -r gCN.Network -o target -cfp src -p /usr/bin/python3.6 -b GLUON