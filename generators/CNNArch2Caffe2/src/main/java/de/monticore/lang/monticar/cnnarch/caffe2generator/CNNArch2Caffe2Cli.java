/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.caffe2generator;

import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.generator.GenericCNNArchCli;

public class CNNArch2Caffe2Cli {
    public static void main(String[] args) {
        CNNArchGenerator generator = new CNNArch2Caffe2();
        GenericCNNArchCli cli = new GenericCNNArchCli(generator);
        cli.run(args);
    }
}
