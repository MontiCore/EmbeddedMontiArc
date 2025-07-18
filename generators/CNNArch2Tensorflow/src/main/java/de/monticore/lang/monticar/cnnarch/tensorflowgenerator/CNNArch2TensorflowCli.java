/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.tensorflowgenerator;

import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.generator.GenericCNNArchCli;

public class CNNArch2TensorflowCli {
    public static void main(String[] args) {
        CNNArchGenerator generator = new CNNArch2Tensorflow();
        GenericCNNArchCli cli = new GenericCNNArchCli(generator);
        cli.run(args);
    }
}
