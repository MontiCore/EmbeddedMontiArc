package de.monticore.lang.monticar.cnnarch.pytorchgenerator;

import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.generator.GenericCNNArchCli;

public class CNNArch2PyTorchCli {

    public static void main(String[] args) {
        CNNArchGenerator generator = new CNNArch2PyTorch();
        GenericCNNArchCli cli = new GenericCNNArchCli(generator);
        cli.run(args);
    }
}
