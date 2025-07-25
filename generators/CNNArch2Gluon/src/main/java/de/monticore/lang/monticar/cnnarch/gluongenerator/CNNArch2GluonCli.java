/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.generator.GenericCNNArchCli;

public class CNNArch2GluonCli {
    public static void main(String[] args) {
        CNNArchGenerator generator = new CNNArch2Gluon();
        GenericCNNArchCli cli = new GenericCNNArchCli(generator);
        cli.run(args);
    }
}
