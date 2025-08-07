/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.generator.GenericCNNArchCli;

public class CNNArch2MxNetCli {
    public static void main(String[] args) {
        CNNArchGenerator generator = new CNNArch2MxNet();
        GenericCNNArchCli cli = new GenericCNNArchCli(generator);
        cli.run(args);
    }
}
