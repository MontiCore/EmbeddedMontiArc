package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.ArchitectureSupportChecker;

public class CNNArch2GluonArchitectureSupportChecker extends ArchitectureSupportChecker {

    public CNNArch2GluonArchitectureSupportChecker() {}

    protected boolean checkMultipleStreams(ArchitectureSymbol architecture) {
        return true;
    }

    protected boolean checkMultipleInputs(ArchitectureSymbol architecture) {
        return true;
    }

    protected boolean checkMultipleOutputs(ArchitectureSymbol architecture) {
        return true;
    }

}
