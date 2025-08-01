/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.generator.ArchitectureSupportChecker;

public class CNNArch2GluonArchitectureSupportChecker extends ArchitectureSupportChecker {

    public CNNArch2GluonArchitectureSupportChecker() {}

    @Override
    protected boolean checkMultipleStreams(ArchitectureSymbol architecture) {
        return true;
    }

    @Override
    protected boolean checkMultipleInputs(ArchitectureSymbol architecture) {
        return true;
    }

    @Override
    protected boolean checkMultipleOutputs(ArchitectureSymbol architecture) {
        return true;
    }

    @Override
    protected boolean checkMultiDimensionalOutput(ArchitectureSymbol architecture) {
        return true;
    }

    @Override
    protected boolean checkConstants(ArchitectureSymbol architecture) {
        return true;
    }

    @Override
    protected boolean checkLayerVariables(ArchitectureSymbol architecture) {
        return true;
    }

    @Override
    protected boolean checkOutputAsInput(ArchitectureSymbol architecture) {
        return true;
    }

    @Override
    protected boolean checkUnroll(ArchitectureSymbol architecture) {
        return true;
    }

}
