package de.monticore.lang.monticar.cnnarch.pytorchgenerator;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.generator.ArchitectureSupportChecker;

public class CNNArch2PyTorchArchitectureSupportChecker extends ArchitectureSupportChecker {
    public CNNArch2PyTorchArchitectureSupportChecker() {}

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

}
