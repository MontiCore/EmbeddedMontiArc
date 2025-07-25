/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.tensorflowgenerator;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.generator.ArchitectureSupportChecker;

public class CNNArch2TensorflowArchitectureSupportChecker extends ArchitectureSupportChecker {

    public CNNArch2TensorflowArchitectureSupportChecker() {}
	
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

}
