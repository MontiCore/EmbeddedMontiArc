/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ConstantSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.IODeclarationSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.SerialCompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.VariableDeclarationSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public abstract class ArchitectureSupportChecker {

    // Overload functions returning always true to enable the features
    protected boolean checkMultipleStreams(ArchitectureSymbol architecture) {
        if (architecture.getNetworkInstructions().size() != 1) {
            Log.error("This cnn architecture has multiple instructions, " +
                      "which is currently not supported by the code generator. "
                      , architecture.getSourcePosition());

            return false;
        }

        return true;
    }
	
    protected boolean checkMultipleInputs(ArchitectureSymbol architecture) {
        if (architecture.getInputs().size() > 1) {
            Log.error("This cnn architecture has multiple inputs, " +
                      "which is currently not supported by the code generator. "
                      , architecture.getSourcePosition());

            return false;
        }
        
        return true;
    }

    protected boolean checkMultipleOutputs(ArchitectureSymbol architecture) {
        if (architecture.getOutputs().size() > 1) {
            Log.error("This cnn architecture has multiple outputs, " +
                      "which is currently not supported by the code generator. "
                      , architecture.getSourcePosition());

            return false;
        }

        return true;
    }

    protected boolean checkMultiDimensionalOutput(ArchitectureSymbol architecture) {
        IODeclarationSymbol ioDeclaration = (IODeclarationSymbol) architecture.getOutputs().get(0).getDeclaration();

        if (ioDeclaration.getType().getWidth() != 1 || ioDeclaration.getType().getHeight() != 1) {
            Log.error("This cnn architecture has a multi-dimensional output, " +
                      "which is currently not supported by the code generator."
                      , architecture.getSourcePosition());

            return false;
        }

        return true;
    }

    private boolean hasConstant(ArchitectureElementSymbol element) {
        ArchitectureElementSymbol resolvedElement = (ArchitectureElementSymbol) element.getResolvedThis().get();

        if (resolvedElement instanceof CompositeElementSymbol) {
            List<ArchitectureElementSymbol> constructedElements = ((CompositeElementSymbol) resolvedElement).getElements();

            for (ArchitectureElementSymbol constructedElement : constructedElements) {
                if (hasConstant(constructedElement)) {
                    return true;
                }
            }
        }
        else if (resolvedElement instanceof ConstantSymbol) {
            return true;
        }

        return false;
    }

    protected boolean checkConstants(ArchitectureSymbol architecture) {
        for (NetworkInstructionSymbol networkInstruction : architecture.getNetworkInstructions()) {
            for (ArchitectureElementSymbol element : networkInstruction.getBody().getElements()) {
                if (hasConstant(element)) {
                    Log.error("This cnn architecture has a constant, which is currently not supported by the code generator."
                            , architecture.getSourcePosition());
                    return false;
                }
            }
        }

        return true;
    }

    protected boolean checkLayerVariables(ArchitectureSymbol architecture) {
        if (!architecture.getLayerVariableDeclarations().isEmpty()) {
            Log.error("This cnn architecture uses layer variables, which are currently not supported by the code generator."
                    , architecture.getSourcePosition());
            return false;
        }

        return true;
    }

    protected boolean checkOutputAsInput(ArchitectureSymbol architecture) {
        for (NetworkInstructionSymbol networkInstruction : architecture.getNetworkInstructions()) {
            for (ArchitectureElementSymbol element : networkInstruction.getBody().getFirstAtomicElements()) {
                if (element.isOutput()) {
                    Log.error("This cnn architecture uses an output as an input, which is currently not supported by the code generator."
                            , architecture.getSourcePosition());
                    return false;
                }
            }
        }

        return true;
    }

    protected boolean checkUnroll(ArchitectureSymbol architecture) {
        for (NetworkInstructionSymbol networkInstruction : architecture.getNetworkInstructions()) {
            if (networkInstruction.isUnroll()) {
                Log.error("This cnn architecture uses unrolls, which are currently not supported by the code generator."
                        , architecture.getSourcePosition());
                return false;
            }
        }

        return true;
    }

    public boolean check(ArchitectureSymbol architecture) {
        return checkMultipleStreams(architecture)
                && checkMultipleInputs(architecture)
                && checkMultipleOutputs(architecture)
                && checkMultiDimensionalOutput(architecture)
                && checkConstants(architecture)
                && checkLayerVariables(architecture)
                && checkOutputAsInput(architecture)
                && checkUnroll(architecture);
    }
}
