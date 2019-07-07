package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ConstantSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.SerialCompositeElementSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.List;

public abstract class ArchitectureSupportChecker {

    // Overload functions returning always true to enable the features
    protected boolean checkMultipleStreams(ArchitectureSymbol architecture) {
        if (architecture.getStreams().size() != 1) {
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
        if (architecture.getOutputs().get(0).getDefinition().getType().getWidth() != 1 ||
            architecture.getOutputs().get(0).getDefinition().getType().getHeight() != 1) {
            Log.error("This cnn architecture has a multi-dimensional output, " +
                      "which is currently not supported by the code generator."
                      , architecture.getSourcePosition());

            return false;
        }

        return true;
    }

    protected boolean hasConstant(ArchitectureElementSymbol element) {
        ArchitectureElementSymbol resolvedElement = element.getResolvedThis().get();

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
        for (SerialCompositeElementSymbol stream : architecture.getStreams()) {
            for (ArchitectureElementSymbol element : stream.getElements()) {
                if (hasConstant(element)) {
                    Log.error("This cnn architecture has a constant, which is currently not supported by the code generator."
                            , architecture.getSourcePosition());
                    return false;
                }
            }
        }

        return true;
    }

    public boolean check(ArchitectureSymbol architecture) {
        return checkMultipleStreams(architecture)
                && checkMultipleInputs(architecture)
                && checkMultipleOutputs(architecture)
                && checkMultiDimensionalOutput(architecture)
                && checkConstants(architecture);
    }
}
