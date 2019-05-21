package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.se_rwth.commons.logging.Log;

public class ArchitectureSupportChecker {

    public ArchitectureSupportChecker() {}

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

    public boolean check(ArchitectureSymbol architecture) {
        return checkMultipleStreams(architecture)
                && checkMultipleInputs(architecture)
                && checkMultipleOutputs(architecture)
                && checkMultiDimensionalOutput(architecture);
    }
}
