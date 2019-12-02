/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class CheckIOAccessAndIOMissing extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchitectureSymbol architecture) {
        for (IODeclarationSymbol ioDeclaration : architecture.getIODeclarations()){
            if (ioDeclaration.getArrayLength() == 1){
                checkSingleIO(ioDeclaration);
            }
            else {
                checkIOArray(ioDeclaration);
            }

            if (ioDeclaration.isOutput()) {
                checkOutputWrittenToOnce(ioDeclaration);
            }
        }
    }

    private void checkSingleIO(IODeclarationSymbol ioDeclaration){
        if (ioDeclaration.getConnectedElements().isEmpty()){
            Log.error("0" + ErrorCodes.MISSING_IO + " Input or output '" + ioDeclaration.getName() + "' was declared but never used."
                    , ioDeclaration.getSourcePosition());
        }
        else {
            for (VariableSymbol ioElement : ioDeclaration.getConnectedElements()){
                if (ioElement.getArrayAccess().isPresent()){
                    Log.error("0" + ErrorCodes.INVALID_ARRAY_ACCESS + " Invalid IO array access. " +
                                    "This input or output is not an array."
                            , ioElement.getSourcePosition());
                }
            }
        }
    }

    private void checkIOArray(IODeclarationSymbol ioDeclaration){
        List<Integer> unusedIndices = IntStream.range(0, ioDeclaration.getArrayLength()).boxed().collect(Collectors.toList());

        for (VariableSymbol ioElement : ioDeclaration.getConnectedElements()){
            if (ioElement.getArrayAccess().isPresent()){
                Optional<Integer> arrayAccess = ioElement.getArrayAccess().get().getIntValue();
                if (arrayAccess.isPresent() && arrayAccess.get() >= 0 && arrayAccess.get() < ioDeclaration.getArrayLength()){
                    unusedIndices.remove(arrayAccess.get());
                }
                else {
                    Log.error("0" + ErrorCodes.INVALID_ARRAY_ACCESS + " The IO array access value of '" + ioElement.getName() +
                                    "' must be an integer between 0 and " + (ioDeclaration.getArrayLength()-1) + ". " +
                                    "The current value is: " + ioElement.getArrayAccess().get().getValue().get().toString()
                                , ioElement.getSourcePosition());
                }
            }
            else{
                unusedIndices = new ArrayList<>();
            }
        }

        if (!unusedIndices.isEmpty()){
            Log.error("0" + ErrorCodes.MISSING_IO + " Input or output array with name '" + ioDeclaration.getName() + "' was declared but not used. " +
                            "The following indices are unused: " + Joiners.COMMA.join(unusedIndices) + "."
                    , ioDeclaration.getSourcePosition());
        }
    }

    private void checkOutputWrittenToOnce(IODeclarationSymbol ioDeclaration) {
        List<Integer> written = new ArrayList<>();

        for (NetworkInstructionSymbol networkInstruction : ioDeclaration.getArchitecture().getNetworkInstructions()) {
            if (networkInstruction.isStream()) {
                SerialCompositeElementSymbol body = networkInstruction.getBody();
                List<ArchitectureElementSymbol> outputs = body.getLastAtomicElements();

                for (ArchitectureElementSymbol output : outputs) {
                    if (output instanceof VariableSymbol) {
                        VariableSymbol variable = (VariableSymbol) output;

                        if (variable.getName().equals(ioDeclaration.getName())) {
                            int arrayAccess = 0;

                            if (variable.getArrayAccess().isPresent()) {
                                arrayAccess = variable.getArrayAccess().get().getIntValue().orElse(0);
                            }

                            if (!written.contains(arrayAccess)) {
                                written.add(arrayAccess);
                            } else {
                                Log.error("0" + ErrorCodes.OUTPUT_WRITTEN_TO_MULTIPLE_TIMES + " " + variable.getName() + "["
                                                + arrayAccess + "] is written to multiple times, this is currently not allowed."
                                        , networkInstruction.getSourcePosition());
                            }
                        }
                    }
                }
            } else if (networkInstruction.isUnroll()) {
                for (SerialCompositeElementSymbol body : networkInstruction.toUnrollInstruction().getResolvedBodies()) {
                    List<ArchitectureElementSymbol> outputs = body.getLastAtomicElements();

                    for (ArchitectureElementSymbol output : outputs) {
                        if (output instanceof VariableSymbol) {
                            VariableSymbol variable = (VariableSymbol) output;

                            if (variable.getName().equals(ioDeclaration.getName())) {
                                int arrayAccess = 0;

                                if (variable.getArrayAccess().isPresent()) {
                                    arrayAccess = variable.getArrayAccess().get().getIntValue().orElse(0);
                                }

                                if (!written.contains(arrayAccess)) {
                                    written.add(arrayAccess);
                                } else {
                                    Log.error("0" + ErrorCodes.OUTPUT_WRITTEN_TO_MULTIPLE_TIMES + " " + variable.getName() + "["
                                                    + arrayAccess + "] is written to multiple times, this is currently not allowed."
                                            , networkInstruction.getSourcePosition());
                                }
                            }
                        }
                    }
                }
            }
        }
    }

}
