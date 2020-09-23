/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class ParallelCompositeElementSymbol extends CompositeElementSymbol {

    protected void setElements(List<ArchitectureElementSymbol> elements) {
        for (ArchitectureElementSymbol current : elements){
            if (getInputElement().isPresent()){
                current.setInputElement(getInputElement().get());
            }
            if (getOutputElement().isPresent()){
                current.setOutputElement(getOutputElement().get());
            }
        }
        this.elements = elements;
    }

    @Override
    public void setInputElement(ArchitectureElementSymbol inputElement) {
        super.setInputElement(inputElement);
        for (ArchitectureElementSymbol current : getElements()){
            current.setInputElement(inputElement);
        }
    }

    @Override
    public void setOutputElement(ArchitectureElementSymbol outputElement) {
        super.setOutputElement(outputElement);
        for (ArchitectureElementSymbol current : getElements()){
            current.setOutputElement(outputElement);
        }
    }

    @Override
    public List<ArchitectureElementSymbol> getFirstAtomicElements() {
        if (getElements().isEmpty()){
            return Collections.singletonList(this);
        }
        else {
            List<ArchitectureElementSymbol> firstElements = new ArrayList<>();
            for (ArchitectureElementSymbol element : getElements()){
                firstElements.addAll(element.getFirstAtomicElements());
            }
            return firstElements;
        }
    }

    @Override
    public List<ArchitectureElementSymbol> getLastAtomicElements() {
        if (getElements().isEmpty()){
            return Collections.singletonList(this);
        }
        else {
            List<ArchitectureElementSymbol> lastElements = new ArrayList<>();
            for (ArchitectureElementSymbol element : getElements()){
                lastElements.addAll(element.getLastAtomicElements());
            }
            return lastElements;
        }
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes() {
        if (getElements().isEmpty()){
            if (getInputElement().isPresent()){
                return getInputElement().get().getOutputTypes();
            }
            else {
                return Collections.emptyList();
            }
        }
        else {
            List<ArchTypeSymbol> outputShapes = new ArrayList<>(getElements().size());
            for (ArchitectureElementSymbol element : getElements()){
                if (element.getOutputTypes().size() != 0){
                    outputShapes.add(element.getOutputTypes().get(0));
                }
            }
            return outputShapes;
        }
    }

    @Override
    public void checkInput() {
        if (!getElements().isEmpty()){
            for (ArchitectureElementSymbol element : getElements()){
                element.checkInput();
            }
            for (ArchitectureElementSymbol element : getElements()){
                if (element.getOutputTypes().size() > 1){
                    Log.error("0" + ErrorCodes.MISSING_MERGE + " Missing merge layer (Add(), Concatenate() or [i]). " +
                                    "Each stream at the end of a parallelization block can only have one output stream. "
                            , getSourcePosition());
                }
            }
        }
    }

    @Override
    public Optional<Integer> getParallelLength() {
        return Optional.of(getElements().size());
    }

    @Override
    public Optional<List<Integer>> getSerialLengths() {
        return Optional.of(Collections.nCopies(getElements().size(), 1));
    }

    @Override
    protected ParallelCompositeElementSymbol preResolveDeepCopy() {
        ParallelCompositeElementSymbol copy = new ParallelCompositeElementSymbol();
        if (getAstNode().isPresent()){
            copy.setAstNode(getAstNode().get());
        }

        List<ArchitectureElementSymbol> elements = new ArrayList<>(getElements().size());
        for (ArchitectureElementSymbol element : getElements()){
            ArchitectureElementSymbol elementCopy = (ArchitectureElementSymbol) element.preResolveDeepCopy();
            elements.add(elementCopy);
        }
        copy.setElements(elements);
        return copy;
    }
}
