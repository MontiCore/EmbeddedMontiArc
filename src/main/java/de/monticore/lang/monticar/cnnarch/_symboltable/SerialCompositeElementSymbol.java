/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import java.util.*;

public class SerialCompositeElementSymbol extends CompositeElementSymbol {

    protected List<List<ArchitectureElementSymbol>> replaySubNetworks = new ArrayList<>(new ArrayList<>());

    protected void setElements(List<ArchitectureElementSymbol> elements) {
        ArchitectureElementSymbol previous = null;
        for (ArchitectureElementSymbol current : elements){
            if (previous != null){
                current.setInputElement(previous);
                previous.setOutputElement(current);
            }
            else {
                if (getInputElement().isPresent()){
                    current.setInputElement(getInputElement().get());
                }
                if (getOutputElement().isPresent()){
                    current.setOutputElement(getOutputElement().get());
                }
            }
            previous = current;
        }
        this.elements = elements;
    }

    protected void setReplaySubNetworks(List<List<ArchitectureElementSymbol>> replaySubNetworks){
        for (List<ArchitectureElementSymbol> subElements: replaySubNetworks){
            ArchitectureElementSymbol previous = null;
            for (ArchitectureElementSymbol current : subElements){
                if (previous != null){
                    current.setInputElement(previous);
                    previous.setOutputElement(current);
                }
                else {
                    if (getInputElement().isPresent()){
                        current.setInputElement(getInputElement().get());
                    }
                    if (getOutputElement().isPresent()){
                        current.setOutputElement(getOutputElement().get());
                    }
                }
                previous = current;
            }
        }
        this.replaySubNetworks = replaySubNetworks;
    }

    public List<List<ArchitectureElementSymbol>> getReplaySubNetworks() {
        return replaySubNetworks;
    }

    @Override
    public void setInputElement(ArchitectureElementSymbol inputElement) {
        super.setInputElement(inputElement);
        if (!getElements().isEmpty()){
            getElements().get(0).setInputElement(inputElement);
        }
    }

    @Override
    public void setOutputElement(ArchitectureElementSymbol outputElement) {
        super.setOutputElement(outputElement);
        if (!getElements().isEmpty()){
            getElements().get(getElements().size()-1).setOutputElement(outputElement);
        }
    }

    @Override
    public List<ArchitectureElementSymbol> getFirstAtomicElements() {
        if (getElements().isEmpty()){
            return Collections.singletonList(this);
        }
        else {
            return getElements().get(0).getFirstAtomicElements();
        }
    }

    @Override
    public List<ArchitectureElementSymbol> getLastAtomicElements() {
        if (getElements().isEmpty()){
            return Collections.singletonList(this);
        }
        else {
            return getElements().get(getElements().size()-1).getLastAtomicElements();
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
            for (ArchitectureElementSymbol element : getElements()){
                element.getOutputTypes();
            }
            return getElements().get(getElements().size() - 1).getOutputTypes();
        }
    }

    @Override
    public void checkInput() {
        for (ArchitectureElementSymbol element : getElements()){
            element.checkInput();
        }
    }

    @Override
    public Optional<Integer> getParallelLength() {
        return Optional.of(1);
    }

    @Override
    public Optional<List<Integer>> getSerialLengths() {
        return Optional.of(Collections.singletonList(getElements().size()));
    }

    @Override
    protected SerialCompositeElementSymbol preResolveDeepCopy() {
        SerialCompositeElementSymbol copy = new SerialCompositeElementSymbol();
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
