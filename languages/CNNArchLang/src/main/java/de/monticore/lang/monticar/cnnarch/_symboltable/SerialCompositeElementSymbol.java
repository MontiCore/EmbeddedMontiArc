/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class SerialCompositeElementSymbol extends CompositeElementSymbol {

    protected List<List<ArchitectureElementSymbol>> episodicSubNetworks = new ArrayList<>(new ArrayList<>());
    protected boolean anyEpisodicLocalAdaptation = false;
    protected boolean lossParameterizingElements = false;

    protected void setElements(List<ArchitectureElementSymbol> elements) {
        ArchitectureElementSymbol previous = null;
        for (ArchitectureElementSymbol current : elements){
            // set the AdaNet Flag to true if current is a AdaNet Layer
            if(current.getName().equals(AllPredefinedLayers.AdaNet_Name)){
                // check if the current layer is an AdaNet layer
                this.setAdaNet(true);
                this.setAdaLayer(current);
            }
            if(AllPredefinedLayers.getLossParameterizingLayers().contains(current.getName())){
                // check if architecture has loss parametrizing layers
                lossParameterizingElements = true;
            }
            if(previous != null){
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
    public boolean containsAdaNet(){
        return super.containsAdaNet();}
    protected void setEpisodicSubNetworks(List<List<ArchitectureElementSymbol>> episodicSubNetworks){
        for (List<ArchitectureElementSymbol> subElements: episodicSubNetworks){
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
        this.episodicSubNetworks = episodicSubNetworks;
    }

    public List<List<ArchitectureElementSymbol>> getEpisodicSubNetworks() {
        return episodicSubNetworks;
    }
    
    protected void setAnyEpisodicLocalAdaptation(boolean value) { anyEpisodicLocalAdaptation = value; }

    public boolean getAnyEpisodicLocalAdaptation() { return anyEpisodicLocalAdaptation; }

    public boolean hasLossParameterizingElements() { return lossParameterizingElements; }

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
