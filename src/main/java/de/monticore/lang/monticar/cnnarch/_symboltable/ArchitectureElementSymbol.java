/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.cnnarch._symboltable;

import java.util.*;

public abstract class ArchitectureElementSymbol extends ResolvableSymbol {

    public static final ArchitectureElementKind KIND = new ArchitectureElementKind();

    private ArchitectureElementSymbol inputElement = null;
    private ArchitectureElementSymbol outputElement = null;
    private List<ArchTypeSymbol> outputTypes = null;

    protected ArchitectureElementSymbol(String name) {
        super(name, KIND);
    }

    @Override
    protected ArchitectureElementScope createSpannedScope() {
        return new ArchitectureElementScope();
    }

    @Override
    public ArchitectureElementScope getSpannedScope() {
        return (ArchitectureElementScope) super.getSpannedScope();
    }

    public Optional<ArchitectureElementSymbol> getInputElement() {
        return Optional.ofNullable(inputElement);
    }

    public void setInputElement(ArchitectureElementSymbol inputElement) {
        this.inputElement = inputElement;
    }

    public Optional<ArchitectureElementSymbol> getOutputElement() {
        return Optional.ofNullable(outputElement);
    }

    public void setOutputElement(ArchitectureElementSymbol outputElement) {
        this.outputElement = outputElement;
    }

    //only call after resolve
    public List<ArchTypeSymbol> getOutputTypes() {
        if (outputTypes == null){
            outputTypes = computeOutputTypes();
        }
        return outputTypes;
    }

    protected void setOutputTypes(List<ArchTypeSymbol> outputTypes) {
        this.outputTypes = outputTypes;
    }

    public List<ArchTypeSymbol> getInputTypes() {
        if (getInputElement().isPresent()){
            return getInputElement().get().getOutputTypes();
        }
        else {
            return new ArrayList<>();
        }
    }

    public boolean isInput(){
        //override by IOSymbol
        return false;
    }

    public boolean isOutput(){
        //override by IOSymbol
        return false;
    }

    /**
     * only call after resolve():
     * @return returns the non-empty atomic element which have the output of this element as input.
     */
    public List<ArchitectureElementSymbol> getNext(){
        if (getOutputElement().isPresent()){
            List<ArchitectureElementSymbol> outputElements = new ArrayList<>();
            for (ArchitectureElementSymbol element : getOutputElement().get().getFirstAtomicElements()){
                if (element.getMaxSerialLength().get() == 0){
                    outputElements.addAll(element.getNext());
                }
                else {
                    outputElements.add(element);
                }
            }
            return outputElements;
        }
        else {
            return new ArrayList<>();
        }
    }

    /**
     * only call after resolve():
     * @return returns the non-empty atomic elements which are the input to this element.
     */
    public List<ArchitectureElementSymbol> getPrevious(){
        if (getInputElement().isPresent()){
            List<ArchitectureElementSymbol> inputElements = new ArrayList<>();
            for (ArchitectureElementSymbol element : getInputElement().get().getLastAtomicElements()){
                if (element.getMaxSerialLength().get() == 0){
                    inputElements.addAll(element.getPrevious());
                }
                else {
                    inputElements.add(element);
                }
            }
            return inputElements;
        }
        else {
            return new ArrayList<>();
        }
    }

    protected void setResolvedThis(ArchitectureElementSymbol resolvedThis) {
        super.setResolvedThis(resolvedThis);

        ArchitectureElementSymbol resolvedElement = (ArchitectureElementSymbol) getResolvedThis().get();

        if (getInputElement().isPresent()){
            resolvedElement.setInputElement(getInputElement().get());
        }
        if (getOutputElement().isPresent()){
            resolvedElement.setOutputElement(getOutputElement().get());
        }
    }

    //only call after resolve
    protected abstract List<ArchTypeSymbol> computeOutputTypes();

    abstract public Optional<Integer> getParallelLength();

    abstract public Optional<List<Integer>> getSerialLengths();

    public Optional<Integer> getMaxSerialLength() {
        Optional<List<Integer>> optLengths = getSerialLengths();
        if (optLengths.isPresent()){
            int max = 0;
            for (int length : optLengths.get()){
                if (length > max){
                    max = length;
                }
            }
            return Optional.of(max);
        }
        else {
            return Optional.empty();
        }
    }

    /**
     * only call after resolve.
     * @return returns the first atomic elements which are contained in this element or itself if it is atomic.
     */
    abstract public List<ArchitectureElementSymbol> getFirstAtomicElements();

    /**
     * only call after resolve.
     * @return returns the last atomic elements which are contained in this element or itself if it is atomic.
     */
    abstract public List<ArchitectureElementSymbol> getLastAtomicElements();

    /**
     * A element is called atomic if it is either a active predefined layer, an single input, an single output or an empty composite.
     * This method only works correctly after a successful resolve().
     * @return returns true iff this element is atomic and resolved.
     */
    abstract public boolean isAtomic();

    //only call after resolve; used in coco CheckElementInputs to check the input type and shape of each element.
    abstract public void checkInput();
}
