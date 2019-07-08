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
            ArchitectureElementSymbol elementCopy = element.preResolveDeepCopy();
            elements.add(elementCopy);
        }
        copy.setElements(elements);
        return copy;
    }
}
