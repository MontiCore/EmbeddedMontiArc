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
package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.predefined.Convolution;
import de.monticore.lang.monticar.cnnarch.predefined.FullyConnected;
import de.monticore.lang.monticar.cnnarch.predefined.Pooling;

import java.util.*;

public class LayerNameCreator {

    private Map<ArchitectureElementSymbol, String> elementToName = new HashMap<>();
    private Set<String> names = new HashSet<>();

    public LayerNameCreator(ArchitectureSymbol architecture) {
        int stage = 1;
        for (NetworkInstructionSymbol networkInstruction : architecture.getNetworkInstructions()) {
            stage = name(networkInstruction.getBody(), stage, new ArrayList<>());

            if (networkInstruction.isUnroll()) {
                UnrollInstructionSymbol unroll = (UnrollInstructionSymbol) networkInstruction;

                for (SerialCompositeElementSymbol body : unroll.getResolvedBodies()) {
                    stage = name(body, stage, new ArrayList<>());
                }
            }
        }
    }

    public String getName(ArchitectureElementSymbol architectureElement){
        return elementToName.get(architectureElement);
    }

    protected int name(ArchitectureElementSymbol architectureElement, int stage, List<Integer> streamIndices){
        if (architectureElement instanceof SerialCompositeElementSymbol) {
            return nameSerialComposite((SerialCompositeElementSymbol) architectureElement, stage, streamIndices);
        } else if (architectureElement instanceof ParallelCompositeElementSymbol) {
            return nameParallelComposite((ParallelCompositeElementSymbol) architectureElement, stage, streamIndices);
        } else {
            if (architectureElement.isAtomic()) {
                if (architectureElement.getMaxSerialLength().get() > 0){
                    return add(architectureElement, stage, streamIndices);
                } else {
                    return stage;
                }
            } else {
                ArchitectureElementSymbol resolvedElement = (ArchitectureElementSymbol) architectureElement.getResolvedThis().get();
                return name(resolvedElement, stage, streamIndices);
            }
        }
    }

    protected int nameSerialComposite(SerialCompositeElementSymbol compositeElement, int stage, List<Integer> streamIndices){
        int endStage = stage;
        for (ArchitectureElementSymbol subElement : compositeElement.getElements()){
            endStage = name(subElement, endStage, streamIndices);
        }
        return endStage;
    }

    protected int nameParallelComposite(ParallelCompositeElementSymbol compositeElement, int stage, List<Integer> streamIndices){
        int startStage = stage + 1;
        streamIndices.add(1);
        int lastIndex = streamIndices.size() - 1;

        List<Integer> endStages = new ArrayList<>();
        for (ArchitectureElementSymbol subElement : compositeElement.getElements()){
            endStages.add(name(subElement, startStage, streamIndices));
            streamIndices.set(lastIndex, streamIndices.get(lastIndex) + 1);
        }

        streamIndices.remove(lastIndex);
        return Collections.max(endStages) + 1;
    }

    protected int add(ArchitectureElementSymbol architectureElement, int stage, List<Integer> streamIndices){
        int endStage = stage;
        if (!elementToName.containsKey(architectureElement)) {
            String name = createName(architectureElement, endStage, streamIndices);

            if (!(architectureElement instanceof VariableSymbol)) {
                while (names.contains(name)) {
                    endStage++;
                    name = createName(architectureElement, endStage, streamIndices);
                }
            }

            elementToName.put(architectureElement, name);
            names.add(name);
        }
        return endStage;
    }

    protected String createName(ArchitectureElementSymbol architectureElement, int stage, List<Integer> streamIndices){
        if (architectureElement instanceof VariableSymbol) {
            VariableSymbol element = (VariableSymbol) architectureElement;

            String name = createBaseName(architectureElement) + "_";

            if (element.getType() == VariableSymbol.Type.LAYER) {
                if (element.getMember() == VariableSymbol.Member.STATE) {
                    name = name + "state_";
                } else {
                    name = name + "output_";
                }
            }

            if (element.getArrayAccess().isPresent()){
                int arrayAccess = element.getArrayAccess().get().getIntValue().get();
                name = name + arrayAccess + "_";
            }

            return name;
        } else {
            return createBaseName(architectureElement) + stage + createStreamPostfix(streamIndices) + "_";
        }
    }


    protected String createBaseName(ArchitectureElementSymbol architectureElement){
        if (architectureElement instanceof LayerSymbol) {
            LayerDeclarationSymbol layerDeclaration = ((LayerSymbol) architectureElement).getDeclaration();
            if (layerDeclaration instanceof Convolution) {
                return "conv";
            } else if (layerDeclaration instanceof FullyConnected) {
                return "fc";
            } else if (layerDeclaration instanceof Pooling) {
                return "pool";
            } else {
                return layerDeclaration.getName().toLowerCase();
            }
        } else if (architectureElement instanceof CompositeElementSymbol){
            return "group";
        } else {
            return architectureElement.getName();
        }
    }

    protected String createStreamPostfix(List<Integer> streamIndices){
        StringBuilder stringBuilder = new StringBuilder();
        for (int streamIndex : streamIndices){
            stringBuilder.append("_");
            stringBuilder.append(streamIndex);
        }
        return stringBuilder.toString();
    }
}
