/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.predefined.Convolution;
import de.monticore.lang.monticar.cnnarch.predefined.FullyConnected;
import de.monticore.lang.monticar.cnnarch.predefined.Pooling;

import java.util.*;

public class LayerNameCreator {

    private Map<ArchitectureElementSymbol, String> elementToName = new HashMap<>();
    private Map<String, ArchitectureElementSymbol> nameToElement = new HashMap<>();

    public LayerNameCreator(ArchitectureSymbol architecture) {
        int stage = 1;
        for (SerialCompositeElementSymbol stream : architecture.getStreams()) {
            stage = name(stream, stage, new ArrayList<>());
        }
    }

    public ArchitectureElementSymbol getArchitectureElement(String name){
        return nameToElement.get(name);
    }

    public String getName(ArchitectureElementSymbol architectureElement){
        return elementToName.get(architectureElement);
    }

    protected int name(ArchitectureElementSymbol architectureElement, int stage, List<Integer> streamIndices){
        if (architectureElement instanceof SerialCompositeElementSymbol) {
            return nameSerialComposite((SerialCompositeElementSymbol) architectureElement, stage, streamIndices);
        } else if (architectureElement instanceof ParallelCompositeElementSymbol){
            return nameParallelComposite((ParallelCompositeElementSymbol) architectureElement, stage, streamIndices);
        } else{
            if (architectureElement.isAtomic()){
                if (architectureElement.getMaxSerialLength().get() > 0){
                    return add(architectureElement, stage, streamIndices);
                } else {
                    return stage;
                }
            } else {
                ArchitectureElementSymbol resolvedElement = architectureElement.getResolvedThis().get();
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

            while (nameToElement.containsKey(name)) {
                endStage++;
                name = createName(architectureElement, endStage, streamIndices);
            }

            elementToName.put(architectureElement, name);

            boolean isLayerVariable = false;

            if (architectureElement instanceof VariableSymbol) {
                isLayerVariable = ((VariableSymbol) architectureElement).getType() == VariableSymbol.Type.LAYER;
            }

            // Do not map names of layer variables to their respective element since the names are not unique
            // for now the name to element mapping is not used anywhere so it doesn't matter
            if (!isLayerVariable) {
                nameToElement.put(name, architectureElement);
            }
        }
        return endStage;
    }

    protected String createName(ArchitectureElementSymbol architectureElement, int stage, List<Integer> streamIndices){
        if (architectureElement instanceof VariableSymbol) {
            VariableSymbol element = (VariableSymbol) architectureElement;

            String name = createBaseName(architectureElement);

            if (element.getType() == VariableSymbol.Type.IO) {
                if (element.getArrayAccess().isPresent()){
                    int arrayAccess = element.getArrayAccess().get().getIntValue().get();
                    name = name + "_" + arrayAccess + "_";
                } else {
                    name = name + "_";
                }
            } else if (element.getType() == VariableSymbol.Type.LAYER) {
                if (element.getMember() == VariableSymbol.Member.STATE) {
                    name = name + "_state_";
                } else {
                    name = name + "_output_";
                }
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

