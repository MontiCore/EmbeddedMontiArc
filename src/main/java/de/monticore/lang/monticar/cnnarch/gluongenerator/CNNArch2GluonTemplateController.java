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
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.generator.ArchitectureElementData;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchTemplateController;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.se_rwth.commons.logging.Log;

import java.io.Writer;
import java.util.*;

public class CNNArch2GluonTemplateController extends CNNArchTemplateController {
    public static final String NET_DEFINITION_MODE_KEY = "mode";

    public CNNArch2GluonTemplateController(ArchitectureSymbol architecture,
                                           TemplateConfiguration templateConfiguration) {
        super(architecture, templateConfiguration);
    }

    public void include(String relativePath, String templateWithoutFileEnding, Writer writer, NetDefinitionMode netDefinitionMode){
        String templatePath = relativePath + templateWithoutFileEnding + FTL_FILE_ENDING;
        Map<String, Object> ftlContext = new HashMap<>();
        ftlContext.put(TEMPLATE_CONTROLLER_KEY, this);
        ftlContext.put(ELEMENT_DATA_KEY, getCurrentElement());
        ftlContext.put(NET_DEFINITION_MODE_KEY, netDefinitionMode.toString());
        getTemplateConfiguration().processTemplate(ftlContext, templatePath, writer);
    }

    public void include(VariableSymbol element, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(element);

        if (element.isAtomic()){
            if (element.getType() == VariableSymbol.Type.IO) {
                if (element.isInput()){
                    include(TEMPLATE_ELEMENTS_DIR_PATH, "Input", writer, netDefinitionMode);
                }
                else {
                    include(TEMPLATE_ELEMENTS_DIR_PATH, "Output", writer, netDefinitionMode);
                }
            }
            else if (element.getType() == VariableSymbol.Type.LAYER) {
                if (element.getMember() == VariableSymbol.Member.STATE) {
                    include(TEMPLATE_ELEMENTS_DIR_PATH, "Output", writer, netDefinitionMode);
                } else if (element.getMember() == VariableSymbol.Member.NONE) {
                    include(TEMPLATE_ELEMENTS_DIR_PATH, element.getLayerVariableDeclaration().getLayer().getName(), writer, netDefinitionMode);
                }
            }
        }
        else {
            include((ArchitectureElementSymbol) element.getResolvedThis().get(), writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(LayerSymbol layer, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(layer);

        if (layer.isAtomic()){
            String templateName = layer.getDeclaration().getName();
            include(TEMPLATE_ELEMENTS_DIR_PATH, templateName, writer, netDefinitionMode);
        }
        else {
            include((ArchitectureElementSymbol) layer.getResolvedThis().get(), writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(CompositeElementSymbol compositeElement, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(compositeElement);

        for (ArchitectureElementSymbol element : compositeElement.getElements()){
            include(element, writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(ArchitectureElementSymbol architectureElement, Writer writer, NetDefinitionMode netDefinitionMode){
        if (architectureElement instanceof CompositeElementSymbol){
            include((CompositeElementSymbol) architectureElement, writer, netDefinitionMode);
        }
        else if (architectureElement instanceof LayerSymbol){
            include((LayerSymbol) architectureElement, writer, netDefinitionMode);
        }
        else if (architectureElement instanceof ConstantSymbol) {

        }
        else {
            include((VariableSymbol) architectureElement, writer, netDefinitionMode);
        }
    }

    public void include(ArchitectureElementSymbol architectureElementSymbol, String netDefinitionMode) {
        include(architectureElementSymbol, NetDefinitionMode.fromString(netDefinitionMode));
    }

    public void include(ArchitectureElementSymbol architectureElement, NetDefinitionMode netDefinitionMode){
        if (getWriter() == null){
            throw new IllegalStateException("missing writer");
        }
        include(architectureElement, getWriter(), netDefinitionMode);
    }

    public Set<String> getStreamInputNames(SerialCompositeElementSymbol stream) {
        return getStreamInputs(stream).keySet();
    }

    // used for unroll
    public List<String> getStreamInputNames(SerialCompositeElementSymbol stream, SerialCompositeElementSymbol currentStream) {
        List<String> inputNames = new LinkedList<>(getStreamInputNames(stream));
        Map<String, String> pairs = getUnrollPairs(stream, currentStream);

        for (int i = 0; i != inputNames.size(); ++i) {
            if (pairs.containsKey(inputNames.get(i))) {
                inputNames.set(i, pairs.get(inputNames.get(i)));
            }
        }

        return inputNames;
    }

    public Collection<List<String>> getStreamInputDimensions(SerialCompositeElementSymbol stream) {
        return getStreamInputs(stream).values();
    }

    public Set<String> getStreamOutputNames(SerialCompositeElementSymbol stream) {
        Set<String> outputNames = new LinkedHashSet<>();

        for (ArchitectureElementSymbol element : stream.getLastAtomicElements()) {
            if (element.isOutput()) {
                outputNames.add(getName(element));
            }
        }

        outputNames.addAll(getStreamLayerVariableMembers(stream, true).keySet());

        return outputNames;
    }

    // used for unroll
    public List<String> getStreamOutputNames(SerialCompositeElementSymbol stream, SerialCompositeElementSymbol currentStream) {
        List<String> outputNames = new LinkedList<>(getStreamOutputNames(stream));
        Map<String, String> pairs = getUnrollPairs(stream, currentStream);

        for (int i = 0; i != outputNames.size(); ++i) {
            if (pairs.containsKey(outputNames.get(i))) {
                outputNames.set(i, pairs.get(outputNames.get(i)));
            }
        }

        return outputNames;
    }

    // Used to initialize all layer variable members which are passed through the networks
    public  Map<String, List<String>> getLayerVariableMembers() {
        Map<String, List<String>> members = new LinkedHashMap<>();

        for (SerialCompositeElementSymbol stream : getArchitecture().getStreams()) {
            members.putAll(getStreamLayerVariableMembers(stream, true));
        }

        return members;
    }

    // Calculate differently named VariableSymbol elements in two streams, currently used for the UnrollInstructionSymbol
    // body which is resolved with t = CONST_OFFSET and the current body of the actual timestep t
    public Map<String, String> getUnrollPairs(ArchitectureElementSymbol element, ArchitectureElementSymbol current) {
        Map<String, String> pairs = new HashMap<>();

        if (element instanceof CompositeElementSymbol && current instanceof CompositeElementSymbol) {
            List<ArchitectureElementSymbol> elements = ((CompositeElementSymbol) element).getElements();
            List<ArchitectureElementSymbol> currentElements = ((CompositeElementSymbol) current).getElements();

            if (elements.size() == currentElements.size()) {
                for (int i = 0; i != currentElements.size(); ++i) {
                    String name = getName(elements.get(i));
                    String currentName = getName(currentElements.get(i));

                    if (elements.get(i) instanceof VariableSymbol && currentElements.get(i) instanceof VariableSymbol) {
                        if (name != null && currentName != null && !name.equals(currentName)) {
                            pairs.put(name, currentName);
                        }
                    }

                    pairs.putAll(getUnrollPairs(elements.get(i), currentElements.get(i)));
                }
            }
        }

        return pairs;
    }

    private Map<String, List<String>> getStreamInputs(SerialCompositeElementSymbol stream) {
        Map<String, List<String>> inputs = new LinkedHashMap<>();

        for (ArchitectureElementSymbol element : stream.getFirstAtomicElements()) {
            if (element.isInput() || element.isOutput()) {
                List<Integer> intDimensions = element.getOutputTypes().get(0).getDimensions();

                List<String> dimensions = new ArrayList<>();
                for (Integer intDimension : intDimensions) {
                    dimensions.add(intDimension.toString());
                }

                inputs.put(getName(element), dimensions);
            }
            else if (element instanceof ConstantSymbol) {
                inputs.put(getName(element), Arrays.asList("1"));
            }
        }

        inputs.putAll(getStreamLayerVariableMembers(stream, false));

        return inputs;
    }

    private Map<String, List<String>> getStreamLayerVariableMembers(SerialCompositeElementSymbol stream, boolean includeOutput) {
        Map<String, List<String>> members = new LinkedHashMap<>();

        List<ArchitectureElementSymbol> elements = stream.getSpannedScope().resolveLocally(ArchitectureElementSymbol.KIND);
        for (ArchitectureElementSymbol element : elements) {
            if (element instanceof VariableSymbol) {
                VariableSymbol variable = (VariableSymbol) element;

                if (variable.getType() == VariableSymbol.Type.LAYER && (variable.getMember() == VariableSymbol.Member.NONE)) {
                    LayerVariableDeclarationSymbol layerVariableDeclaration = variable.getLayerVariableDeclaration();

                    if (layerVariableDeclaration.getLayer().getDeclaration().isPredefined()) {
                        PredefinedLayerDeclaration predefinedLayerDeclaration =
                                (PredefinedLayerDeclaration) layerVariableDeclaration.getLayer().getDeclaration();

                        int arrayLength = predefinedLayerDeclaration.getArrayLength(VariableSymbol.Member.STATE);

                        for (int i = 0; i < arrayLength; ++i) {
                            String name = variable.getName() + "_state_";

                            if (arrayLength > 1) {
                                name += i + "_";
                            }

                            List<Integer> intDimensions = predefinedLayerDeclaration.computeOutputTypes(
                                    layerVariableDeclaration.getLayer().getInputTypes(),
                                    layerVariableDeclaration.getLayer(),
                                    VariableSymbol.Member.STATE
                            ).get(0).getDimensions();

                            List<String> dimensions = new ArrayList<>();

                            for (Integer intDimension : intDimensions) {
                                dimensions.add(intDimension.toString());
                            }

                            members.put(name, dimensions);
                        }

                        if (includeOutput) {
                            arrayLength = predefinedLayerDeclaration.getArrayLength(VariableSymbol.Member.OUTPUT);

                            for (int i = 0; i < arrayLength; ++i) {
                                String name = variable.getName() + "_output_";

                                if (arrayLength > 1) {
                                    name += i + "_";
                                }

                                List<Integer> intDimensions = predefinedLayerDeclaration.computeOutputTypes(
                                        layerVariableDeclaration.getLayer().getInputTypes(),
                                        layerVariableDeclaration.getLayer(),
                                        VariableSymbol.Member.OUTPUT
                                ).get(0).getDimensions();

                                List<String> dimensions = new ArrayList<>();

                                for (Integer intDimension : intDimensions) {
                                    dimensions.add(intDimension.toString());
                                }

                                members.put(name, dimensions);
                            }
                        }
                    }
                }
            }
        }
        return members;
    }

    // cuts
    public List<String> cutDimensions(List<String> dimensions) {
        while (dimensions.size() > 1 && dimensions.get(dimensions.size() - 1).equals("1")) {
            dimensions.remove(dimensions.size() - 1);
        }

        return dimensions;
    }

    public int getBeamSearchWidth(UnrollInstructionSymbol unroll){
        return unroll.getIntValue(AllPredefinedLayers.WIDTH_NAME).get();
    }

    public int getBeamSearchLength(UnrollInstructionSymbol unroll){
        return unroll.getIntValue(AllPredefinedLayers.MAX_LENGTH_NAME).get();
    }

}
