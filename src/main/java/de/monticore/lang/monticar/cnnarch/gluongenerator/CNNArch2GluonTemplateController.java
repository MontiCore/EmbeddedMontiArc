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

import de.monticore.lang.monticar.cnnarch._ast.ASTStream;
import de.monticore.lang.monticar.cnnarch.generator.ArchitectureElementData;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchTemplateController;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;

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

    public void include(VariableSymbol element, boolean partOfUnroll, int unrollIndex, Writer writer, NetDefinitionMode netDefinitionMode){
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
                include(TEMPLATE_ELEMENTS_DIR_PATH, element.getLayerVariableDeclaration().getLayer().getName(), writer, netDefinitionMode);
            }
        }
        else {
            include((ArchitectureElementSymbol) element.getResolvedThis().get(), partOfUnroll, unrollIndex, writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(ConstantSymbol constant, boolean partOfUnroll, int unrollIndex, Writer writer, NetDefinitionMode netDefinitionMode) {
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(constant);

        if (constant.isAtomic()) {
            include(TEMPLATE_ELEMENTS_DIR_PATH, "Const", writer, netDefinitionMode);
        }
        else {
            include((ArchitectureElementSymbol) constant.getResolvedThis().get(), partOfUnroll, unrollIndex, writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(LayerSymbol layer, boolean partOfUnroll, int unrollIndex, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(layer);
        getCurrentElement().setPartOfUnroll(partOfUnroll);
        getCurrentElement().setUnrollIndex(unrollIndex);

        if (layer.isAtomic()){
            String templateName = layer.getDeclaration().getName();
            include(TEMPLATE_ELEMENTS_DIR_PATH, templateName, writer, netDefinitionMode);
        }
        else {
            include((ArchitectureElementSymbol) layer.getResolvedThis().get(), partOfUnroll, unrollIndex, writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(UnrollSymbol unrollElement, boolean partOfUnroll, int unrollIndex, Writer writer, NetDefinitionMode netDefinitionMode){
        include(unrollElement.getBody(), partOfUnroll, unrollIndex, writer, netDefinitionMode);
        String templateName = unrollElement.getDeclaration().getName();
        include(TEMPLATE_ELEMENTS_DIR_PATH, templateName, writer, netDefinitionMode);
    }

    public void include(CompositeElementSymbol compositeElement, boolean partOfUnroll, int unrollIndex, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(compositeElement);

        for (ArchitectureElementSymbol element : compositeElement.getElements()){
            include(element, partOfUnroll, unrollIndex, writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(ArchitectureElementSymbol architectureElement, boolean partOfUnroll, int unrollIndex, Writer writer, NetDefinitionMode netDefinitionMode){
        if (architectureElement instanceof CompositeElementSymbol){
            include((CompositeElementSymbol) architectureElement, partOfUnroll, unrollIndex, writer, netDefinitionMode);
        }
        else if (architectureElement instanceof LayerSymbol){
            include((LayerSymbol) architectureElement, partOfUnroll, unrollIndex, writer, netDefinitionMode);
        }
        else if (architectureElement instanceof ConstantSymbol) {
            include((ConstantSymbol) architectureElement, partOfUnroll, unrollIndex, writer, netDefinitionMode);
        }
        else {
            include((VariableSymbol) architectureElement, partOfUnroll, unrollIndex, writer, netDefinitionMode);
        }
    }

    public void include(ArchitectureElementSymbol architectureElementSymbol, String netDefinitionMode) {
        include(architectureElementSymbol, false, -1, NetDefinitionMode.fromString(netDefinitionMode));
    }

    public void include(ArchitectureElementSymbol architectureElementSymbol, boolean partOfUnroll, int unrollIndex, String netDefinitionMode) {

        int layerIndex = -1;
        include(architectureElementSymbol, partOfUnroll, unrollIndex, NetDefinitionMode.fromString(netDefinitionMode));
    }

    public void include(UnrollSymbol unrollSymbol, boolean partOfUnroll, int unrollIndex, String netDefinitionMode) {
        include(unrollSymbol, partOfUnroll, unrollIndex, NetDefinitionMode.fromString(netDefinitionMode));
    }


    public void include(ArchitectureElementSymbol architectureElement, boolean partOfUnroll, int unrollIndex, NetDefinitionMode netDefinitionMode){
        if (getWriter() == null){
            throw new IllegalStateException("missing writer");
        }
        include(architectureElement, partOfUnroll, unrollIndex, getWriter(), netDefinitionMode);
    }

    public void include(UnrollSymbol unroll, boolean partOfUnroll, int unrollIndex, NetDefinitionMode netDefinitionMode){
        if (getWriter() == null){
            throw new IllegalStateException("missing writer");
        }
        include(unroll, partOfUnroll, unrollIndex, getWriter(), netDefinitionMode);
    }

    public Set<String> getStreamInputNames(SerialCompositeElementSymbol stream) {
        return getStreamInputs(stream).keySet();
    }

    public Set<String> getUnrollInputNames(UnrollSymbol unroll) {
        return getUnrollInputs(unroll).keySet();
    }

    public Collection<List<String>> getStreamInputDimensions(SerialCompositeElementSymbol stream) {
        return getStreamInputs(stream).values();
    }

    public Collection<List<String>> getUnrollInputDimensions(UnrollSymbol unroll) {
        return getUnrollInputs(unroll).values();
    }

    public Set<String> getStreamOutputNames(SerialCompositeElementSymbol stream) {
        Set<String> outputNames = new LinkedHashSet<>();

        for (ArchitectureElementSymbol element : stream.getLastAtomicElements()) {
            if (element.isOutput()) {
                outputNames.add(getName(element));
            }
        }

        outputNames.addAll(getStreamLayerVariableMembers(stream, "1", true).keySet());

        return outputNames;
    }

    public Set<String> getUnrollOutputNames(UnrollSymbol unroll) {
        Set<String> outputNames = new LinkedHashSet<>();

        for (ArchitectureElementSymbol element : unroll.getBody().getElements()) {
            if (element.isOutput()) {
                outputNames.add(getName(element));
            }
        }

        outputNames.addAll(getStreamLayerVariableMembers(unroll.getBody(), "1", true).keySet());

        return outputNames;
    }

    // Used to initialize all layer variable members which are passed through the networks
    public Map<String, List<String>> getLayerVariableMembers(String batchSize) {
        Map<String, List<String>> members = new LinkedHashMap<>();

        for (SerialCompositeElementSymbol stream : getArchitecture().getStreams()) {
            members.putAll(getStreamLayerVariableMembers(stream, batchSize, true));
        }

        return members;
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

                // Add batch size dimension
                dimensions.add(0, "1");

                inputs.put(getName(element), dimensions);
            }
        }

        inputs.putAll(getStreamLayerVariableMembers(stream, "1", false));

        return inputs;
    }

    private Map<String, List<String>> getUnrollInputs(UnrollSymbol unroll) {
        Map<String, List<String>> inputs = new LinkedHashMap<>();

        for (ArchitectureElementSymbol element : unroll.getBody().getFirstAtomicElements()) {
            if (element.isInput() || element.isOutput()) {
                List<Integer> intDimensions = element.getOutputTypes().get(0).getDimensions();

                List<String> dimensions = new ArrayList<>();
                for (Integer intDimension : intDimensions) {
                    dimensions.add(intDimension.toString());
                }

                // Add batch size dimension
                dimensions.add(0, "1");

                inputs.put(getName(element), dimensions);
            }
        }

        inputs.putAll(getStreamLayerVariableMembers(unroll.getBody(), "1", false));

        return inputs;
    }

    private Map<String, List<String>> getStreamLayerVariableMembers(SerialCompositeElementSymbol stream, String batchSize, boolean includeOutput) {
        Map<String, List<String>> members = new HashMap<>();

        List<ArchitectureElementSymbol> elements = stream.getSpannedScope().resolveLocally(ArchitectureElementSymbol.KIND);
        for (ArchitectureElementSymbol element : elements) {
            if (element instanceof VariableSymbol) {
                VariableSymbol variable = (VariableSymbol) element;

                if (variable.getType() == VariableSymbol.Type.LAYER && variable.getMember() == VariableSymbol.Member.NONE) {
                    LayerVariableDeclarationSymbol layerVariableDeclaration = variable.getLayerVariableDeclaration();

                    if (layerVariableDeclaration.getLayer().getDeclaration().isPredefined()) {
                        PredefinedLayerDeclaration predefinedLayerDeclaration =
                                (PredefinedLayerDeclaration) layerVariableDeclaration.getLayer().getDeclaration();

                        if (predefinedLayerDeclaration.isValidMember(VariableSymbol.Member.STATE)) {
                            String name = variable.getName() + "_state_";

                            List<Integer> intDimensions = predefinedLayerDeclaration.computeOutputTypes(
                                    layerVariableDeclaration.getLayer().getInputTypes(),
                                    layerVariableDeclaration.getLayer(),
                                    VariableSymbol.Member.STATE
                            ).get(0).getDimensions();

                            List<String> dimensions = new ArrayList<>();

                            for (Integer intDimension : intDimensions) {
                                dimensions.add(intDimension.toString());
                            }

                            // Add batch size dimension at index 1, since RNN states in Gluon have the format
                            // (layers, batch_size, units)
                            dimensions.add(1, batchSize);

                            members.put(name, dimensions);
                        }

                        if (includeOutput) {
                            if (predefinedLayerDeclaration.isValidMember(VariableSymbol.Member.OUTPUT)) {
                                String name = variable.getName() + "_output_";

                                List<Integer> intDimensions = predefinedLayerDeclaration.computeOutputTypes(
                                        layerVariableDeclaration.getLayer().getInputTypes(),
                                        layerVariableDeclaration.getLayer(),
                                        VariableSymbol.Member.OUTPUT
                                ).get(0).getDimensions();

                                List<String> dimensions = new ArrayList<>();

                                for (Integer intDimension : intDimensions) {
                                    dimensions.add(intDimension.toString());
                                }

                                // Add batch size dimension at index 0, since we use NTC format for RNN output in Gluon
                                dimensions.add(0, batchSize);

                                members.put(name, dimensions);
                            }
                        }
                    }
                }
            }
        }
        return members;
    }

}
